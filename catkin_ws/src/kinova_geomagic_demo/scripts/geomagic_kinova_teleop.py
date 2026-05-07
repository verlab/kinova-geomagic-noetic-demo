#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Unified Geomagic Touch <-> Kinova Gen3 7-DOF teleoperation with haptic force feedback.

Controls:
  Dark button  (button 0) — hold to enable teleop (clutch); release to pause.
  Light button (button 1) — press to toggle gripper open/close.

Teleop (Geomagic -> Kinova):
  Geomagic stylus position (FK on /geomagic/joint_states) is mapped to a desired
  Kinova EE position via workspace-scaled delta coupling.  A damped least-squares
  IK on the linear rows of the 6x7 Jacobian produces joint velocities sent to
  the arm via SendJointSpeedsCommand.

Force feedback (Kinova -> Geomagic):
  Kinova tool_external_wrench from /my_gen3/base_feedback is low-pass filtered,
  scaled to the Geomagic's 3-DOF force range (~3.3 N peak, ~0.88 N continuous),
  and published on /arm/force_feedback (consumed by omni_cartesian node which
  applies it via HD_CURRENT_FORCE).

Requires:
  - omni_cartesian (geomagic_control) running with torque_mode:=false
  - kortex_driver running for the Kinova arm
"""
import math
import threading

import numpy as np
import PyKDL as kdl
import rospy
import urdf_parser_py.urdf as urdf
from kdl_parser_py.urdf import treeFromUrdfModel
from kortex_driver.msg import (
    BaseCyclic_Feedback,
    Finger,
    GripperCommand,
    Gripper,
    JointSpeed,
)
from kortex_driver.srv import (
    SendGripperCommand,
    SendGripperCommandRequest,
    SendJointSpeedsCommand,
    SendJointSpeedsCommandRequest,
)
from sensor_msgs.msg import ChannelFloat32, JointState
from std_msgs.msg import Int32MultiArray

GRIPPER_POSITION_MODE = 2


def _chain_joint_names(chain):
    names = []
    for i in range(chain.getNrOfSegments()):
        jt = chain.getSegment(i).getJoint()
        if jt.getTypeName() in ("None", "Fixed"):
            continue
        names.append(jt.getName())
    return names


def _jnt_array(names, name_to_pos):
    q = kdl.JntArray(len(names))
    for i, n in enumerate(names):
        q[i] = name_to_pos.get(n, 0.0)
    return q


class GeomagicKinovaTeleop:
    def __init__(self):
        # ── Parameters ──────────────────────────────────────────────────────
        self.robot_desc = rospy.get_param("~robot_description_param", "/my_gen3/robot_description")
        self.haptic_desc = rospy.get_param("~geomagic_description_param", "/geomagic/robot_description")
        self.arm_js_topic = rospy.get_param("~arm_joint_state_topic", "/my_gen3/base_feedback/joint_state")
        self.hap_js_topic = rospy.get_param("~haptic_joint_topic", "/geomagic/joint_states")
        self.button_topic = rospy.get_param("~button_topic", "/geomagic/button")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/my_gen3/base_feedback")
        self.force_topic = rospy.get_param("~force_topic", "/arm/force_feedback")
        self.robot_ns = rospy.get_param("~robot_service_ns", "my_gen3")

        self.robot_chain_root = rospy.get_param("~robot_chain_root", "base_link")
        self.robot_chain_tip = rospy.get_param("~robot_chain_tip", "end_effector_link")
        self.haptic_chain_root = rospy.get_param("~haptic_chain_root", "base")
        self.haptic_chain_tip = rospy.get_param("~haptic_chain_tip", "stylus")

        # Teleop gains
        self.position_gain = rospy.get_param("~position_gain", 1.2)
        self.workspace_scale = rospy.get_param("~workspace_scale", 3.0)
        self.max_deg_s = rospy.get_param("~max_joint_speed_deg_s", 25.0)
        self.damping = rospy.get_param("~damping_lambda", 0.05)

        # Force feedback
        self.force_scale = rospy.get_param("~force_scale", 0.15)
        self.force_filter_alpha = rospy.get_param("~force_filter_alpha", 0.25)
        self.max_feedback_force = rospy.get_param("~max_feedback_force_n", 2.5)

        control_hz = rospy.get_param("~control_rate_hz", 40.0)

        # ── KDL setup ───────────────────────────────────────────────────────
        rxml = rospy.get_param(self.robot_desc)
        gxml = rospy.get_param(self.haptic_desc)
        rbot = urdf.URDF.from_xml_string(rxml)
        gbot = urdf.URDF.from_xml_string(gxml)
        ok, rtree = treeFromUrdfModel(rbot)
        if not ok:
            raise RuntimeError("KDL tree (robot) failed")
        ok, gtree = treeFromUrdfModel(gbot)
        if not ok:
            raise RuntimeError("KDL tree (geomagic) failed")

        self.arm_chain = rtree.getChain(self.robot_chain_root, self.robot_chain_tip)
        self.hap_chain = gtree.getChain(self.haptic_chain_root, self.haptic_chain_tip)
        self.arm_joint_names = _chain_joint_names(self.arm_chain)
        self.hap_joint_names = _chain_joint_names(self.hap_chain)
        self.n_arm = self.arm_chain.getNrOfJoints()

        self._fk_arm = kdl.ChainFkSolverPos_recursive(self.arm_chain)
        self._fk_hap = kdl.ChainFkSolverPos_recursive(self.hap_chain)
        self._jac_solver = kdl.ChainJntToJacSolver(self.arm_chain)
        self._jac = kdl.Jacobian(self.n_arm)

        rospy.loginfo("[teleop] Arm joints (%d): %s", self.n_arm, self.arm_joint_names)
        rospy.loginfo("[teleop] Haptic joints (%d): %s", len(self.hap_joint_names), self.hap_joint_names)

        # ── State ────────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._q_arm = None
        self._q_hap = None
        self._hap0 = None
        self._ee0 = None
        self._last_qdot = np.zeros(self.n_arm)
        self._wrench_raw = np.zeros(3)
        self._f_filtered = np.zeros(3)
        self.enabled = False
        self.gripper_closed = False
        self._btn_prev = [0, 0]

        # ── Kortex services ──────────────────────────────────────────────────
        ns = "/" + self.robot_ns.lstrip("/")
        speed_srv = ns + "/base/send_joint_speeds_command"
        grip_srv = ns + "/base/send_gripper_command"
        rospy.loginfo("[teleop] Waiting for %s ...", speed_srv)
        rospy.wait_for_service(speed_srv, timeout=120.0)
        rospy.loginfo("[teleop] Waiting for %s ...", grip_srv)
        rospy.wait_for_service(grip_srv, timeout=120.0)
        self._speed_srv = rospy.ServiceProxy(speed_srv, SendJointSpeedsCommand)
        self._grip_srv = rospy.ServiceProxy(grip_srv, SendGripperCommand)

        # ── Subscribers ──────────────────────────────────────────────────────
        rospy.Subscriber(self.arm_js_topic, JointState, self._on_arm_js, queue_size=1)
        rospy.Subscriber(self.hap_js_topic, JointState, self._on_hap_js, queue_size=1)
        rospy.Subscriber(self.button_topic, Int32MultiArray, self._on_button, queue_size=1)
        rospy.Subscriber(self.wrench_topic, BaseCyclic_Feedback, self._on_wrench, queue_size=1)

        # ── Publisher ────────────────────────────────────────────────────────
        self._force_pub = rospy.Publisher(self.force_topic, ChannelFloat32, queue_size=1)

        # ── Control timer ────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0 / control_hz), self._control_loop)
        rospy.loginfo("[teleop] Ready. Dark button = clutch, Light button = gripper toggle.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_arm_js(self, msg):
        with self._lock:
            self._q_arm = _jnt_array(
                self.arm_joint_names, dict(zip(msg.name, msg.position))
            )

    def _on_hap_js(self, msg):
        with self._lock:
            self._q_hap = _jnt_array(
                self.hap_joint_names, dict(zip(msg.name, msg.position))
            )

    def _on_wrench(self, msg):
        fx = msg.base.tool_external_wrench_force_x
        fy = msg.base.tool_external_wrench_force_y
        fz = msg.base.tool_external_wrench_force_z
        with self._lock:
            self._wrench_raw = np.array([fx, fy, fz])

    def _on_button(self, msg):
        if not msg.data or len(msg.data) < 2:
            return
        b0, b1 = msg.data[0], msg.data[1]

        # Dark button (b0): clutch — hold to enable teleop
        if b0 == 1 and self._btn_prev[0] == 0:
            self.enabled = True
            with self._lock:
                self._hap0 = None
                self._ee0 = None
            rospy.loginfo("[teleop] ENABLED (dark button pressed)")
        elif b0 == 0 and self._btn_prev[0] == 1:
            self.enabled = False
            self._send_zero_speeds()
            rospy.loginfo("[teleop] DISABLED (dark button released)")

        # Light button (b1): toggle gripper on press
        if b1 == 1 and self._btn_prev[1] == 0:
            self.gripper_closed = not self.gripper_closed
            self._send_gripper(self.gripper_closed)
            state_str = "CLOSED" if self.gripper_closed else "OPEN"
            rospy.loginfo("[teleop] Gripper %s", state_str)

        self._btn_prev = [b0, b1]

    # ── Main control loop ────────────────────────────────────────────────────

    def _control_loop(self, _evt):
        with self._lock:
            q_arm = self._q_arm
            q_hap = self._q_hap
            wrench = self._wrench_raw.copy()

        if q_arm is None:
            return

        # -- Force feedback (always active, even when teleop is disabled) --
        f_scaled = np.clip(
            wrench * self.force_scale,
            -self.max_feedback_force,
            self.max_feedback_force,
        )
        self._f_filtered = (
            (1.0 - self.force_filter_alpha) * self._f_filtered
            + self.force_filter_alpha * f_scaled
        )
        self._publish_force(self._f_filtered)

        # -- Teleop --
        if not self.enabled or q_hap is None:
            return

        hap_pos = self._fk_position(self._fk_hap, q_hap)
        if hap_pos is None:
            return

        with self._lock:
            if self._hap0 is None:
                self._hap0 = hap_pos.copy()
                ee_pos = self._fk_position(self._fk_arm, q_arm)
                if ee_pos is None:
                    return
                self._ee0 = ee_pos.copy()
                rospy.loginfo(
                    "[teleop] Anchored: hap0=%s  ee0=%s",
                    np.round(self._hap0, 4),
                    np.round(self._ee0, 4),
                )
            hap0 = self._hap0.copy()
            ee0 = self._ee0.copy()

        p_des = ee0 + self.workspace_scale * (hap_pos - hap0)
        p_cur = self._fk_position(self._fk_arm, q_arm)
        if p_cur is None:
            return

        err = p_des - p_cur
        v_des = self.position_gain * err
        qdot = self._ik_linear_velocity(q_arm, v_des)
        self._send_speeds(qdot)

    # ── Kinematics helpers ───────────────────────────────────────────────────

    def _fk_position(self, fk_solver, q):
        T = kdl.Frame()
        if fk_solver.JntToCart(q, T) < 0:
            return None
        return np.array([T.p[0], T.p[1], T.p[2]])

    def _ik_linear_velocity(self, q_arm, v_des):
        """Damped least-squares IK: desired EE linear velocity -> joint velocities."""
        self._jac_solver.JntToJac(q_arm, self._jac)
        J = np.zeros((6, self.n_arm))
        for i in range(6):
            for j in range(self.n_arm):
                J[i, j] = self._jac[i, j]
        # KDL: rows 0-2 = linear, rows 3-5 = angular
        twist = np.zeros(6)
        twist[0] = v_des[0]
        twist[1] = v_des[1]
        twist[2] = v_des[2]
        JJt = J @ J.T + self.damping * np.eye(6)
        return J.T @ np.linalg.solve(JJt, twist)

    # ── Kortex commands ──────────────────────────────────────────────────────

    def _send_speeds(self, qdot_rad_s):
        max_r = self.max_deg_s * math.pi / 180.0
        q = np.clip(qdot_rad_s, -max_r, max_r)
        req = SendJointSpeedsCommandRequest()
        for i, w in enumerate(q):
            req.input.joint_speeds.append(
                JointSpeed(joint_identifier=i, value=w * 180.0 / math.pi, duration=0)
            )
        try:
            self._speed_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "[teleop] send_joint_speeds: %s", e)

    def _send_zero_speeds(self):
        self._send_speeds(np.zeros(self.n_arm))

    def _send_gripper(self, close):
        req = SendGripperCommandRequest()
        req.input.mode = GRIPPER_POSITION_MODE
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 0.85 if close else 0.0
        req.input.gripper.finger.append(finger)
        try:
            self._grip_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "[teleop] send_gripper_command: %s", e)

    def _publish_force(self, f):
        msg = ChannelFloat32()
        msg.name = "fx_fy_fz"
        msg.values = [float(f[0]), float(f[1]), float(f[2])]
        self._force_pub.publish(msg)


def main():
    rospy.init_node("geomagic_kinova_teleop")
    GeomagicKinovaTeleop()
    rospy.spin()


if __name__ == "__main__":
    main()
