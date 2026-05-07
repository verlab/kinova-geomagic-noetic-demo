#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Geomagic Touch <-> Kinova Gen3: Cartesian EE teleop with force feedback + gripper.

Teleop (Geomagic -> Kinova):
  Geomagic stylus EE position (FK) delta is workspace-scaled and mapped to a
  desired Kinova EE position.  Damped least-squares IK on the linear rows of
  the Jacobian produces joint velocities sent via SendJointSpeedsCommand.

Force feedback (Kinova -> Geomagic):
  tool_external_wrench from /my_gen3/base_feedback (already gravity-compensated
  by the Kinova firmware) is filtered and scaled to HD_CURRENT_FORCE via
  omni_cartesian (torque_mode:=false).  Zero force when no contact.

Controls:
  Dark button  (button 0) — hold to enable teleop (clutch)
  Light button (button 1) — press to toggle Robotiq 2F-85 gripper

Requires omni_cartesian with torque_mode:=false.
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
    GripperMode,
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

        # Teleop
        self.position_gain = rospy.get_param("~position_gain", 1.2)
        self.workspace_scale = rospy.get_param("~workspace_scale", 3.0)
        self.max_deg_s = rospy.get_param("~max_joint_speed_deg_s", 25.0)
        self.damping = rospy.get_param("~damping_lambda", 0.05)

        # Force feedback (published as N, omni_cartesian divides by 10 internally)
        self.force_scale = rospy.get_param("~force_scale", 3.0)
        self.force_deadband = rospy.get_param("~force_deadband", 0.5)
        self.force_filter_alpha = rospy.get_param("~force_filter_alpha", 0.15)
        self.max_feedback_force = rospy.get_param("~max_feedback_force", 25.0)

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

        # ── State ────────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._q_arm = None
        self._q_hap = None
        self._hap0 = None
        self._ee0 = None
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

        # ── ROS I/O ──────────────────────────────────────────────────────────
        rospy.Subscriber(self.arm_js_topic, JointState, self._on_arm_js, queue_size=1)
        rospy.Subscriber(self.hap_js_topic, JointState, self._on_hap_js, queue_size=1)
        rospy.Subscriber(self.button_topic, Int32MultiArray, self._on_button, queue_size=1)
        rospy.Subscriber(self.wrench_topic, BaseCyclic_Feedback, self._on_wrench, queue_size=1)
        self._force_pub = rospy.Publisher(self.force_topic, ChannelFloat32, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / control_hz), self._control_loop)
        rospy.loginfo("[teleop] Ready. Dark button = clutch, Light button = gripper.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_arm_js(self, msg):
        q = _jnt_array(self.arm_joint_names, dict(zip(msg.name, msg.position)))
        with self._lock:
            self._q_arm = q

    def _on_hap_js(self, msg):
        q = _jnt_array(self.hap_joint_names, dict(zip(msg.name, msg.position)))
        with self._lock:
            self._q_hap = q

    def _on_wrench(self, msg):
        w = np.array([
            msg.base.tool_external_wrench_force_x,
            msg.base.tool_external_wrench_force_y,
            msg.base.tool_external_wrench_force_z,
        ])
        with self._lock:
            self._wrench_raw = w

    def _on_button(self, msg):
        if not msg.data or len(msg.data) < 2:
            return
        b0, b1 = msg.data[0], msg.data[1]

        # Dark button: clutch (hold to enable)
        if b0 == 1 and self._btn_prev[0] == 0:
            self.enabled = True
            with self._lock:
                self._hap0 = None
                self._ee0 = None
            rospy.loginfo("[teleop] ENABLED")
        elif b0 == 0 and self._btn_prev[0] == 1:
            self.enabled = False
            self._send_zero_speeds()
            rospy.loginfo("[teleop] DISABLED")

        # Light button: toggle gripper
        if b1 == 1 and self._btn_prev[1] == 0:
            self.gripper_closed = not self.gripper_closed
            self._send_gripper(self.gripper_closed)
            rospy.loginfo("[teleop] Gripper %s", "CLOSED" if self.gripper_closed else "OPEN")

        self._btn_prev = [b0, b1]

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self, _evt):
        with self._lock:
            q_arm = self._q_arm
            q_hap = self._q_hap
            wrench = self._wrench_raw.copy()

        if q_arm is None:
            return

        # ── Force feedback (always active) ───────────────────────────────
        self._publish_force_feedback(wrench)

        # ── Cartesian teleop ─────────────────────────────────────────────
        if not self.enabled or q_hap is None:
            return

        hap_pos = self._fk_pos(self._fk_hap, q_hap)
        if hap_pos is None:
            return

        with self._lock:
            if self._hap0 is None:
                self._hap0 = hap_pos.copy()
                self._ee0 = self._fk_pos(self._fk_arm, q_arm)
                if self._ee0 is None:
                    return
            hap0 = self._hap0
            ee0 = self._ee0

        p_des = ee0 + self.workspace_scale * (hap_pos - hap0)
        p_cur = self._fk_pos(self._fk_arm, q_arm)
        if p_cur is None:
            return

        v_des = self.position_gain * (p_des - p_cur)
        qdot = self._ik_velocity(q_arm, v_des)
        self._send_speeds(qdot)

    def _publish_force_feedback(self, wrench):
        f = wrench * self.force_scale
        for i in range(3):
            if abs(f[i]) < self.force_deadband:
                f[i] = 0.0
        f = np.clip(f, -self.max_feedback_force, self.max_feedback_force)
        self._f_filtered = (
            (1.0 - self.force_filter_alpha) * self._f_filtered
            + self.force_filter_alpha * f
        )
        msg = ChannelFloat32()
        msg.name = "fx_fy_fz"
        msg.values = [float(self._f_filtered[0]), float(self._f_filtered[1]), float(self._f_filtered[2])]
        self._force_pub.publish(msg)

    # ── Kinematics ───────────────────────────────────────────────────────────

    def _fk_pos(self, solver, q):
        T = kdl.Frame()
        if solver.JntToCart(q, T) < 0:
            return None
        return np.array([T.p[0], T.p[1], T.p[2]])

    def _ik_velocity(self, q_arm, v_des):
        self._jac_solver.JntToJac(q_arm, self._jac)
        J = np.zeros((6, self.n_arm))
        for i in range(6):
            for j in range(self.n_arm):
                J[i, j] = self._jac[i, j]
        # KDL: rows 0-2 = linear, rows 3-5 = angular
        twist = np.zeros(6)
        twist[0], twist[1], twist[2] = v_des[0], v_des[1], v_des[2]
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
        req.input.mode = GripperMode.GRIPPER_POSITION
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 0.9 if close else 0.0
        req.input.gripper.finger.append(finger)
        try:
            self._grip_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "[teleop] send_gripper_command: %s", e)


def main():
    rospy.init_node("geomagic_kinova_teleop")
    GeomagicKinovaTeleop()
    rospy.spin()


if __name__ == "__main__":
    main()
