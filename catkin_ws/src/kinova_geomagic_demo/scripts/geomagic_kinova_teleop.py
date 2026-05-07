#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Geomagic Touch <-> Kinova Gen3 7-DOF: joint-to-joint teleop with torque feedback.

Joint mapping (Geomagic 6DOF -> Kinova 7DOF):
  waist    -> joint_1   (base rotation)      [torque feedback]
  shoulder -> joint_2   (shoulder pitch)      [torque feedback]
             joint_3   (arm rotation — unmapped, holds position)
  elbow    -> joint_4   (elbow pitch)         [torque feedback]
  yaw      -> joint_5   (wrist rotation)
  pitch    -> joint_6   (wrist pitch)
  roll     -> joint_7   (tool rotation)

Force feedback (3DOF — Geomagic base joints only):
  Kinova actuator torques on joints 1,2,4 are scaled and applied as
  HD_CURRENT_JOINT_TORQUE (mNm) on Geomagic waist, shoulder, elbow.

Controls:
  Dark button  (button 0) — hold to enable teleop (clutch)
  Light button (button 1) — press to toggle gripper open/close

Requires omni_cartesian with torque_mode:=true.
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

# Geomagic Touch joint name -> Kinova joint index (0-based, matching actuator order)
DEFAULT_JOINT_MAP = [
    ("waist",    0),   # -> joint_1 (base rotation)
    ("shoulder", 1),   # -> joint_2 (shoulder pitch)
    ("elbow",    3),   # -> joint_4 (elbow pitch)
    ("yaw",      4),   # -> joint_5 (wrist rotation)
    ("pitch",    5),   # -> joint_6 (wrist pitch)
    ("roll",     6),   # -> joint_7 (tool rotation)
]

# Geomagic only has 3DOF force feedback (waist=0, shoulder=1, elbow=2)
# Map from Kinova actuator index to Geomagic torque channel (0-2)
TORQUE_FB_MAP = {0: 0, 1: 1, 3: 2}


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
        self.arm_js_topic = rospy.get_param("~arm_joint_state_topic", "/my_gen3/base_feedback/joint_state")
        self.hap_js_topic = rospy.get_param("~haptic_joint_topic", "/geomagic/joint_states")
        self.button_topic = rospy.get_param("~button_topic", "/geomagic/button")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/my_gen3/base_feedback")
        self.force_topic = rospy.get_param("~force_topic", "/arm/force_feedback")
        self.robot_ns = rospy.get_param("~robot_service_ns", "my_gen3")

        self.robot_chain_root = rospy.get_param("~robot_chain_root", "base_link")
        self.robot_chain_tip = rospy.get_param("~robot_chain_tip", "end_effector_link")

        # Joint velocity control
        self.joint_gain = rospy.get_param("~joint_gain", 1.5)
        self.joint_scale = rospy.get_param("~joint_scale", 3.0)
        self.max_deg_s = rospy.get_param("~max_joint_speed_deg_s", 25.0)

        # Torque feedback (values published in mNm for HD_CURRENT_JOINT_TORQUE)
        self.torque_scale = rospy.get_param("~torque_scale", 30.0)
        self.torque_filter_alpha = rospy.get_param("~torque_filter_alpha", 0.2)
        self.max_torque_mNm = rospy.get_param("~max_torque_mNm", 400.0)
        self.gravity_compensate = rospy.get_param("~gravity_compensate", True)

        self.n_kinova = 7
        control_hz = rospy.get_param("~control_rate_hz", 40.0)

        # ── KDL for gravity compensation ─────────────────────────────────────
        if self.gravity_compensate:
            rxml = rospy.get_param(self.robot_desc)
            rbot = urdf.URDF.from_xml_string(rxml)
            ok, rtree = treeFromUrdfModel(rbot)
            if not ok:
                raise RuntimeError("KDL tree (robot) failed")
            arm_chain = rtree.getChain(self.robot_chain_root, self.robot_chain_tip)
            self._arm_joint_names = _chain_joint_names(arm_chain)
            self._dyn = kdl.ChainDynParam(arm_chain, kdl.Vector(0, 0, -9.81))
            self._tau_g = kdl.JntArray(arm_chain.getNrOfJoints())
            rospy.loginfo("[teleop] Gravity compensation ON (%d joints)", arm_chain.getNrOfJoints())
        else:
            self._arm_joint_names = None
            self._dyn = None

        # ── State ────────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._q_arm_dict = {}
        self._q_hap_dict = {}
        self._hap_ref = None
        self._arm_ref = None
        self._actuator_torques = np.zeros(self.n_kinova)
        self._tau_fb_filtered = np.zeros(3)
        self.enabled = False
        self.gripper_closed = False
        self._btn_prev = [0, 0]

        # ── Joint mapping info ───────────────────────────────────────────────
        self._joint_map = DEFAULT_JOINT_MAP
        rospy.loginfo("[teleop] Joint mapping:")
        for hap_name, kin_idx in self._joint_map:
            fb_str = " [torque FB]" if kin_idx in TORQUE_FB_MAP else ""
            rospy.loginfo("  Geomagic %-10s -> Kinova joint_%d%s", hap_name, kin_idx + 1, fb_str)
        rospy.loginfo("  Kinova joint_3 — unmapped (holds position)")

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
        rospy.Subscriber(self.wrench_topic, BaseCyclic_Feedback, self._on_feedback, queue_size=1)

        # ── Publisher ────────────────────────────────────────────────────────
        self._torque_pub = rospy.Publisher(self.force_topic, ChannelFloat32, queue_size=1)

        # ── Control timer ────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0 / control_hz), self._control_loop)
        rospy.loginfo("[teleop] Ready. Dark button = clutch, Light button = gripper.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_arm_js(self, msg):
        with self._lock:
            self._q_arm_dict = dict(zip(msg.name, msg.position))

    def _on_hap_js(self, msg):
        with self._lock:
            self._q_hap_dict = dict(zip(msg.name, msg.position))

    def _on_feedback(self, msg):
        torques = np.zeros(self.n_kinova)
        for i, act in enumerate(msg.actuators):
            if i < self.n_kinova:
                torques[i] = act.torque
        with self._lock:
            self._actuator_torques = torques

    def _on_button(self, msg):
        if not msg.data or len(msg.data) < 2:
            return
        b0, b1 = msg.data[0], msg.data[1]

        if b0 == 1 and self._btn_prev[0] == 0:
            self.enabled = True
            with self._lock:
                self._hap_ref = dict(self._q_hap_dict)
                self._arm_ref = dict(self._q_arm_dict)
            rospy.loginfo("[teleop] ENABLED — anchored reference pose")
        elif b0 == 0 and self._btn_prev[0] == 1:
            self.enabled = False
            self._send_zero_speeds()
            rospy.loginfo("[teleop] DISABLED")

        if b1 == 1 and self._btn_prev[1] == 0:
            self.gripper_closed = not self.gripper_closed
            self._send_gripper(self.gripper_closed)
            rospy.loginfo("[teleop] Gripper %s", "CLOSED" if self.gripper_closed else "OPEN")

        self._btn_prev = [b0, b1]

    # ── Main control loop ────────────────────────────────────────────────────

    def _control_loop(self, _evt):
        with self._lock:
            q_hap = dict(self._q_hap_dict)
            q_arm_dict = dict(self._q_arm_dict)
            actuator_torques = self._actuator_torques.copy()
            hap_ref = self._hap_ref
            arm_ref = self._arm_ref

        if not q_arm_dict:
            return

        # ── Force feedback (always active) ───────────────────────────────
        self._update_torque_feedback(q_arm_dict, actuator_torques)

        # ── Joint velocity teleop ────────────────────────────────────────
        if not self.enabled or not q_hap or hap_ref is None or arm_ref is None:
            return

        qdot = np.zeros(self.n_kinova)
        for hap_name, kin_idx in self._joint_map:
            hap_current = q_hap.get(hap_name, 0.0)
            hap_zero = hap_ref.get(hap_name, 0.0)
            delta = hap_current - hap_zero

            arm_joint_name = "joint_%d" % (kin_idx + 1)
            arm_current = q_arm_dict.get(arm_joint_name, 0.0)
            arm_zero = arm_ref.get(arm_joint_name, 0.0)
            arm_desired = arm_zero + self.joint_scale * delta

            error = arm_desired - arm_current
            qdot[kin_idx] = self.joint_gain * error

        self._send_speeds(qdot)

    # ── Torque feedback ──────────────────────────────────────────────────────

    def _update_torque_feedback(self, q_arm_dict, actuator_torques):
        gravity_torques = np.zeros(self.n_kinova)
        if self.gravity_compensate and self._dyn is not None and self._arm_joint_names:
            q_kdl = _jnt_array(self._arm_joint_names, q_arm_dict)
            self._dyn.JntToGravity(q_kdl, self._tau_g)
            for i in range(min(len(self._arm_joint_names), self.n_kinova)):
                gravity_torques[i] = self._tau_g[i]

        external_torques = actuator_torques - gravity_torques

        raw_fb = np.zeros(3)
        for kin_idx, geo_ch in TORQUE_FB_MAP.items():
            if kin_idx < self.n_kinova:
                raw_fb[geo_ch] = external_torques[kin_idx] * self.torque_scale

        raw_fb = np.clip(raw_fb, -self.max_torque_mNm, self.max_torque_mNm)

        self._tau_fb_filtered = (
            (1.0 - self.torque_filter_alpha) * self._tau_fb_filtered
            + self.torque_filter_alpha * raw_fb
        )

        msg = ChannelFloat32()
        msg.name = "t0_t1_t2"
        msg.values = [
            float(self._tau_fb_filtered[0]),
            float(self._tau_fb_filtered[1]),
            float(self._tau_fb_filtered[2]),
        ]
        self._torque_pub.publish(msg)

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
        self._send_speeds(np.zeros(self.n_kinova))

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


def main():
    rospy.init_node("geomagic_kinova_teleop")
    GeomagicKinovaTeleop()
    rospy.spin()


if __name__ == "__main__":
    main()
