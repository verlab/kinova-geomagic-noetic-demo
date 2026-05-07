#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Cartesian teleop: Geomagic stylus pose -> Kinova Gen3 joint velocity (KDL + kortex)."""
import math

import numpy as np
import PyKDL as kdl
import rospy
import urdf_parser_py.urdf as urdf
from kdl_parser_py.urdf import treeFromUrdfModel
from kortex_driver.msg import JointSpeed
from kortex_driver.srv import SendJointSpeedsCommand, SendJointSpeedsCommandRequest
from sensor_msgs.msg import JointState
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


class TeleopNode(object):
    def __init__(self):
        self.robot_description_param = rospy.get_param("~robot_description_param", "/my_gen3/robot_description")
        self.geomagic_description_param = rospy.get_param("~geomagic_description_param", "/geomagic/robot_description")
        self.arm_joint_state_topic = rospy.get_param(
            "~arm_joint_state_topic", "/my_gen3/base_feedback/joint_state"
        )
        self.haptic_joint_topic = rospy.get_param("~haptic_joint_topic", "/geomagic/joint_states")
        self.button_topic = rospy.get_param("~button_topic", "/geomagic/button")
        self.robot_chain_root = rospy.get_param("~robot_chain_root", "base_link")
        self.robot_chain_tip = rospy.get_param("~robot_chain_tip", "end_effector_link")
        self.haptic_chain_root = rospy.get_param("~haptic_chain_root", "base")
        self.haptic_chain_tip = rospy.get_param("~haptic_chain_tip", "stylus")
        self.gain = rospy.get_param("~position_gain", 0.9)
        self.workspace_scale = rospy.get_param("~workspace_scale", 3.0)
        self.max_deg_s = rospy.get_param("~max_joint_speed_deg_s", 35.0)
        self.damping = rospy.get_param("~damping_lambda", 0.02)
        self.send_zero_when_disabled = rospy.get_param("~send_zero_when_disabled", True)
        self.robot_ns = rospy.get_param("~robot_service_ns", "my_gen3")
        self.enabled = False
        self._q_arm = None
        self._name_to_q = {}
        self._hap0 = None
        self._ee0 = None

        rxml = rospy.get_param(self.robot_description_param)
        gxml = rospy.get_param(self.geomagic_description_param)
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
        self._hap_joint_names = _chain_joint_names(self.hap_chain)
        n = self.arm_chain.getNrOfJoints()
        self._fk_arm = kdl.ChainFkSolverPos_recursive(self.arm_chain)
        self._fk_hap = kdl.ChainFkSolverPos_recursive(self.hap_chain)
        self._jac_arm = kdl.ChainJntToJacSolver(self.arm_chain)
        self._jac = kdl.Jacobian(n)
        self._last_qdot = np.zeros(n)

        rospy.loginfo("Arm joints (%d): %s", n, self.arm_joint_names)
        service = "/" + self.robot_ns.lstrip("/") + "/base/send_joint_speeds_command"
        rospy.loginfo("Waiting for %s", service)
        rospy.wait_for_service(service, timeout=120.0)
        self._set_speed = rospy.ServiceProxy(service, SendJointSpeedsCommand)
        rospy.Subscriber(self.arm_joint_state_topic, JointState, self._on_arm_state, queue_size=1)
        rospy.Subscriber(self.haptic_joint_topic, JointState, self._on_haptic_state, queue_size=1)
        rospy.Subscriber(self.button_topic, Int32MultiArray, self._on_button, queue_size=1)
        hz = rospy.get_param("~control_rate_hz", 40.0)
        rospy.Timer(rospy.Duration(1.0 / hz), self._on_control)

    def _base_linear_velocity(self, v_des):
        """Map desired EE linear velocity in base frame to joint velocities (damped LS)."""
        n = self.arm_chain.getNrOfJoints()
        self._jac_arm.JntToJac(self._q_arm, self._jac)
        J = np.zeros((6, n))
        for i in range(6):
            for j in range(n):
                J[i, j] = self._jac[i, j]
        # KDL Jacobian: rows 0-2 = linear velocity, rows 3-5 = angular velocity
        t = np.zeros(6)
        t[0] = v_des[0]
        t[1] = v_des[1]
        t[2] = v_des[2]
        jjt = J @ J.T + self.damping * np.eye(6)
        qdot = J.T @ np.linalg.solve(jjt, t)
        return qdot

    def _on_button(self, msg):
        if not msg.data or len(msg.data) < 2:
            return
        if msg.data[0] == 1 and msg.data[1] == 0:
            self.enabled = True
            self._hap0 = None
            self._ee0 = None
            rospy.loginfo("Teleop enabled (Geomagic dark button).")
        if msg.data[1] == 1 and msg.data[0] == 0:
            self.enabled = False
            rospy.loginfo("Teleop disabled (Geomagic light button).")
            if self.send_zero_when_disabled:
                self._send_speeds(np.zeros(self.arm_chain.getNrOfJoints()))

    def _on_arm_state(self, msg):
        self._name_to_q = dict(zip(msg.name, msg.position))
        self._q_arm = _jnt_array(self.arm_joint_names, self._name_to_q)

    def _on_haptic_state(self, msg):
        if self._q_arm is None:
            return
        m = dict(zip(msg.name, msg.position))
        q_h = _jnt_array(self._hap_joint_names, m)
        T = kdl.Frame()
        if self._fk_hap.JntToCart(q_h, T) < 0:
            return
        p = np.array([T.p[0], T.p[1], T.p[2]])
        if self._hap0 is None:
            self._hap0 = p.copy()
            T0 = kdl.Frame()
            if self._fk_arm.JntToCart(self._q_arm, T0) < 0:
                return
            self._ee0 = np.array([T0.p[0], T0.p[1], T0.p[2]])
        p_des = self._ee0 + self.workspace_scale * (p - self._hap0)
        Tc = kdl.Frame()
        if self._fk_arm.JntToCart(self._q_arm, Tc) < 0:
            return
        p_cur = np.array([Tc.p[0], Tc.p[1], Tc.p[2]])
        err = p_des - p_cur
        v = self.gain * err
        if not self.enabled:
            self._last_qdot = np.zeros(self.arm_chain.getNrOfJoints())
            return
        self._last_qdot = self._base_linear_velocity(v)

    def _on_control(self, _evt):
        if not self.enabled or self._q_arm is None:
            return
        self._send_speeds(self._last_qdot)

    def _send_speeds(self, qdot_rad_s):
        max_r = self.max_deg_s * math.pi / 180.0
        q = np.clip(qdot_rad_s, -max_r, max_r)
        req = SendJointSpeedsCommandRequest()
        for i, w in enumerate(q):
            req.input.joint_speeds.append(JointSpeed(i, w * 180.0 / math.pi, 0))
        try:
            self._set_speed(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "send_joint_speeds: %s", e)

    def run(self):
        rospy.loginfo("geomagic_kinova_cartesian_teleop running.")
        rospy.spin()


def main():
    rospy.init_node("geomagic_kinova_cartesian_teleop")
    TeleopNode().run()


if __name__ == "__main__":
    main()
