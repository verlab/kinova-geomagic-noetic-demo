#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Feed Geomagic Touch from Kinova state.

Publishes sensor_msgs/ChannelFloat32 on /arm/force_feedback (default), matching
geomagic_control omni_cartesian (see ~torque_mode on that node).

Modes (~source):
  kdl_gravity  — gravity torque via PyKDL, mapped to a 3D wrench with a linear
                 least-squares fit tau ≈ J_v^T f (demo / no F/T sensor).
  robot_wrench — use tool external wrench from /my_gen3/base_feedback (real arm).
"""
import rospy
import numpy as np
import PyKDL as kdl
import urdf_parser_py.urdf as urdf
from kdl_parser_py.urdf import treeFromUrdfModel
from sensor_msgs.msg import ChannelFloat32, JointState

try:
    from kortex_driver.msg import BaseCyclic_Feedback
except Exception:  # noqa: BLE001
    BaseCyclic_Feedback = None


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


class FeedbackNode(object):
    def __init__(self):
        self.robot_description_param = rospy.get_param("~robot_description_param", "/my_gen3/robot_description")
        self.joint_topic = rospy.get_param("~arm_joint_state_topic", "/my_gen3/base_feedback/joint_state")
        self.out_topic = rospy.get_param("~feedback_topic", "/arm/force_feedback")
        self.chain_root = rospy.get_param("~robot_chain_root", "base_link")
        self.chain_tip = rospy.get_param("~robot_chain_tip", "end_effector_link")
        self.source = rospy.get_param("~source", "kdl_gravity")
        self.force_scale = rospy.get_param("~force_scale", 0.12)
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.35)
        self.max_force = rospy.get_param("~max_force_n", 6.0)
        self.invert_x = rospy.get_param("~invert_x", True)
        self.wrench_topic = rospy.get_param("~wrench_topic", "/my_gen3/base_feedback")

        rxml = rospy.get_param(self.robot_description_param)
        robot = urdf.URDF.from_xml_string(rxml)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("KDL tree failed")
        self.chain = tree.getChain(self.chain_root, self.chain_tip)
        self.joint_names = _chain_joint_names(self.chain)
        self.nj = self.chain.getNrOfJoints()
        self.dyn = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self._jac = kdl.Jacobian(self.nj)
        self._tau_g = kdl.JntArray(self.nj)
        self._q = None
        self._f_prev = np.zeros(3)
        self._wrench = np.zeros(3)

        rospy.Subscriber(self.joint_topic, JointState, self._on_js, queue_size=1)
        if self.source == "robot_wrench":
            if BaseCyclic_Feedback is None:
                rospy.logfatal("robot_wrench requested but kortex_driver.msg not importable.")
                raise RuntimeError("missing kortex_driver")
            rospy.Subscriber(self.wrench_topic, BaseCyclic_Feedback, self._on_wrench, queue_size=1)

        self._pub = rospy.Publisher(self.out_topic, ChannelFloat32, queue_size=1)
        self._rate = rospy.Rate(rospy.get_param("~publish_hz", 50.0))

    def _on_wrench(self, msg):
        x = msg.base.tool_external_wrench_force_x
        y = msg.base.tool_external_wrench_force_y
        z = msg.base.tool_external_wrench_force_z
        if self.invert_x:
            self._wrench = np.array([-x, y, z])
        else:
            self._wrench = np.array([x, y, z])

    def _on_js(self, msg):
        self._q = _jnt_array(self.joint_names, dict(zip(msg.name, msg.position)))

    def _gravity_torque(self):
        self.dyn.JntToGravity(self._q, self._tau_g)
        return np.array([self._tau_g[i] for i in range(self.nj)])

    def _lstsq_force(self, tau):
        self.jac_solver.JntToJac(self._q, self._jac)
        # KDL Jacobian: rows 0-2 = linear velocity (J_v), rows 3-5 = angular velocity
        # tau = J_v^T * f  =>  f = (J_v^T)^+ * tau
        Jt = np.zeros((self.nj, 3))
        for j in range(self.nj):
            for r in range(3):
                Jt[j, r] = self._jac[r, j]
        f, *_ = np.linalg.lstsq(Jt, tau, rcond=None)
        return f

    def _compute_raw(self):
        if self.source == "robot_wrench":
            return self._wrench.copy()
        if self.source == "kdl_gravity":
            if self._q is None:
                return np.zeros(3)
            tau = self._gravity_torque()
            return self._lstsq_force(tau)
        rospy.logwarn_throttle(5.0, "Unknown ~source %s; publishing zeros.", self.source)
        return np.zeros(3)

    def run(self):
        rospy.loginfo("kdl_haptic_feedback source=%s -> %s", self.source, self.out_topic)
        while not rospy.is_shutdown():
            f = self._compute_raw()
            f = np.clip(f * self.force_scale, -self.max_force, self.max_force)
            self._f_prev = (1.0 - self.filter_alpha) * self._f_prev + self.filter_alpha * f
            msg = ChannelFloat32()
            # sensor_msgs/ChannelFloat32: single string name + float[] values (omni_cartesian uses values[0:3])
            msg.name = "fx_fy_fz"
            msg.values = [float(self._f_prev[0]), float(self._f_prev[1]), float(self._f_prev[2])]
            self._pub.publish(msg)
            self._rate.sleep()


def main():
    rospy.init_node("kdl_haptic_feedback")
    FeedbackNode().run()


if __name__ == "__main__":
    main()
