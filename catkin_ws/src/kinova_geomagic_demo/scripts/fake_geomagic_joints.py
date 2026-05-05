#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Publish a repeating JointState for the Geomagic model when no hardware is connected."""
import math

import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("fake_geomagic_joints")
    pub = rospy.Publisher("/geomagic/joint_states", JointState, queue_size=1)
    names = ["waist", "shoulder", "elbow", "yaw", "pitch", "roll"]
    rate = rospy.Rate(rospy.get_param("~hz", 30.0))
    phase = rospy.get_param("~phase", 0.0)
    amp = rospy.get_param("~amplitude_rad", 0.08)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() + phase
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = [
            amp * math.sin(0.7 * t),
            0.6 + 0.05 * math.sin(0.5 * t),
            0.2 + 0.05 * math.cos(0.6 * t),
            0.1 * math.sin(0.4 * t),
            0.05 * math.sin(0.9 * t),
            0.05 * math.cos(0.8 * t),
        ]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
