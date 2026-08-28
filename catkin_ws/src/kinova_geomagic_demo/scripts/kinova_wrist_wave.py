#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Kinova Gen3: stand the arm fully vertical, then wave the wrist.

Sequence:
  1. Clear faults and go to the Kortex zero pose (all joints 0°) — arm straight up.
  2. Loop forever: spherical-wrist pitch (joint 6 on 7-DoF, joint 5 on 6-DoF)
     to +90°, then back to 0°. At the vertical pose that pitch is a rotation
     of the tool about the robot-base X axis.

Ctrl+C sends a Stop command. Requires kortex_driver already running
(e.g. demo_kinova_only.launch) or launch demo_kinova_wrist_wave.launch.

Docker:
  docker compose run --rm demo \\
    roslaunch kinova_geomagic_demo demo_kinova_wrist_wave.launch
"""
import sys
import time

import rospy
from kortex_driver.msg import (
    ActionEvent,
    ActionNotification,
    AngularWaypoint,
    Waypoint,
    WaypointList,
)
from kortex_driver.srv import (
    Base_ClearFaults,
    ExecuteAction,
    ExecuteActionRequest,
    OnNotificationActionTopic,
    OnNotificationActionTopicRequest,
    Stop,
    ValidateWaypointList,
)


class KinovaWristWave(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.hold_sec = float(rospy.get_param("~hold_sec", 1.0))
        self.wrist_angle_deg = float(rospy.get_param("~wrist_angle_deg", 90.0))
        self.cycles = int(rospy.get_param("~cycles", 0))  # 0 = infinite
        self.action_timeout = float(rospy.get_param("~action_timeout_sec", 45.0))
        self.max_waypoint_duration = int(rospy.get_param("~max_waypoint_duration_sec", 30))

        ns = "/" + self.robot_name.lstrip("/")
        self.dof = int(rospy.get_param(ns + "/degrees_of_freedom", 7))
        # 1-based actuator index: spherical-wrist pitch (tilts the tool about base X at zero).
        self.wrist_joint = int(rospy.get_param("~wrist_joint", self.dof - 1))
        if self.wrist_joint < 1 or self.wrist_joint > self.dof:
            raise rospy.ROSException(
                "wrist_joint=%d is out of range for %d-DoF arm" % (self.wrist_joint, self.dof)
            )

        self.last_action_event = None
        rospy.Subscriber(ns + "/action_topic", ActionNotification, self._on_action)

        rospy.loginfo("[wrist_wave] Waiting for Kortex services on %s ...", ns)
        rospy.wait_for_service(ns + "/base/clear_faults", timeout=120.0)
        rospy.wait_for_service(ns + "/base/execute_action", timeout=120.0)
        rospy.wait_for_service(ns + "/base/validate_waypoint_list", timeout=120.0)
        rospy.wait_for_service(ns + "/base/activate_publishing_of_action_topic", timeout=120.0)
        rospy.wait_for_service(ns + "/base/stop", timeout=120.0)

        self._clear_faults = rospy.ServiceProxy(ns + "/base/clear_faults", Base_ClearFaults)
        self._execute_action = rospy.ServiceProxy(ns + "/base/execute_action", ExecuteAction)
        self._validate_waypoints = rospy.ServiceProxy(
            ns + "/base/validate_waypoint_list", ValidateWaypointList
        )
        self._activate_action_topic = rospy.ServiceProxy(
            ns + "/base/activate_publishing_of_action_topic", OnNotificationActionTopic
        )
        self._stop = rospy.ServiceProxy(ns + "/base/stop", Stop)

        rospy.on_shutdown(self._on_shutdown)

    def _on_action(self, notif):
        self.last_action_event = notif.action_event

    def _on_shutdown(self):
        try:
            self._stop()
        except rospy.ServiceException:
            pass

    def activate_notifications(self):
        try:
            self._activate_action_topic(OnNotificationActionTopicRequest())
        except rospy.ServiceException as exc:
            rospy.logerr("[wrist_wave] OnNotificationActionTopic: %s", exc)
            return False
        rospy.sleep(1.0)
        return True

    def clear_faults(self):
        try:
            self._clear_faults()
        except rospy.ServiceException as exc:
            rospy.logerr("[wrist_wave] ClearFaults: %s", exc)
            return False
        rospy.loginfo("[wrist_wave] Faults cleared")
        rospy.sleep(2.0)
        return True

    def wait_for_action(self):
        t0 = time.time()
        while not rospy.is_shutdown():
            if self.last_action_event == ActionEvent.ACTION_END:
                return True
            if self.last_action_event == ActionEvent.ACTION_ABORT:
                rospy.logerr("[wrist_wave] ACTION_ABORT")
                return False
            if time.time() - t0 > self.action_timeout:
                rospy.logerr("[wrist_wave] Action timed out after %.1f s", self.action_timeout)
                return False
            time.sleep(0.01)
        return False

    def send_joint_angles(self, angles_deg):
        """Send one angular waypoint (degrees, length = DoF)."""
        if len(angles_deg) != self.dof:
            rospy.logerr(
                "[wrist_wave] Expected %d joint angles, got %d", self.dof, len(angles_deg)
            )
            return False

        waypoint = Waypoint()
        angular = AngularWaypoint()
        for angle in angles_deg:
            angular.angles.append(float(angle))
        angular.duration = 0
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angular)

        trajectory = WaypointList()
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        duration = 0
        while duration < self.max_waypoint_duration:
            try:
                res = self._validate_waypoints(trajectory)
            except rospy.ServiceException as exc:
                rospy.logerr("[wrist_wave] ValidateWaypointList: %s", exc)
                return False
            n_err = len(res.output.trajectory_error_report.trajectory_error_elements)
            if n_err == 0:
                break
            duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = duration
        else:
            rospy.logerr("[wrist_wave] WaypointList invalid after %d s", self.max_waypoint_duration)
            return False

        self.last_action_event = None
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        try:
            self._execute_action(req)
        except rospy.ServiceException as exc:
            rospy.logerr("[wrist_wave] ExecuteAction: %s", exc)
            return False
        return self.wait_for_action()

    def vertical_pose(self):
        return [0.0] * self.dof

    def wrist_pose(self, angle_deg):
        q = self.vertical_pose()
        q[self.wrist_joint - 1] = float(angle_deg)
        return q

    def run(self):
        if not self.activate_notifications():
            return False
        if not self.clear_faults():
            return False

        rospy.loginfo(
            "[wrist_wave] %d-DoF arm: going vertical (all zeros), then joint %d  0° ↔ %.0f°",
            self.dof,
            self.wrist_joint,
            self.wrist_angle_deg,
        )
        if not self.send_joint_angles(self.vertical_pose()):
            rospy.logerr("[wrist_wave] Failed to reach vertical pose")
            return False
        rospy.loginfo("[wrist_wave] Arm is vertical. Starting wrist loop (Ctrl+C to stop).")
        rospy.sleep(self.hold_sec)

        cycle = 0
        while not rospy.is_shutdown():
            cycle += 1
            rospy.loginfo("[wrist_wave] Cycle %d: wrist → %.0f° (X)", cycle, self.wrist_angle_deg)
            if not self.send_joint_angles(self.wrist_pose(self.wrist_angle_deg)):
                return False
            rospy.sleep(self.hold_sec)

            rospy.loginfo("[wrist_wave] Cycle %d: wrist → 0°", cycle)
            if not self.send_joint_angles(self.wrist_pose(0.0)):
                return False
            rospy.sleep(self.hold_sec)

            if self.cycles > 0 and cycle >= self.cycles:
                rospy.loginfo("[wrist_wave] Finished %d cycles", cycle)
                return True
        return True


def main():
    rospy.init_node("kinova_wrist_wave")
    ok = KinovaWristWave().run()
    if not ok and not rospy.is_shutdown():
        rospy.logerr("[wrist_wave] Stopped with an error")
        sys.exit(1)


if __name__ == "__main__":
    main()
