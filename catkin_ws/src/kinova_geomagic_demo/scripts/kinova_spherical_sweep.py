#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Finite, stop-at-each-pose J5/J6 sweep for the Gen3 7-DoF arm.

Defaults to an RViz-only preview. With execution_mode:=hardware, uses the
same angular-waypoint duration selection as kinova_wrist_wave.py.
See docs/SPHERICAL_SWEEP.md for the sequence, limits and Docker commands.
"""
import math
import sys
import threading
import time

import rospy
from kortex_driver.msg import (
    ActionEvent, ActionNotification, ActionType, AngularWaypoint,
    BaseCyclic_Feedback, Waypoint, WaypointList,
)
from kortex_driver.srv import (
    Base_ClearFaults, ExecuteAction, ExecuteActionRequest,
    OnNotificationActionTopic, OnNotificationActionTopicRequest,
    Stop, ValidateWaypointList,
)


def angular_delta(target, current):
    """Signed shortest angular distance in degrees, including 315 -> 0 = +45."""
    return (target - current + 180.0) % 360.0 - 180.0


def build_poses(first_tilt_deg=120.0):
    # 120 is below the manufacturer's SOFTWARE limit of 120.3 (guide r12, p109).
    # A smaller first ring can be selected explicitly; the other rings stay fixed.
    if not math.isfinite(first_tilt_deg) or not 90.0 < first_tilt_deg <= 120.0:
        raise ValueError("first_tilt_deg must be finite and in (90, 120]")
    poses = [[0.0] * 7]
    for tilt in (first_tilt_deg, 90.0, 60.0, 30.0, 0.0):
        for azimuth in range(0, 360, 45):
            poses.append([0.0, 0.0, 0.0, 0.0, float(azimuth), tilt, 0.0])
    poses.append([0.0, 0.0, 0.0, 0.0, 360.0, 0.0, 0.0])
    return poses


class KinovaSphericalSweep:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "my_gen3").strip("/")
        ns = "/" + self.robot_name
        if int(rospy.get_param(ns + "/degrees_of_freedom", 7)) != 7:
            raise ValueError("This sequence requires a Gen3 with 7 DoF")
        self.poses = build_poses(float(rospy.get_param("~first_tilt_deg", 120.0)))
        self.hold_sec = self.positive_param("hold_sec", 1.0)
        self.action_timeout = self.positive_param("action_timeout_sec", 45.0)
        self.settle_timeout = self.positive_param("settle_timeout_sec", 10.0)
        self.feedback_timeout = self.positive_param("feedback_timeout_sec", 0.5)
        self.position_tolerance = self.positive_param("position_tolerance_deg", 0.1)
        self.velocity_tolerance = self.positive_param("velocity_tolerance_deg_s", 0.5)
        self.max_duration = int(rospy.get_param("~max_waypoint_duration_sec", 30))
        if self.max_duration < 1 or self.max_duration >= self.action_timeout:
            raise ValueError("Require 0 < max_waypoint_duration_sec < action_timeout_sec")
        if self.position_tolerance > 0.2:
            raise ValueError("position_tolerance_deg must be <= 0.2 near the J6 limit")

        self.lock = threading.Lock()
        self.feedback = None
        self.action_id = 1000
        self.expected_action = None
        self.action_event = None
        self.motion_error = None
        self.j5_tracking = False
        self.j5_last = 0.0
        self.j5_travel = 0.0
        self.j5_peak = 0.0
        self.j5_expected = 0.0
        self._stop = None
        rospy.Subscriber(ns + "/action_topic", ActionNotification, self.on_action, queue_size=50)
        rospy.Subscriber(ns + "/base_feedback", BaseCyclic_Feedback,
                         self.on_feedback, queue_size=1)
        services = (
            ("_clear_faults", "/base/clear_faults", Base_ClearFaults),
            ("_execute", "/base/execute_action", ExecuteAction),
            ("_validate", "/base/validate_waypoint_list", ValidateWaypointList),
            ("_activate", "/base/activate_publishing_of_action_topic", OnNotificationActionTopic),
            ("_stop", "/base/stop", Stop),
        )
        for attr, suffix, service_type in services:
            rospy.wait_for_service(ns + suffix, timeout=120.0)
            setattr(self, attr, rospy.ServiceProxy(ns + suffix, service_type))
        rospy.on_shutdown(self.stop)

    @staticmethod
    def positive_param(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be finite and > 0")
        return value

    def stop(self):
        if self._stop is not None:
            try:
                self._stop()
            except rospy.ServiceException as exc:
                rospy.logerr("[spherical_sweep] Stop failed: %s", exc)

    def on_action(self, notification):
        with self.lock:
            if (notification.handle.identifier != self.expected_action or
                    notification.handle.action_type != ActionType.EXECUTE_WAYPOINT_LIST):
                return
            event = notification.action_event
            if event in (ActionEvent.ACTION_ABORT, ActionEvent.ACTION_PREPROCESS_ABORT,
                         ActionEvent.ACTION_POSTPROCESS_ABORT, ActionEvent.ACTION_PAUSE):
                self.motion_error = "Action failed/paused: event=%d, abort_details=%d" % (
                    event, notification.abort_details)
            # Do not let delayed progress events overwrite a terminal event.
            if event == ActionEvent.ACTION_END:
                self.action_event = event

    def on_feedback(self, message):
        received = time.monotonic()
        positions = tuple(a.position for a in message.actuators)
        velocities = tuple(a.velocity for a in message.actuators)
        with self.lock:
            if (len(positions) != 7 or
                    not all(math.isfinite(v) for v in positions + velocities)):
                self.motion_error = "Invalid feedback: expected seven finite joint states"
                return
            if self.j5_tracking:
                self.j5_travel += angular_delta(positions[4], self.j5_last)
                self.j5_last = positions[4]
                self.j5_peak = max(self.j5_peak, self.j5_travel)
                # Detect a controller taking the long/reverse route across 360/0.
                # 1 degree is a monitoring tolerance, not a manufacturer safety margin.
                if (self.j5_travel < self.j5_peak - 1.0 or
                        self.j5_travel > self.j5_expected + 1.0):
                    self.motion_error = "J5 moved outside the intended positive sweep"
            self.feedback = (received, positions, velocities)

    def state(self):
        with self.lock:
            if self.motion_error:
                raise RuntimeError(self.motion_error)
            feedback = self.feedback
        if feedback is None or time.monotonic() - feedback[0] > self.feedback_timeout:
            raise RuntimeError("Missing or stale base_feedback")
        return feedback

    def at_rest(self, positions, velocities, target):
        return (all(abs(angular_delta(q, p)) <= self.position_tolerance
                    for q, p in zip(target, positions)) and
                all(abs(v) <= self.velocity_tolerance for v in velocities))

    def wait_for_initial_feedback(self):
        deadline = time.monotonic() + self.settle_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if self.motion_error:
                    raise RuntimeError(self.motion_error)
                ready = self.feedback is not None
            if ready:
                self.state()
                return
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for seven-joint feedback")

    def send_pose(self, target, track_j5):
        _, positions, velocities = self.state()
        with self.lock:
            self.expected_action = None
            self.action_event = None
            self.j5_tracking = track_j5
            self.j5_last = positions[4]
            self.j5_travel = 0.0
            self.j5_peak = 0.0
            self.j5_expected = max(0.0, angular_delta(target[4], positions[4]))
        if self.at_rest(positions, velocities, target):
            return  # Already there: still perform the full dwell in run().

        waypoint = Waypoint()
        angular = AngularWaypoint()
        angular.angles = list(target)
        # Express wrap-around locally, e.g. 315 -> 360 instead of 315 -> 0.
        # Feedback checks also reject reverse/extra turns during each sweep step.
        if track_j5:
            angular.angles[4] = positions[4] + angular_delta(target[4], positions[4])
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angular)
        trajectory = WaypointList()
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)
        # Same policy as wrist_wave: 0, 1, ... 29 s; no speed/acceleration override.
        for duration in range(self.max_duration):
            angular.duration = duration
            self.state()
            errors = self._validate(trajectory).output.trajectory_error_report.trajectory_error_elements
            if not errors:
                break
        else:
            raise RuntimeError("Waypoint rejected by controller: %s" % errors)

        request = ExecuteActionRequest()
        request.input.name = "spherical_sweep"
        request.input.handle.identifier = self.action_id
        request.input.handle.action_type = ActionType.EXECUTE_WAYPOINT_LIST
        request.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        with self.lock:
            self.expected_action = self.action_id
        self.action_id += 1
        rospy.loginfo("[spherical_sweep] Validated duration: %d s", duration)
        deadline = time.monotonic() + self.action_timeout
        self._execute(request)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.state()
            with self.lock:
                if self.action_event == ActionEvent.ACTION_END:
                    return
            time.sleep(0.01)
        raise RuntimeError("Action timed out or execution interrupted")

    def hold_pose(self, target):
        """Require fresh, in-tolerance, stationary feedback for the whole dwell."""
        deadline = time.monotonic() + self.settle_timeout + self.hold_sec
        stable_since = None
        last_received = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            received, positions, velocities = self.state()
            if received != last_received:
                if self.at_rest(positions, velocities, target):
                    if stable_since is None:
                        stable_since = received
                    if received - stable_since >= self.hold_sec:
                        return
                else:
                    stable_since = None
                last_received = received
            time.sleep(0.01)
        raise RuntimeError("Pose did not remain stationary for the required dwell")

    def run(self):
        try:
            self.wait_for_initial_feedback()
            self._activate(OnNotificationActionTopicRequest())
            rospy.sleep(1.0)  # Same notification setup delay as wrist_wave.
            self._clear_faults()
            rospy.sleep(2.0)
            for index, target in enumerate(self.poses):
                if rospy.is_shutdown():
                    return False
                rospy.loginfo("[spherical_sweep] Pose %d/%d: J5=%.1f, J6=%.1f deg",
                              index + 1, len(self.poses), target[4], target[5])
                self.send_pose(target, track_j5=index > 0)
                self.hold_pose(target)
            rospy.loginfo("[spherical_sweep] Completed; all joints at zero (modulo 360)")
            return True
        except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("[spherical_sweep] %s", exc)
            return False
        finally:
            self.stop()



def visual_segment(start, target, fraction, duration):
    """Cubic joint interpolation; shortest positive wrap for this sweep."""
    u = min(1.0, max(0.0, fraction))
    blend = u * u * (3.0 - 2.0 * u)
    rate = 6.0 * u * (1.0 - u) / duration
    delta = [angular_delta(t, q) for q, t in zip(start, target)]
    return ([q + blend * d for q, d in zip(start, delta)],
            [rate * d for d in delta])


class RvizSphericalSweep:
    """Visualization only: JointState publisher, no Kortex clients or subscribers."""
    def __init__(self):
        from sensor_msgs.msg import JointState
        self.message_type = JointState
        self.poses = build_poses(float(rospy.get_param("~first_tilt_deg", 120.0)))
        self.hold_sec = KinovaSphericalSweep.positive_param("hold_sec", 1.0)
        self.duration = KinovaSphericalSweep.positive_param("visual_move_sec", 2.0)
        self.hz = KinovaSphericalSweep.positive_param("visual_rate_hz", 30.0)
        self.publisher = rospy.Publisher("/spherical_sweep_preview/joint_states",
                                         JointState, queue_size=1, latch=True)

    def publish(self, positions, velocities):
        message = self.message_type()
        message.header.stamp = rospy.Time.now()
        message.name = ["joint_%d" % i for i in range(1, 8)]
        message.position = [math.radians(q) for q in positions]
        message.velocity = [math.radians(v) for v in velocities]
        self.publisher.publish(message)

    def animate(self, start, target, duration, moving):
        began = time.monotonic()
        while not rospy.is_shutdown():
            elapsed = time.monotonic() - began
            positions, velocities = visual_segment(start, target, elapsed / duration, duration)
            if not moving:
                positions, velocities = list(start), [0.0] * 7
            self.publish(positions, velocities)
            if elapsed >= duration:
                return positions
            time.sleep(min(1.0 / self.hz, duration - elapsed))
        return None

    def run(self):
        current = [0.0] * 7
        self.publish(current, [0.0] * 7)
        # Give robot_state_publisher time to subscribe before playing the sequence.
        deadline = time.monotonic() + 10.0
        while self.publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                rospy.logerr("[sweep_preview] No joint-state subscriber")
                return False
            time.sleep(0.05)
        for index, target in enumerate(self.poses):
            if rospy.is_shutdown():
                return False
            rospy.loginfo("[sweep_preview] Pose %d/%d: J5=%.1f J6=%.1f deg",
                          index + 1, len(self.poses), target[4], target[5])
            if index:
                current = self.animate(current, target, self.duration, moving=True)
                if current is None:
                    return False
            if self.animate(current, current, self.hold_sec, moving=False) is None:
                return False
        rospy.loginfo("[sweep_preview] Finished. Holding final pose; Ctrl+C to close.")
        # Keep final TF available for late RViz subscribers, without replaying motion.
        while not rospy.is_shutdown():
            self.publish(current, [0.0] * 7)
            time.sleep(1.0 / self.hz)
        return True


def make_runner():
    mode = rospy.get_param("~execution_mode", "rviz")
    if mode == "rviz":
        return RvizSphericalSweep()
    if mode == "hardware":
        return KinovaSphericalSweep()
    raise ValueError("execution_mode must be 'rviz' or 'hardware'")


def main():
    rospy.init_node("kinova_spherical_sweep")
    try:
        ok = make_runner().run()
    except (ValueError, rospy.ROSException) as exc:
        rospy.logerr("[spherical_sweep] %s", exc)
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
