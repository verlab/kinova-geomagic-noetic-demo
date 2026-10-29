"""Offline sequence/control tests; no ROS installation or hardware connection."""
import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


class ROSException(Exception):
    pass


class ServiceException(Exception):
    pass


class AngularWaypoint:
    def __init__(self):
        self.angles = []
        self.maximum_velocities = []
        self.duration = 0


class Waypoint:
    def __init__(self):
        self.oneof_type_of_waypoint = types.SimpleNamespace(angular_waypoint=[])


class WaypointList:
    def __init__(self):
        self.waypoints = []
        self.duration = 0
        self.use_optimal_blending = False


class ExecuteActionRequest:
    def __init__(self):
        self.input = types.SimpleNamespace(
            name='', handle=types.SimpleNamespace(identifier=0, action_type=0),
            oneof_action_parameters=types.SimpleNamespace(execute_waypoint_list=[]))


rospy = types.ModuleType('rospy')
rospy.ROSException = ROSException
rospy.ServiceException = ServiceException
rospy.get_param = Mock(side_effect=lambda name, default: default)
for name in ('Subscriber', 'wait_for_service', 'ServiceProxy', 'on_shutdown',
             'sleep', 'loginfo', 'logerr'):
    setattr(rospy, name, Mock())
rospy.is_shutdown = Mock(return_value=False)
msg = types.ModuleType('kortex_driver.msg')
msg.ActionEvent = types.SimpleNamespace(ACTION_END=1, ACTION_ABORT=2, ACTION_PAUSE=3,
                                      ACTION_START=4, ACTION_PREPROCESS_ABORT=6,
                                      ACTION_POSTPROCESS_ABORT=9)
msg.ActionType = types.SimpleNamespace(EXECUTE_WAYPOINT_LIST=41)
for cls in (AngularWaypoint, Waypoint, WaypointList):
    setattr(msg, cls.__name__, cls)
msg.ActionNotification = object
msg.BaseCyclic_Feedback = object
srv = types.ModuleType('kortex_driver.srv')
srv.ExecuteActionRequest = ExecuteActionRequest
for name in ('Base_ClearFaults', 'ExecuteAction', 'OnNotificationActionTopic',
             'OnNotificationActionTopicRequest', 'Stop', 'ValidateWaypointList'):
    setattr(srv, name, type(name, (), {}))
script = Path(__file__).resolve().parents[1] / 'catkin_ws/src/kinova_geomagic_demo/scripts/kinova_spherical_sweep.py'
spec = importlib.util.spec_from_file_location('sweep', script)
sweep = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'rospy': rospy, 'kortex_driver.msg': msg, 'kortex_driver.srv': srv}):
    spec.loader.exec_module(sweep)


def report(errors=()):
    return types.SimpleNamespace(output=types.SimpleNamespace(
        trajectory_error_report=types.SimpleNamespace(trajectory_error_elements=errors)))


def notification(identifier, event, action_type=41):
    return types.SimpleNamespace(handle=types.SimpleNamespace(
        identifier=identifier, action_type=action_type), action_event=event, abort_details=0)


class Clock:
    def __init__(self):
        self.now = 0.0
        self.tick = lambda: None

    def sleep(self, seconds):
        self.now += seconds
        self.tick()


class SweepTests(unittest.TestCase):
    def setUp(self):
        rospy.get_param.side_effect = lambda name, default: default
        rospy.is_shutdown.return_value = False
        self.clock = Clock()
        self.addCleanup(patch.stopall)
        patch.object(sweep.time, 'monotonic', lambda: self.clock.now).start()
        patch.object(sweep.time, 'sleep', self.clock.sleep).start()
        patch.object(rospy, 'sleep', self.clock.sleep).start()
        self.node = sweep.KinovaSphericalSweep()
        self.positions = [0.] * 7
        self.velocities = [0.] * 7
        self.clock.tick = self.publish
        self.publish()
        self.commands = []
        self.node._activate = Mock()
        self.node._clear_faults = Mock()
        self.node._stop = Mock()
        self.node._validate = Mock(side_effect=lambda tr: report(
            [] if tr.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration >= 2 else ['duration']))
        self.node._execute = Mock(side_effect=self.execute)

    def publish(self):
        self.node.on_feedback(types.SimpleNamespace(actuators=[
            types.SimpleNamespace(position=p, velocity=v)
            for p, v in zip(self.positions, self.velocities)]))

    def execute(self, request):
        tr = request.input.oneof_action_parameters.execute_waypoint_list[0]
        self.commands.append(copy.deepcopy(tr))
        self.positions = [q % 360 for q in tr.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].angles]
        self.publish()
        self.node.on_action(notification(request.input.handle.identifier, msg.ActionEvent.ACTION_END))

    def test_requested_sequence_and_paired_transitions(self):
        poses = sweep.build_poses()
        self.assertEqual(len(poses), 42)
        self.assertEqual(poses[0], [0.] * 7)
        for ring, tilt in enumerate((120, 90, 60, 30, 0)):
            group = poses[1 + ring * 8:9 + ring * 8]
            self.assertEqual([q[4] for q in group], list(range(0, 360, 45)))
            self.assertTrue(all(q[5] == tilt for q in group))
        for index in (9, 17, 25, 33):
            self.assertEqual(poses[index-1][4], 315)
            self.assertEqual(poses[index][4], 0)
            self.assertEqual(poses[index][5], poses[index-1][5] - 30)
        self.assertEqual(poses[-1], [0., 0., 0., 0., 360., 0., 0.])
        self.assertTrue(all(q[i] == 0 for q in poses for i in (0, 1, 2, 3, 6)))

    def test_full_run_holds_every_pose_and_ends(self):
        dwells = []
        original_hold = self.node.hold_pose
        def hold(target):
            before = self.clock.now
            original_hold(target)
            dwells.append(self.clock.now - before)
        self.node.hold_pose = hold
        self.assertTrue(self.node.run())
        self.assertEqual(len(dwells), 42)
        self.assertTrue(all(t >= 1.0 - 1e-9 for t in dwells))
        self.assertEqual(len(self.commands), 41)  # Initial all-zero pose already reached.
        for tr in self.commands:
            self.assertEqual(len(tr.waypoints), 1)
            self.assertFalse(tr.use_optimal_blending)
            wp = tr.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0]
            self.assertEqual(wp.duration, 2)
            self.assertEqual(wp.maximum_velocities, [])
        for index in (8, 16, 24, 32, 40):
            q = self.commands[index].waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].angles
            self.assertEqual(q[4], 360)
            self.assertEqual(q[5], {8:90, 16:60, 24:30, 32:0, 40:0}[index])
        self.assertEqual(self.positions, [0.] * 7)
        self.node._stop.assert_called_once()

    def test_invalid_configuration_before_movement(self):
        for value in (120.3, 121, 90, 0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                sweep.build_poses(value)
        self.assertEqual(sweep.build_poses(115)[1][5], 115)
        for key, value in (('/my_gen3/degrees_of_freedom', 6), ('~hold_sec', 0),
                           ('~position_tolerance_deg', .3), ('~hold_sec', float('nan'))):
            rospy.get_param.side_effect = lambda name, default: value if name == key else default
            with self.assertRaises(ValueError):
                sweep.KinovaSphericalSweep()

    def test_rejection_stops_without_advancing(self):
        self.node._validate = Mock(return_value=report(['joint limit']))
        self.assertFalse(self.node.run())
        self.node._execute.assert_not_called()
        self.node._stop.assert_called_once()
        self.assertEqual(self.node._validate.call_count, 30)

    def test_unrelated_events_and_late_progress_are_ignored(self):
        self.node.expected_action = 1001
        self.node.on_action(notification(1000, msg.ActionEvent.ACTION_END))
        self.node.on_action(notification(1001, msg.ActionEvent.ACTION_END, action_type=0))
        self.assertIsNone(self.node.action_event)
        self.node.on_action(notification(1001, msg.ActionEvent.ACTION_END))
        self.node.on_action(notification(1001, msg.ActionEvent.ACTION_START))
        self.assertEqual(self.node.action_event, msg.ActionEvent.ACTION_END)

    def test_abort_pause_and_preprocess_failure_stop(self):
        for event in (msg.ActionEvent.ACTION_ABORT, msg.ActionEvent.ACTION_PAUSE,
                      msg.ActionEvent.ACTION_PREPROCESS_ABORT):
            with self.subTest(event=event):
                self.node.motion_error = None
                self.node._execute.side_effect = lambda req: self.node.on_action(
                    notification(req.input.handle.identifier, event))
                self.assertFalse(self.node.run())
        self.assertEqual(self.node._stop.call_count, 3)

    def test_service_failure_stops(self):
        self.node._execute.side_effect = ServiceException('transport failed')
        self.assertFalse(self.node.run())
        self.node._stop.assert_called_once()

    def test_action_timeout_stops(self):
        self.node._execute.side_effect = None
        self.assertFalse(self.node.run())
        self.node._stop.assert_called_once()
        self.assertEqual(self.node._execute.call_count, 1)

    def test_feedback_loss_during_motion_stops(self):
        self.node._execute.side_effect = lambda request: setattr(self.clock, 'tick', lambda: None)
        self.assertFalse(self.node.run())
        self.assertEqual(self.node._execute.call_count, 1)
        self.node._stop.assert_called_once()

    def test_dwell_restarts_on_motion_or_position_error(self):
        for moving in (True, False):
            start = self.clock.now
            def tick():
                disturbed = .4 < self.clock.now - start < .6
                self.positions[0] = .2 if disturbed and not moving else 0
                self.velocities[0] = 1 if disturbed and moving else 0
                self.publish()
            self.clock.tick = tick
            self.node.hold_pose([0.] * 7)
            self.assertGreaterEqual(self.clock.now - start, 1.6)

    def test_nonzero_velocity_never_completes_dwell(self):
        self.velocities[6] = 1
        self.publish()
        with self.assertRaisesRegex(RuntimeError, 'stationary'):
            self.node.hold_pose([0.] * 7)

    def test_stale_or_invalid_feedback_cannot_finish(self):
        self.clock.tick = lambda: None
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.node.hold_pose([0.] * 7)
        for positions in ([0.] * 6, [float('nan')] * 7):
            self.node.motion_error = None
            self.positions = positions
            self.publish()
            with self.assertRaisesRegex(RuntimeError, 'seven finite'):
                self.node.state()

    def test_positive_wrap_and_reverse_motion_detection(self):
        self.positions[4] = 315
        self.publish()
        self.node.send_pose([0., 0., 0., 0., 0., 90., 0.], True)
        self.assertAlmostEqual(self.node.j5_travel, 45)
        self.assertIsNone(self.node.motion_error)
        self.positions[4] = 358  # Two degrees backwards after reaching 360/0.
        self.publish()
        with self.assertRaisesRegex(RuntimeError, 'positive sweep'):
            self.node.state()
        self.node.motion_error = None
        self.node.j5_last = 315
        self.node.j5_travel = 0
        self.node.j5_peak = 0
        self.positions[4] = 313
        self.publish()
        with self.assertRaisesRegex(RuntimeError, 'positive sweep'):
            self.node.state()

    def test_shutdown_does_not_send_next_pose(self):
        rospy.is_shutdown.return_value = True
        self.assertFalse(self.node.run())
        self.node._execute.assert_not_called()
        self.node._stop.assert_called_once()


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.addCleanup(patch.stopall)
        self.now = 0.0
        self.shutdown = False
        self.messages = []
        rospy.get_param.side_effect = lambda name, default: default
        patch.object(rospy, 'is_shutdown', lambda: self.shutdown).start()
        patch.object(sweep.time, 'monotonic', lambda: self.now).start()
        patch.object(sweep.time, 'sleep', self.sleep).start()
        self.publisher = Mock()
        self.publisher.get_num_connections.return_value = 1
        self.publisher.publish.side_effect = self.messages.append
        self.pub_factory = patch.object(rospy, 'Publisher', return_value=self.publisher, create=True).start()
        patch.object(rospy, 'Time', types.SimpleNamespace(now=lambda: self.now), create=True).start()
        sensor_msg = types.ModuleType('sensor_msgs.msg')
        sensor_msg.JointState = lambda: types.SimpleNamespace(header=types.SimpleNamespace(stamp=None))
        patch.dict(sys.modules, {'sensor_msgs.msg': sensor_msg}).start()
        self.service = patch.object(rospy, 'ServiceProxy', side_effect=AssertionError('hardware service')).start()
        self.subscriber = patch.object(rospy, 'Subscriber', side_effect=AssertionError('hardware subscription')).start()
        patch.object(rospy, 'wait_for_service', side_effect=AssertionError('hardware wait')).start()

    def sleep(self, duration):
        self.now += duration

    def test_visual_interpolation_wrap_and_zero_endpoint_velocity(self):
        start = [0.] * 7
        start[4], start[5] = 315., 120.
        target = [0.] * 7
        target[5] = 90.
        middle, velocity = sweep.visual_segment(start, target, .5, 2.)
        self.assertEqual(middle[4:6], [337.5, 105.])
        self.assertGreater(velocity[4], 0)
        self.assertLess(velocity[5], 0)
        end, velocity = sweep.visual_segment(start, target, 1., 2.)
        self.assertEqual(end[4:6], [360., 90.])
        self.assertEqual(velocity, [0.] * 7)

    def test_complete_preview_has_no_hardware_calls(self):
        def log(message, *args):
            if 'Finished.' in message:
                self.shutdown = True
        patch.object(rospy, 'loginfo', side_effect=log).start()
        runner = sweep.make_runner()  # Default must be RViz.
        self.assertIsInstance(runner, sweep.RvizSphericalSweep)
        self.assertTrue(runner.run())
        self.pub_factory.assert_called_once_with('/spherical_sweep_preview/joint_states',
                                                runner.message_type, queue_size=1, latch=True)
        self.assertAlmostEqual(self.now, 124., places=6)
        self.assertEqual(self.messages[-1].name, ['joint_%d' % i for i in range(1, 8)])
        self.assertAlmostEqual(self.messages[-1].position[4], 10 * sweep.math.pi)
        self.assertEqual(self.messages[-1].velocity, [0.] * 7)
        q5 = [m.position[4] for m in self.messages]
        self.assertTrue(all(b >= a - 1e-10 for a, b in zip(q5, q5[1:])))
        self.service.assert_not_called()
        self.subscriber.assert_not_called()

    def test_only_explicit_hardware_mode_selects_real_runner(self):
        with patch.object(sweep, 'KinovaSphericalSweep') as hardware:
            rospy.get_param.side_effect = lambda name, default: 'hardware'
            self.assertIs(sweep.make_runner(), hardware.return_value)
            hardware.assert_called_once()
            hardware.reset_mock()
            rospy.get_param.side_effect = lambda name, default: 'typo'
            with self.assertRaises(ValueError):
                sweep.make_runner()
            hardware.assert_not_called()

    def test_shutdown_during_animation(self):
        runner = sweep.RvizSphericalSweep()
        self.shutdown = True
        self.assertIsNone(runner.animate([0.] * 7, [1.] * 7, 2., True))
        self.service.assert_not_called()


if __name__ == '__main__':
    unittest.main()
