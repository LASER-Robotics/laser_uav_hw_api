"""Exercise the executable over DDS with an isolated, controlled simulation clock."""

import math
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class FastLioBridgeTest(unittest.TestCase):
    def test_bridge(self):
        rclpy.init(args=[])
        node = rclpy.create_node('bridge_probe', namespace='/bridge_test')
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        correction_pub = node.create_publisher(
            Odometry, 'fast_lio/odometry', qos_profile_sensor_data)
        odometry_pub = node.create_publisher(
            Odometry, 'fast_lio/odometry_high_freq', qos_profile_sensor_data)
        received = []
        node.create_subscription(
            VehicleOdometry, 'fmu/in/vehicle_visual_odometry', received.append, 10)

        def spin_until(predicate, timeout=2.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if predicate():
                    return True
            return predicate()

        def spin_for(duration=0.08):
            spin_until(lambda: False, duration)

        sim_ns = 100_000_000_000

        def advance(delta=10_000_000):
            nonlocal sim_ns
            sim_ns += delta
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(sim_ns, 1_000_000_000)
            clock_pub.publish(clock)
            spin_for()

        def sample(offset=0):
            msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                sim_ns + offset, 1_000_000_000)
            msg.header.frame_id = 'uav7/world'
            msg.child_frame_id = 'uav7/livox_up/imu'
            msg.pose.pose.position.x = 1.0
            msg.pose.pose.position.y = 2.0
            msg.pose.pose.position.z = 3.0
            q = msg.pose.pose.orientation
            q.w = q.x = q.y = q.z = 0.5
            v = msg.twist.twist.linear
            v.x, v.y, v.z = 4.0, 5.0, 6.0
            w = msg.twist.twist.angular
            w.x, w.y, w.z = 0.1, 0.2, 0.3
            for i in range(6):
                msg.pose.covariance[i * 7] = float(i + 1)
            msg.twist.covariance[7] = 0.4
            msg.twist.covariance[14] = -1.0
            return msg

        def correct():
            correction_pub.publish(sample())
            spin_for()

        def reject(msg):
            count = len(received)
            odometry_pub.publish(msg)
            spin_for()
            self.assertEqual(len(received), count)

        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([
                sys.argv[1], '--ros-args', '-r', '__ns:=/bridge_test',
                '-p', 'uav_name:=uav7', '-p', 'use_sim_time:=true',
            ], stdout=log, stderr=subprocess.STDOUT)
            try:
                self.assertTrue(spin_until(lambda: (
                    odometry_pub.get_subscription_count() == 1
                    and correction_pub.get_subscription_count() == 1
                    and clock_pub.get_subscription_count() == 1
                    and node.count_publishers(
                        '/bridge_test/fmu/in/vehicle_visual_odometry') == 1), 5.0))
                advance()
                reject(sample())  # IMU-only propagation must not start fusion.
                correct()
                msg = sample()
                odometry_pub.publish(msg)
                self.assertTrue(spin_until(lambda: len(received) == 1))
                out = received[-1]
                self.assertEqual(list(out.position), [1.0, -2.0, -3.0])
                self.assertEqual(list(out.q), [0.5, 0.5, -0.5, -0.5])
                self.assertEqual(list(out.velocity), [4.0, -5.0, -6.0])
                for actual, expected in zip(out.angular_velocity, [0.1, -0.2, -0.3]):
                    self.assertAlmostEqual(actual, expected, places=6)
                self.assertEqual(out.pose_frame, VehicleOdometry.POSE_FRAME_FRD)
                self.assertEqual(out.velocity_frame, VehicleOdometry.VELOCITY_FRAME_BODY_FRD)
                self.assertEqual(out.timestamp_sample, sim_ns // 1000)
                self.assertEqual(out.timestamp, sim_ns // 1000)
                self.assertEqual(list(out.position_variance), [1.0, 2.0, 3.0])
                self.assertEqual(list(out.orientation_variance), [4.0, 5.0, 6.0])
                self.assertTrue(math.isnan(out.velocity_variance[0]))
                self.assertAlmostEqual(out.velocity_variance[1], 0.4, places=6)
                self.assertTrue(math.isnan(out.velocity_variance[2]))
                reject(msg)  # Duplicate timestamp.
                reject(sample(-1_000_000))  # Out of order.

                advance()
                invalid = sample()
                invalid.header.frame_id = 'uav8/world'
                reject(invalid)
                invalid = sample()
                invalid.child_frame_id = 'untransformed_sensor'
                reject(invalid)
                invalid = sample()
                invalid.pose.pose.orientation.w = 0.0
                invalid.pose.pose.orientation.x = 0.0
                invalid.pose.pose.orientation.y = 0.0
                invalid.pose.pose.orientation.z = 0.0
                reject(invalid)
                invalid = sample()
                invalid.pose.pose.position.x = math.nan
                reject(invalid)
                reject(sample(-1_000_000_000))
                reject(sample(100_000_000))

                advance(600_000_000)
                reject(sample())  # High-frequency output continues after LiDAR stops.
                correction_pub.publish(sample(-1_000_000_000))
                spin_for()
                reject(sample())  # Old corrections do not refresh the watchdog.
                correct()
                msg = sample()
                msg.child_frame_id = 'uav7/fcu'
                msg.twist.twist.linear.x = math.nan
                msg.twist.twist.angular.z = math.inf
                odometry_pub.publish(msg)
                self.assertTrue(spin_until(lambda: len(received) == 2))
                self.assertTrue(all(math.isnan(v) for v in received[-1].velocity))
                self.assertTrue(all(math.isnan(v) for v in received[-1].angular_velocity))
            except Exception:
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
