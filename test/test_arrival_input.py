# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Arrival follows the held navigation input and checks its coordinate frame."""
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from dji_serial_bridge.msg import RefSysStatus
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data


def test_arrival_uses_held_position():
    rclpy.init()
    node = rclpy.create_node('arrival_input_test')
    goals = []
    node.create_subscription(PointStamped, '/dji_serial_bridge/nav_goal', goals.append, 10)
    ref = node.create_publisher(
        RefSysStatus, '/dji_serial_bridge/ref_sys', qos_profile_sensor_data)
    held = node.create_publisher(Odometry, '/mcb/navigation_odom', 10)
    local = node.create_publisher(Odometry, '/localization/odom', 10)
    executable = Path(get_package_prefix('thornbots_nav')) / 'lib/thornbots_nav/waypoint_nav'
    config = Path(get_package_share_directory('thornbots_nav')) / 'config/route.yaml'
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config)])

    def spin(duration=.25):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None
            rclpy.spin_once(node, timeout_sec=.01)

    def pose(publisher, x, y, frame='map'):
        message = Odometry()
        message.header.frame_id = frame
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        publisher.publish(message)
        spin()

    try:
        deadline = time.monotonic() + 10
        while not ref.get_subscription_count() or not held.get_subscription_count():
            assert time.monotonic() < deadline
            spin(.02)
        status = RefSysStatus()
        status.robot_id = 7
        status.robot_hp, status.game_stage = 400, 4
        node.create_timer(.05, lambda: ref.publish(status))
        ref.publish(status)
        spin()
        pose(local, -5.25, -3.)
        assert not goals
        pose(held, -4., -3.)
        assert goals[-1].header.frame_id == 'map'
        assert abs(goals[-1].point.y + 3.) < 1e-9
        pose(local, -5.25, -3.)
        pose(held, -5.25, -3., frame='nav_odom')
        assert abs(goals[-1].point.y + 3.) < 1e-9
        pose(held, -5.25, -3.)
        assert abs(goals[-1].point.y + .635) < 1e-9
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
