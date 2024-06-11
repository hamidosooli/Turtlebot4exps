#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
        PoseWithCovarianceStamped,
        '/amcl_pose',
        self.listener_callback,
        qos_profile_sensor_data)
        self.subscription
        self.path = []

    def listener_callback(self, msg):
        p = msg.pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.get_clock().now().to_msg()

        self.pose.pose.position.x = p.pose.position.x
        self.pose.pose.position.y = p.pose.position.y
        self.pose.pose.orientation.z = p.pose.orientation.z
        self.pose.pose.orientation.w = p.pose.orientation.w

        self.path.append(self.pose)

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            qos_profile_sensor_data)
        self.subscription
        self.stop_button_pressed = False

    def joystick_callback(self, msg):
        # buttons indices -> 0: A(cross), 1: B(circle), 2: X(square), 3: Y(triangle)
        if msg.buttons[2] == 1:
            self.stop_button_pressed = True
            print("Stop button pressed.")

def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()
    navigator.waitUntilNav2Active()

    creating_path = True

    minimal_subscriber = MinimalSubscriber()
    joystick_controller = JoystickController()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.add_node(joystick_controller)

    try:
        print('Please use the teleoperation to move the robot, then press "△" to play the trajectory.')
        while creating_path:
            executor.spin_once(timeout_sec=0.1)
            if joystick_controller.stop_button_pressed:
                creating_path = False
    finally:
        initial_pose = minimal_subscriber.path[0]
        navigator.setInitialPose(initial_pose)
        navigator.startFollowWaypoints(minimal_subscriber.path)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
