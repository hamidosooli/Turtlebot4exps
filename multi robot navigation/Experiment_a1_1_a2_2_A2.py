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
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    # if not navigator.getDockedStatus():
    #     navigator.info('Docking before intialising pose')
    #     navigator.dock()

    Address = {1:[-0.1, .26], 2:[.85, .7], 3:[1.8, .7], 4:[.36, 1.2],
               5:[.6, 1.3], 6:[1.55, 1.4], 7:[.7, 2.3], 8:[.7, 3.05],
               9:[.7, 3.8], 10:[0.0, 4.5], 11:[.6, 4.5], 12:[1.45, 4.5],
               13:[-0.2, 5.1], 14:[.4, 5.3], 15:[1.55, 5.7]}

    # Set initial pose
    # initial_pose = navigator.getPoseStamped(Address[6], TurtleBot4Directions.SOUTH)
    # navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    # navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

# (6, 3) --> 6 --> S
# (6, 2) --> 5 --> W
# (5, 2) --> 7 --> W
# (4, 2) --> 8 --> W
# (3, 2) --> 9 --> W
# (2, 2) --> 11 --> W
# (1, 2) --> 14 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N
# (1,3) --> 15 --> N

    goal_pose.append(navigator.getPoseStamped(Address[5], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[7], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[8], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[9], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[11], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[14], TurtleBot4Directions.NORTH))

    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))
    # Undock
    # navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    # navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
