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
import numpy as np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from searchAlgs import ant_colony, random_walk, straight_walk

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    Address = np.array([[[0.0, 0.0],   [0.0, -0.3],   [0.0, -0.6],   [0.0, -0.9]],
                        [[-0.35, 0.0], [-0.35, -0.3], [-0.35, -0.6], [-0.35, -0.9]],
                        [[-0.65, 0.0], [-0.65, -0.3], [-0.65, -0.6], [-0.65, -0.9]],
                        [[-0.95, 0.0], [-0.95, -0.3], [-0.95, -0.6], [-0.95, -0.9]]])

    env_map = np.zeros((4, 4))
    env_mat = np.pad(env_map, pad_width=1, mode='constant', constant_values=np.nan)
    pos = [0, 0]
    moves = 30  # How many movements should the robot have
    # Set initial pose
    initial_pose = navigator.getPoseStamped(Address[pos[0], pos[1]], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

    for _ in range(moves):
        pos, orientation, env_mat = ant_colony(env_mat, env_map, pos, num_acts=4)
        # pos, orientation, env_mat = straight_walk(env_mat, env_map, pos, num_acts=4)
        # pos, orientation = random_walk(env_mat, env_map, pos, num_acts=4)
        goal_pose.append(navigator.getPoseStamped(Address[pos[0], pos[1]], orientation))

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
