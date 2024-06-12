#!/usr/bin/env python3

import rclpy
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    Address = {1:[-0.1, .3], 2:[.85, .7], 3:[1.8, .7], 4:[.36, 1.2],
               5:[.8, 1.5], 6:[1.7, 1.7], 7:[.7, 2.3], 8:[.6, 3.1],
               9:[.7, 3.8], 10:[0.0, 4.5], 11:[.6, 4.5], 12:[1.45, 4.5],
               13:[-0.3, 5.1], 14:[.5, 5.4], 15:[1.55, 5.4]}

    # Set initial pose
    initial_pose = navigator.getPoseStamped(Address[12], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

# (2, 3) --> 12 --> S
# (2, 3) --> 12 --> S
# (2, 3) --> 12 --> S
# (2, 3) --> 12 --> W
# (1, 3) --> 15 --> S
# (1, 3) --> 15 --> S
# (1, 3) --> 15 --> S
# (1, 3) --> 15 --> S
# (1, 3) --> 15 --> S
# (1, 2) --> 14 --> E
# (2, 2) --> 11 --> E
# (3, 2) --> 9 --> E
# (4, 2) --> 8 --> E
# (5, 2) --> 7 --> E
# (6, 2) --> 5 --> N
# (6, 3) --> 6 --> E
# (7, 3) --> 3 --> N

    goal_pose.append(navigator.getPoseStamped(Address[12], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[12], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[12], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.SOUTH))

    goal_pose.append(navigator.getPoseStamped(Address[14], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[11], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[9], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[8], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[7], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[5], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[6], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[3], TurtleBot4Directions.NORTH))

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
