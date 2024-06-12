#!/usr/bin/env python3

import rclpy
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    Address = {1:[0.0, .7], 2:[.9, .8], 3:[1.8, .7], 4:[-0.1, 1.4],
               5:[.7, 1.5], 55:[1., 1.3], 6:[1.55, 1.4], 7:[.7, 2.2], 8:[.6, 3.0],
               9:[.6, 3.8], 10:[0.0, 4.5], 11:[.7, 4.7], 12:[1.45, 4.5],
               13:[-0.2, 5.3], 14:[.4, 5.4], 15:[1.55, 5.7]}

    # Set initial pose
    initial_pose = navigator.getPoseStamped(Address[10], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

# (2,1) --> 10 --> N
# (2,1) --> 10 --> N
# (2,1) --> 10 --> N
# (1,1) --> 13 --> E
# (1,1) --> 13 --> E
# (1,1) --> 13 --> E
# (2,1) --> 10 --> N
# (2,1) --> 10 --> N
# (2,2) --> 11 --> E
# (3,2) --> 9 --> E
# (4,2) --> 8 --> E
# (5,2) --> 7 --> E
# (6,2) --> 5 --> E
# (7,2) --> 2 --> S
# (7,1) --> 1 --> S

    goal_pose.append(navigator.getPoseStamped(Address[10], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[10], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[13], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[13], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[13], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[10], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[10], TurtleBot4Directions.NORTH))

    goal_pose.append(navigator.getPoseStamped(Address[11], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[9], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[8], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[7], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[5], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped(Address[2], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[1], TurtleBot4Directions.SOUTH))

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
