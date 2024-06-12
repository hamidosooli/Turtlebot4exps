#!/usr/bin/env python3

import rclpy
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    Address = {1:[-0.1, .26], 2:[.85, .7], 3:[1.8, .7], 4:[-.1, 1.5],
               5:[.6, 1.5], 55:[1., 1.6], 6:[1.55, 1.5], 7:[.8, 2.4], 8:[.7, 3.2],
               9:[.7, 3.9], 10:[0.0, 4.5], 11:[.5, 4.6], 12:[1.6, 4.7],
               13:[-0.2, 5.1], 14:[.4, 5.3], 15:[1.4, 5.6]}

    # Set initial pose
    initial_pose = navigator.getPoseStamped(Address[6], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

# (6, 3) --> 6 --> S
# (6, 2) --> 5 --> S
# (6, 1) --> 4 --> N
# (6, 1) --> 4 --> N
# (6, 1) --> 4 --> N
# (6, 1) --> 4 --> N
# (6, 1) --> 4 --> N
# (6, 1) --> 4 --> N
# (6, 2) --> 5 --> W
# (5, 2) --> 7 --> W
# (4, 2) --> 8 --> W
# (3, 2) --> 9 --> W
# (2, 2) --> 11 --> N
# (2, 3) --> 12 --> W
# (1, 3) --> 15 --> N

    goal_pose.append(navigator.getPoseStamped(Address[5], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[4], TurtleBot4Directions.NORTH))

    goal_pose.append(navigator.getPoseStamped(Address[55], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[7], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[8], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[9], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[11], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped(Address[12], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped(Address[15], TurtleBot4Directions.NORTH))

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
