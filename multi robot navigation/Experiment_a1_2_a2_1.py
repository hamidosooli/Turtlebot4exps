#!/usr/bin/env python3
import multiprocessing
from joblib import Parallel, delayed
import rclpy
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import os

goal_pose = {'tb4_1':[], 'tb4_2':[]}

def task(data):
    i = data[0]
    rdi = data[1][0]
    rns = data[1][1]
    os.environ['ROS_DOMAIN_ID'] = rdi

    os.environ['ROS_NAMESPACE'] = rns
    rclpy.init()
    navigator = TurtleBot4Navigator()
    navigator.startToPose(goal_pose[rns][i])
    rclpy.shutdown()

def main(ros_domain_id_list=['1', '5'], ros_namespace_list=['tb4_1', 'tb4_2']):

    Address = {'tb4_1':{1:[-0.1, .26], 2:[.85, .5], 3:[1.8, .7], 4:[.36, 1.2],
               5:[.8, 1.4], 6:[1.55, 1.4], 7:[.8, 2.2], 8:[.7, 2.85],
               9:[.7, 3.6], 10:[0.0, 4.5], 11:[.5, 4.5], 12:[1.45, 4.5],
               13:[-0.2, 5.1], 14:[.6, 5.4], 15:[1.55, 5.3]},
               'tb4_2':{1:[-0.1, .26], 2:[.85, .7], 3:[1.8, .7], 4:[-0.1, 1.4],
               5:[.6, 1.3], 55:[1., 1.3], 6:[1.55, 1.4], 7:[.8, 2.4], 8:[.7, 3.15],
               9:[.7, 3.9], 10:[0.0, 4.5], 11:[.6, 4.7], 12:[1.45, 4.5],
               13:[-0.2, 5.1], 14:[.4, 5.4], 15:[1.55, 5.5]}}

    init_pose = {'tb4_1': Address['tb4_1'][12], 'tb4_2': Address['tb4_2'][6]}

    # goal_pose = {'tb4_1':[], 'tb4_2':[]}
    rclpy.init()
    navigator = TurtleBot4Navigator()

    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][11], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][9], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][8], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][7], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][5], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][2], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))

    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][5], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][4], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][4], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][4], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][4], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][4], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][55], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][7], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][8], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][9], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][11], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][14], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))

    rclpy.shutdown()

    for rdi, rns in zip(ros_domain_id_list, ros_namespace_list):
        os.environ['ROS_DOMAIN_ID'] = rdi
        os.environ['ROS_NAMESPACE'] = rns
        rclpy.init()
        navigator = TurtleBot4Navigator()
        # Set initial pose
        initial_pose = navigator.getPoseStamped(init_pose[rns], TurtleBot4Directions.SOUTH)
        navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        navigator.waitUntilNav2Active()
        rclpy.shutdown()


    for i in range(len(goal_pose['tb4_1'])):
        Parallel(n_jobs=2)(delayed(task)((i, info)) for info in zip(ros_domain_id_list, ros_namespace_list))
        #
        # for rdi, rns in zip(ros_domain_id_list, ros_namespace_list):
        #
        #     os.environ['ROS_DOMAIN_ID'] = rdi
        #     os.environ['ROS_NAMESPACE'] = rns
        #     rclpy.init()
        #     navigator = TurtleBot4Navigator()
        #     navigator.startToPose(goal_pose[rns][i])
        #     rclpy.shutdown()



if __name__ == '__main__':
    main()
