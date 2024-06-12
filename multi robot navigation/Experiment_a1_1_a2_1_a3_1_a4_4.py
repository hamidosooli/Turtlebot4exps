#!/usr/bin/env python3
import multiprocessing
from joblib import Parallel, delayed
import rclpy
import numpy as  np
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import os

goal_pose = {'tb4_1':[], 'tb4_2':[], 'tb4_3':[], 'tb4_4':[]}

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

def main(ros_domain_id_list=['1', '2', '3', '4'], ros_namespace_list=['tb4_1', 'tb4_2', 'tb4_3', 'tb4_4']):

    Address = {'tb4_1':{1:[-0.1, -.1], 2:[.85, .7], 3:[1.6, .5], 4:[.36, 1.2],
                          5:[.8, 1.5], 6:[1.8, 1.5], 7:[.8, 2.1], 8:[.7, 2.9],
                          9:[.7, 3.8], 10:[0.0, 4.5], 11:[.6, 4.6], 12:[1.5, 4.4],
                          13:[-0.3, 5.1], 14:[.7, 5.4], 15:[1.4, 5.4]},
               'tb4_2':{1:[-0.1, .26], 2:[.85, .7], 3:[1.8, .7], 4:[.36, 1.],
                          5:[.6, 1.3], 6:[1.55, 1.4], 7:[.5, 2.5], 8:[.6, 3.05],
                          9:[.6, 3.8], 10:[0.0, 4.5], 11:[.5, 4.7], 12:[1.45, 4.5],
                          13:[-0.2, 5.1], 14:[.4, 5.3], 15:[1.7, 5.5]},
               'tb4_3':{1:[-0.1, .4], 2:[.9, .5], 3:[1.8, .7], 4:[.36, 1.2],
                          5:[.8, 1.6], 6:[1.55, 1.4], 7:[1., 2.], 8:[.9, 2.8],
                          9:[.7, 3.6], 10:[0.0, 4.5], 11:[1., 4.4], 1111:[.6, 4.6], 12:[1.7, 4.5],
                          13:[-0.2, 5.2], 14:[.4, 5.3], 15:[1.55, 5.7]},
               'tb4_4':{1:[-0.1, .26], 2:[.85, .7], 3:[1.8, .7], 4:[.36, 1.2],
                          5:[1.1, 1.], 6:[1.55, 1.4], 7:[.8, 2.2], 8:[.7, 3.],
                          9:[.8, 3.8], 10:[-.3, 4.6], 11:[.8, 4.6], 12:[1.45, 4.5],
                          13:[-0.1, 5.6], 14:[.4, 5.3], 15:[1.55, 5.7]}}

    init_pose = {'tb4_1': Address['tb4_1'][12], 'tb4_2': Address['tb4_2'][6],
                 'tb4_3': Address['tb4_3'][10], 'tb4_4': Address['tb4_4'][4]}

    # goal_pose = {'tb4_1':[], 'tb4_2':[], 'tb4_3':[], 'tb4_4':[]}
    rclpy.init()
    navigator = TurtleBot4Navigator()

    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][12], TurtleBot4Directions.WEST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][15], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][15], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][15], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][15], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][15], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][11], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][9], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][8], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][7], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][5], TurtleBot4Directions.NORTH))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][6], TurtleBot4Directions.EAST))
    goal_pose['tb4_1'].append(navigator.getPoseStamped(Address['tb4_1'][3], TurtleBot4Directions.NORTH))

    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][6], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][6], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][5], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][7], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][8], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][9], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][11], TurtleBot4Directions.WEST))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][14], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))
    goal_pose['tb4_2'].append(navigator.getPoseStamped(Address['tb4_2'][15], TurtleBot4Directions.NORTH))

    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][10], TurtleBot4Directions.NORTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][10], TurtleBot4Directions.NORTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][10], TurtleBot4Directions.NORTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][11], TurtleBot4Directions.NORTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][12], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][1111], TurtleBot4Directions.EAST))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][9], TurtleBot4Directions.EAST))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][8], TurtleBot4Directions.EAST))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][7], TurtleBot4Directions.EAST))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][5], TurtleBot4Directions.EAST))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][2], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][1], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][1], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_3'].append(navigator.getPoseStamped(Address['tb4_3'][1], TurtleBot4Directions.SOUTH))

    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][5], TurtleBot4Directions.WEST))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][7], TurtleBot4Directions.WEST))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][8], TurtleBot4Directions.WEST))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][9], TurtleBot4Directions.WEST))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][11], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][10], TurtleBot4Directions.WEST))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))
    goal_pose['tb4_4'].append(navigator.getPoseStamped(Address['tb4_4'][13], TurtleBot4Directions.SOUTH))

    rclpy.shutdown()

    for i, (rdi, rns) in enumerate(zip(ros_domain_id_list, ros_namespace_list)):
        os.environ['ROS_DOMAIN_ID'] = rdi
        os.environ['ROS_NAMESPACE'] = rns
        rclpy.init()
        navigator = TurtleBot4Navigator()
        # Set initial pose
        if i < 2:
            initial_pose = navigator.getPoseStamped(init_pose[rns], TurtleBot4Directions.SOUTH)
        else:
            initial_pose = navigator.getPoseStamped(init_pose[rns], TurtleBot4Directions.NORTH)
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
