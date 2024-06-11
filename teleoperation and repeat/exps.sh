#!/bin/bash
# change ROS_DOMAIN_ID according to your robot
export ROS_DOMAIN_ID=5
# run the following with a pregenerated map of your environment (this is for a 5sq ft map)
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=LARI_SQ.yaml &
# visualize the navigation in the map
ros2 launch turtlebot4_viz view_robot.launch.py &
# make sure to disable the safety of the robot
ros2 param set /motion_control safety_override full
ros2 param set /motion_control safety_override full
ros2 param set /motion_control safety_override full
# run the script
python3 test2.py
