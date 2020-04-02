#!/bin/sh
export ROBOT_INITIAL_POSE="-x 0 -y 0 -Y 3.14"
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 10
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
# xterm -e " roslaunch gmapping slam_gmapping_pr2.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
