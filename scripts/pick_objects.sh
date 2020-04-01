#!/bin/sh
export ROBOT_INITIAL_POSE="-x 0 -y 0 -Y 3.14"
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch pick_objects pick_objects.launch"
