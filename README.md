HomeServiceRobot
==================

# TLDR
This projects combines methods from robot **Localization (AMCL)**, **Mapping (SLAM)**, **Navigation (Dijkstra)** and implements them in **ROS** environment to drive robot to the goal locations in the unknown environment.

Here I simulated Turtlebot robot in the **Gazebo** environent and implemented the functionality of the *HomeServiceRobot* which goal is to pick up objects in one part of the home and moving it to another.

![gazebo environment](/screenshots/gazebo.jpg)


# Setup

```
mkdir -p HomeServiceRobot/catkin_ws/src
cd HomeServiceRobot/catkin_ws/src
git clone https://github.com/max-kazak/HomeServiceRobot.git .
catkin_init_workspace
cd ..
catkin_make
```

# Mapping
This project uses [gmapping](http://wiki.ros.org/gmapping) package that allows robot with laser range finder sensor and RGB-D camera to build the map of the surrounding environment while simultaneously localizing itself in it.

![slam_map](/screenshots/slam_map.jpg)

Terminal1 (running simulation and slam):

```
cd HomeServiceRobot/catkin_ws
source devel/setup.bash
./src/scripts/test_slam.sh
```

Using controls in the last opened _teleop_ terminal navigate robot in the environment until the map is complete in rviz window.

Terminal2 (to save map):

```
cd HomeServiceRobot/catkin_ws
source devel/setup.bash
rosrun map_server map_saver -f src/map/slam_map
```

# Localization
After the mapping process is complete robot can use Adaptive Monte-Carlo Localization to constanly localize itself on the map using laser sensors. To implement this method this project utilizes [AMCL](http://wiki.ros.org/amcl) ROS package.

Terminal (running localization test):

```
cd HomeServiceRobot/catkin_ws
source devel/setup.bash
./src/scripts/test_navigation.sh
```

# Navigation
To navigate robot to the goal on the map this project uses [ROS Navigation Stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) commands issued by **pick_objects** package.

**pick_objects** package simulates behavior of navigating robot to pickup site, picking virtual object, navigating to dropoff and dropping object at the goal loaction.

Terminal (running navigation test):

```
cd HomeServiceRobot/catkin_ws
source devel/setup.bash
./src/scripts/pick_objects.sh
```

# Combining all together
Finally, to simulate autonomous behavior of the HomeServiceRobot this project combines Localization and Navigation stacks which use map generated during SLAM tests and provides visual representation of the virtual objects with **add_markers** package.  

**add_markers** package subscibes to */home_robot_state* topic and through it receives robot state messages from **pick_objects** package. Based on the state of the robot **add_markers** then adds/removes markers from the simulation.

![rviz vizualization](/screenshots/rviz.png)

Terminal (running HomeServiceRobot simulation):

```
cd HomeServiceRobot/catkin_ws
source devel/setup.bash
./src/scripts/home_service.sh
```
