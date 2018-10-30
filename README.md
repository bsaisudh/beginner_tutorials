## beginner_tutorials
Beginner tutorial for ROS

## Overview

This tutorial explains the basic subscriber publiser pattern on ROS Kinatic. This tutorial example has a talker node, listener node and related servieces and messages

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## ROS installation

ROS Kinetic is needed for tis tutorial. Visit the [ROS Kinetic installation](http://wiki.ros.org/kinetic/Installation) page and follow the steps.

## ROS package dependencies

cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy

Note: Most of the packages are by default installed with ROS full development installation

## Steps for building the package

* Install catkin 
```
sudo apt-get install ros-kinetic-catkin
```
* Setup Catkin Workspace
```
mkdir path_to_catkin_workspace
cd path_to_catkin_workspace
mkdir src
cd src
```
* Clone beginner_tutorials repository
```
cd path_to_catkin_workspace/src
git clone --recursive https://github.com/bsaisudh/beginner_tutorials.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make install
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```
* Running the tutorial
```
// Run each command in a seperate terminal
roscore                                 // Start ROS master
rosrun beginner_tutorials talker        // Run talker node
rosrun beginner_tutorials listener      // Run Listener node
// Press CTRL + C to terminate
```



