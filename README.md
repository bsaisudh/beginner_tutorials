[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/beginner_tutorials/blob/master/LICENSE)
# ENPM 808X Beginner Tutorial for ROS

## Overview

This tutorial explains the basic subscriber publisher pattern on ROS Kinetic. beginner_tutorials package includes talker node, listener node and related services and messages

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/bsaisudh/beginner_tutorials/blob/master/LICENSE) file for details

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* catkin
* Ubuntu 16.04

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
git clone -b Week10_HW -recursive https://github.com/bsaisudh/beginner_tutorials.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make install
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```
## Running tutorial form command line

* Running roscore and listener
```
// Run each command in a seperate terminal
roscore                                 // Start ROS master
rosrun beginner_tutorials listener        // Run talker node
// Press CTRL + C to terminate
```
* Running talker form
  'rosrun beginner_tutorials listener \[looprate\]' is the commandline format.
  \[looprate\] is optional parameter and accepts value from the range \[1, 50) in Hz.
  The default looprate is 10 Hz
```
// Run talker node at loop rate of 10 Hz
rosrun beginner_tutorials talker 10
```
* Calling service to change to different message
```
rosservice call /toggle_msg
```
  the output should be
  ```
  success : True
  message : Message is changed
  ```
* Calling service to change to different message
```
rosservice call /custom_message 'Fear the turtle'
```
  the output should be
  ```
  success : True
  response : Message is changed
  ```

### Tutorial Output running from command line

</p>
<p align="center">
<img src="/readme_images/Listener Talker.png">
</p>
</p>

## Running with launch file

* Both talker and listener can be launched at the same time using launch file using below command
  The argument 'rate' specifies the loop rate of the talker node. It is optional and if not specified the default value is 10 Hz.
  'rate' argument accepts accepts value from the range \[1, 50) in Hz.
```
roslaunch beginner_tutorials beginner_tutorials.launch rate:=5
```
* Calling service to change to different message from commandline
```
rosservice call /toggle_msg
```
  the output should be
  ```
  success : True
  message : Message is changed
  ```
* Calling service to change to different message from commandline
```
rosservice call /custom_message 'Fear the turtle'
```
  the output should be
  ```
  success : True
  response : Message is changed
  ```
### Tutorial Output running from launch file

</p>
<p align="center">
<img src="/readme_images/Listener Talker Launch File.png">
</p>
</p>


## ROS Graph

</p>
<p align="center">
<img src="/readme_images/Listener Talker Graph.png">
</p>
</p>

## Changing logger level using rqt_logger_level

* Run both talker and listener nodes
* Open rqt_logger_level
```
rosrun rqt_logger_level rqt_logger_level
```
* Select the  node and select logger level as needed form the GUI

</p>
<p align="center">
<img src="/readme_images/Logger Level.png">
</p>
</p>

## Viewing Log Messages

* Run both talker and listener nodes
* Set required logger levels
* Run rqt_console
```
rosrun rqt_console rqt_console
```
</p>
<p align="center">
<img src="/readme_images/Logger levels and console.png">
</p>
</p>


