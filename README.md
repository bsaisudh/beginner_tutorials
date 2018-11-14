[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/beginner_tutorials/blob/master/LICENSE)
# ENPM 808X Beginner Tutorial for ROS

## Overview

This tutorial explains the basic subscriber publisher pattern on ROS Kinetic. beginner_tutorials package includes talker node, listener node and related services and messages. It also contains transform frame broadcast example and rosbag record and play instructions. The different options are explained form command line point of view and roslaunch point of view. 

An example for Level 2 integration test using rostest and getest has been provided

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/bsaisudh/beginner_tutorials/blob/master/LICENSE) file for details

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* catkin
* Ubuntu 16.04
* Google test framework

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
rostest  

Note: Most of the packages are by default installed with ROS full development installation

## Steps for building package

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
git clone -b Week11_HW --recursive https://github.com/bsaisudh/beginner_tutorials.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make install
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```

### Running rostest

* Build package aling with rostest
```
cd path_to_catkin_workspace
catkin_make run_tests beginner_tutorials
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```
The test output should be
```
[ROSUNIT] Outputting test results to /home/bala/workspace/ROS/hello-terp-ws/build/test_results/beginner_tutorials/rostest-test_beginner_tutorials_test.xml
[Testcase: testbeginnerTutorialsTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-beginnerTutorialsTest/customMessageExistance][passed]
[beginner_tutorials.rosunit-beginnerTutorialsTest/toggleMessageExistance][passed]
[beginner_tutorials.rosunit-beginnerTutorialsTest/customMessagerun][passed]
[beginner_tutorials.rosunit-beginnerTutorialsTest/toggleMessageRun][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 4
 * ERRORS: 0
 * FAILURES: 0
```
* Runing rostest seperately
```
rostest beginner_tutorials beginner_tutorials_test.test
```
## Running tutorial form command line

* Running roscore and listener
```
// Run each command in a seperate terminal
roscore                                 // Start ROS master
rosrun beginner_tutorials listener        // Run listener node
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
* Recording messages on all topics for a duration of 10 seconds
```
rosbag record --duration=10 -a -O record_beginner_tutorials.bag
```
* Playing rosbag file
```
roscore		//run roscore
rosrun beginner_tutorials listener        // Run listener node  alone in seperate terminal
rosbag play record_beginner_tutorials.bag
```

### Tutorial Output running from command line

</p>
<p align="center">
<img src="/readme_images/Listener Talker.png">
</p>
</p>

### Tutorial Output running from command line - Rosbag Record

</p>
<p align="center">
<img src="/readme_images/Rosbag Record.png">
</p>
</p>

### Tutorial Output running from command line - Rosbag Play

</p>
<p align="center">
<img src="/readme_images/Rosbag Play.png">
</p>
</p>

## Running with launch file

* Both talker and listener can be launched at the same time using launch file using below command
  <br>Argument 1 - rate : The argument 'rate' specifies the loop rate of the talker node. It is optional and if not specified the default value is 10 Hz.'rate' argument accepts accepts value from the range \[1, 50) in Hz.
  <br>Argument 1 - record : The argument 'record' whether to record rosbag files or not. It is optional and if not specified the default value is false. The messages will be recorded for a duration of 20 seconds.
```
roslaunch beginner_tutorials beginner_tutorials.launch rate:=5 record:=1
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
* Playing rosbag file
```
roscore		//run roscore
rosrun beginner_tutorials listener        // Run listener node  alone in seperate terminal
rosbag play ./ros/record_beginner_tutorials.bag
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

## Working with tf Frames
* Run the beginner tutorials launch file. The talker will be broadcasting the transfromation with respect to world frame
* viewing the frames
```
rosrun tf view_frames
```
sample output
```
Listening to /tf for 5.000000 seconds
Done Listening
dot - graphviz version 2.38.0 (20140413.2041)

Detected dot version 2.38
frames.pdf generated
```
* The information on frames broadcasted can be viewed using echo command
```
rosrun tf tf_echo world talker
```
sample output
```
At time 1542156018.162
- Translation: [5.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, -0.000, 1.570]
            in RPY (degree) [0.000, -0.000, 89.954]
At time 1542156018.862
- Translation: [5.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, -0.000, 1.570]
            in RPY (degree) [0.000, -0.000, 89.954]
At time 1542156019.862
- Translation: [5.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, -0.000, 1.570]
            in RPY (degree) [0.000, -0.000, 89.954]

```
* Alternatively the same information can be for using rqt_tf_tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
</p>
<p align="center">
<img src="/readme_images/frames.png">
</p>
</p>
