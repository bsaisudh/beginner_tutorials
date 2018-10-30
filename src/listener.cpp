/*
 * @file listener.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 * @author Bala Murali Manoghar Sai Sudhakar
 * @brief This demonstrates simple sending of messages over the ROS system and depicts a listener node.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief Call back function that gets executed if there is a message in topic.
 * @param Pointer pointer to message object 
 * @return None
 */
void chatterCallback(const std_msgs::String& msg) {
  ROS_INFO_STREAM(msg.data);
}
/**
 * @brief Main block that runs the node.
 * @parms Number of command line arguments
 * @parms Pointer to command line arguments
 * @return Status of execution
 */
int main(int argc, char **argv) {
  // Initializing ROS node
  ros::init(argc, argv, "listener");
  // Creating node handle
  ros::NodeHandle n;
  // Setup loop rate
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // Check for call back
  ros::spin();
  return 0;
}
