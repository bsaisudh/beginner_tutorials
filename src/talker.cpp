/*
 * @file talker.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 * @author Bala Murali Manoghar Sai Sudhakar
 * @brief This demonstrates simple sending of messages over the ROS system and depicts a talker node.
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief Main block that runs the node.
 * @param Number of command line arguments
 * @param Pointer to command line arguments
 * @return Status of execution
 */
int main(int argc, char **argv) {
  // Initializing ROS node
  ros::init(argc, argv, "talker");
  // Creating node handle
  ros::NodeHandle n;
  // Creating publisher object
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Setup loop rate
  ros::Rate loop_rate(10);
  // Message counter
  int count = 0;
  // Loop till ROS system is active
  while (ros::ok()) {
    // Creating message object
    std_msgs::String msg;
    // Creaing string stream object
    std::stringstream ss;
    // Sending message to string stream
    ss << "Hello Terps!! " << count;
    // Assign message to object
    msg.data = ss.str();
    // Printing on console output
    ROS_INFO_STREAM(msg.data.c_str());
    // Publish message to topic
    chatter_pub.publish(msg);
    // Check for call back
    ros::spinOnce();
    // Wait for required time
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
