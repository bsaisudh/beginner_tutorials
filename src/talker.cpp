/*
 * @file talker.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 * @author Bala Murali Manoghar Sai Sudhakar
 * @brief This demonstrates simple sending of messages over the ROS system and depicts a talker node.
 */

/*
 * MIT License
 *
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <sstream>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/custom_message.h"

// Global variables
std::string outMsg = "Go Terps!";

/**
 * @brief service to change the output string to a custom message.
 * @param req reference to request object
 * @param resp reference to response object
 * @return boolean success or failure
 */
bool getMessage(beginner_tutorials::custom_message::Request &req,
                beginner_tutorials::custom_message::Response &resp) {
  outMsg = req.message;
  ROS_DEBUG_STREAM("Received service request");
  ROS_INFO_STREAM("Sending \"" << req.message << "\" as message");
  resp.response = "Message is changed";
  resp.success = true;
  return true;
}

/**
 * @brief service to change the output string to a different message.
 * @param req reference to request object
 * @param resp reference to response object
 * @return boolean success or failure
 */
bool toggleMessage(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &resp) {
  outMsg = "Go Green!";
  ROS_DEBUG_STREAM("Received service request");
  ROS_INFO_STREAM("Sending different message");
  resp.message = "Message is changed";
  resp.success = true;
  return true;
}
/**
 * @brief Main block that runs the node.
 * @param argc Number of command line arguments
 * @param argv Pointer to command line arguments
 * @return Status of execution
 */
int main(int argc, char **argv) {
// Initializing ROS node
  ros::init(argc, argv, "talker");
// Parse command line arguments
  std::stringstream ss;
  ss.exceptions(std::ios::failbit);
  ss << argv[1];
  int looprate;
// try and catch exception
  try {
    ss >> looprate;
  } catch (std::exception &e) {
    ROS_FATAL_STREAM("Command line argument error : "<< e.what());
    ROS_WARN_STREAM("Setting default looprate, looprate = 10 Hz");
    looprate = 10;
  }
// Handling loop rate errors
  if (looprate == 0) {
    ROS_ERROR_STREAM("Invalid looprate : " << looprate);
    ROS_WARN_STREAM("Setting default looprate, looprate = 10 Hz");
    looprate = 10;
  } else if (looprate > 50) {
    ROS_ERROR_STREAM("Invalid looprate : " << looprate);
    ROS_WARN_STREAM("Setting default looprate, looprate = 10 Hz");
  } else {
    ROS_INFO_STREAM("Setting looprate to " << looprate);
  }
// Creating node handle
  ros::NodeHandle n;
// Creating publisher object
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// Subscribe to service call
  ros::ServiceServer toggleServer = n.advertiseService("toggle_msg",
                                                       &toggleMessage);
  ros::ServiceServer customMsgServer = n.advertiseService("custom_message",
                                                          &getMessage);
// Setup loop rate
  ros::Rate loop_rate(looprate);
// Message counter
  int count = 1;
// Loop till ROS system is active
  while (ros::ok()) {
    // Creating message object
    std_msgs::String msg;
    // Creating string stream object
    std::stringstream ss;
    // Sending message to string stream
    ss << outMsg << " " << count;
    // Assign message to object
    msg.data = ss.str();
    // Printing on console output
    ROS_INFO_STREAM(msg.data.c_str());
    // Publish message to topic
    chatter_pub.publish(msg);
    // Warn every 100 messages
    if (count % 100 == 0) {
      ROS_WARN_STREAM("Count reached to : " << count);
    }
    // Check for call back
    ros::spinOnce();
    // Wait for required time
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
