/*
 * @file talkerTest.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 * @author Bala Murali Manoghar Sai Sudhakar
 * @brief Tests for talker node in beginner_tutorials package.
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

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/custom_message.h"

/**
 * @brief Testing the existence of custom message server
 */
TEST(talkerNodeService, customMessageExistance) {
  // Initializing node handle
  ros::NodeHandle nh;
  // Initialize service client
  auto client = nh.serviceClient<beginner_tutorials::custom_message>(
      "custom_message");
  // Wait for exitstance and test
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));;
}

/**
 * @brief Testing the existence of toggle message server
 */
TEST(talkerNodeService, toggleMessageExistance) {
  // Initializing node handle
  ros::NodeHandle nh;
  // Create client to request to service
  auto client = nh.serviceClient<std_srvs::Trigger>("toggle_msg");
  // Wait for existance and test
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief Testing custom message server call
 */
TEST(talkerNodeService, customMessagerun) {
  // Inititalize node handle
  ros::NodeHandle nh;
  // Create service client
  auto client = nh.serviceClient<beginner_tutorials::custom_message>(
      "custom_message");
  // Create message object
  beginner_tutorials::custom_message msg;
  // Assign message
  msg.request.message = "Go Green";
  // Call service
  client.call(msg.request, msg.response);
  // Verivy response
  EXPECT_STREQ("Message is changed", msg.response.response.c_str());
  EXPECT_TRUE(msg.response.success);
}

/**
 * @brief Testing toggle message server call
 */
TEST(talkerNodeService, toggleMessageRun) {
  // Initialize node handle
  ros::NodeHandle nh;
  // Create service client
  auto client = nh.serviceClient<std_srvs::Trigger>("toggle_msg");
  // Initialize message object
  std_srvs::Trigger msg;
  // Call service
  client.call(msg.request, msg.response);
  // Verify response 
  EXPECT_STREQ("Message is changed", msg.response.message.c_str());
  EXPECT_TRUE(msg.response.success);
}

