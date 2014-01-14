/**
Software License Agreement (proprietary)

\file      motor_challenge_node.cpp
\authors    <rgariepy@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.

\description Given encoder data, motor status feedback, and commands to the robot, publishes TRUE on the
status topic if the encoders are good, FALSE if they are not.
TODO: Write this code in a testable fashion.
*/

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "motor_challenge/EncoderData.h"
#include "motor_challenge/MotorStatus.h"

#include "motor_challenge/motor_challenge_node.h"

/**
 * Separate ROS initialization step for better testability.
 */
EncodersMonitor::EncodersMonitor(ros::NodeHandle* nh)
{
    // Create subscribers and publishers
    sub_encoders_ = nh->subscribe("encoders", 1, &EncodersMonitor::encodersCallback, this);
    sub_motors_ = nh->subscribe("motors", 1, &EncodersMonitor::motorsCallback, this);
    sub_twist_ = nh->subscribe("twist", 1, &EncodersMonitor::twistCallback, this);
    pub_status_ = nh->advertise<std_msgs::Bool>("status", 1);
}

/**
 * New encoder data received
 * Expected at 20 Hz
 */
void EncodersMonitor::encodersCallback(const motor_challenge::EncoderDataConstPtr& encoders)
{
    ROS_INFO("New encoder data!");

    // Publish an arbitrary status update.
    std_msgs::Bool status;
    status.data = true;
    pub_status_.publish(status);
}

/**
 * New motor status update 
 * Expected at 20 Hz
 */
void EncodersMonitor::motorsCallback(const motor_challenge::MotorStatusConstPtr& motors)
{
    ROS_INFO("New motor data!");
}

/**
 * New twist update 
 * Expected at 10 Hz
 */
void EncodersMonitor::twistCallback(const geometry_msgs::TwistConstPtr& motors)
{
    ROS_INFO("New twist data!");
}

/**
 * Node entry point
 */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motor_challenge_node"); 

  ros::NodeHandle nh("");
  EncodersMonitor enc(&nh);

  ros::spin();
}








