/**
Software License Agreement (proprietary)

\file      motor_challenge_node.h
\authors    <rgariepy@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#ifndef MOTOR_CHALLENGE_MOTOR_CHALLENGE_NODE_H
#define MOTOR_CHALLENGE_MOTOR_CHALLENGE_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "motor_challenge/EncoderData.h"
#include "motor_challenge/MotorStatus.h"

class EncodersMonitor {
public:
  EncodersMonitor() {}
  EncodersMonitor(ros::NodeHandle* nh);

  // Callbacks receive inbound data
  void encodersCallback(const motor_challenge::EncoderDataConstPtr&);
  void motorsCallback(const motor_challenge::MotorStatusConstPtr&);
  void twistCallback(const geometry_msgs::TwistConstPtr&);

protected:
  ros::Subscriber sub_encoders_; 
  ros::Subscriber sub_motors_; 
  ros::Subscriber sub_twist_; 
  ros::Publisher pub_status_;
};

#endif
