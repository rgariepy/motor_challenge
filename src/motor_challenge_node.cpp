/**
Software License Agreement (BSD)

\file      motor_challenge_node.cpp
\authors    <rgariepy@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//TODO: Write this code in a testable fashion.

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
