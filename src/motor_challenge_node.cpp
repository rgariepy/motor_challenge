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
#include "motor_challenge/EncoderStatus.h"
#include "std_msgs/Float64MultiArray.h"

#include "motor_challenge/motor_challenge_node.h"

#include <iostream>
#include <cmath>

/**
 * Separate ROS initialization step for better testability.
 */
EncodersMonitor::EncodersMonitor(ros::NodeHandle* nh)
{
    // Create subscribers and publishers
    sub_encoders_ = nh->subscribe("encoders", 1, &EncodersMonitor::encodersCallback, this);
    sub_motors_ = nh->subscribe("motors", 1, &EncodersMonitor::motorsCallback, this);
    sub_twist_ = nh->subscribe("twist", 1, &EncodersMonitor::twistCallback, this);
    pub_status_ = nh->advertise<motor_challenge::EncoderStatus>("status", 1);
#ifndef DEBUG
    //only advertise debug topic if compiled with debug
    pub_debug_=nh->advertise<std_msgs::Float64MultiArray>("debug_data",1);
#endif /*DEBUG*/

    //load robot parameters
    if(ros::param::has("robot_width") &&
          ros::param::has("wheel_rad")&&
            ros::param::has("enc_ticks") &&
                ros::param::has("K_v")&&
            ros::param::has("R"))
    {
    nh->getParam("robot_width",robot_width_);
    nh->getParam("wheel_rad",wheel_rad_);
    nh->getParam("enc_ticks",enc_ticks_);
    nh->getParam("K_v",K_v_);
    nh->getParam("R",R_);
    }else
    {
        ROS_INFO("Robot model is not complete, check parameters server for: robot_width, wheel_rad, enc_ticks, K_v, R");
        ros::shutdown();
    }

    if (ros::param::has("num_samples"))
    {
        nh->getParam("num_samples",num_samples);
    }
    else
    {
        ROS_INFO("Checking variance on one sample");
        num_samples=1;
    }
    variance_left.setSize(num_samples);
    variance_right.setSize(num_samples);
    if (ros::param::has("encoder_variance"))
    {
        nh->getParam("encoder_variance",encoder_variance);
    }
    else
    { 
        // variance of 3 ticks by default
        encoder_variance=3.0*2*M_PI/(static_cast<double>(enc_ticks_))*wheel_rad_;
        ROS_INFO("The variance is set to %f", encoder_variance);
    }

    //initial believe that sensor are ok
    left_enc_ok=true;
    right_enc_ok=true;
    time_enc_state_=-1;

}

/**
 * New encoder data received
 * Expected at 20 Hz
 */
void EncodersMonitor::encodersCallback(const motor_challenge::EncoderDataConstPtr& encoders)
{
    double prev_time_enc_state=time_enc_state_;
    time_enc_state_=ros::Time::now().toSec();

     //sample data does not indicate timing issues in the encoder data, thus no check is implement for it
    //first sample is skipped to initialize velocity estimation based on encoder readings
    if(prev_time_enc_state!=-1)
    {

        //using numerical differentiation for simplicity. SVF will yield better result in general.
        double prev_left_angle=static_cast<double>(enc_data_.left_ticks)*2*M_PI/(static_cast<double>(enc_ticks_));
        double left_angle=static_cast<double>(encoders->left_ticks)*2*M_PI/(static_cast<double>(enc_ticks_));

        double prev_right_angle=static_cast<double>(enc_data_.right_ticks)*2*M_PI/(static_cast<double>(enc_ticks_));
        double right_angle=static_cast<double>(encoders->right_ticks)*2*M_PI/(static_cast<double>(enc_ticks_));

        double angular_velocity_left=(left_angle-prev_left_angle)/(time_enc_state_-prev_time_enc_state);
        left_velocity_= angular_velocity_left*wheel_rad_;

        double angular_velocity_right=(right_angle-prev_right_angle)/(time_enc_state_-prev_time_enc_state);
        right_velocity_= angular_velocity_right*wheel_rad_;

        updateEncoderStatus();
    }
    motor_challenge::EncoderStatus status;
    status.left_encoder_ok = left_enc_ok;
    status.right_encoder_ok = right_enc_ok;
    enc_data_=*encoders;
    pub_status_.publish(status);
}

/**
 * New motor status update 
 * Expected at 20 Hz
 */
void EncodersMonitor::motorsCallback(const motor_challenge::MotorStatusConstPtr& motors)
{
 motor_status_=*motors;

}

/**
 * New twist update 
 * Expected at 10 Hz
 */
void EncodersMonitor::twistCallback(const geometry_msgs::TwistConstPtr& twist_robot_0_ptr)
{
    twist_robot_0_=*twist_robot_0_ptr;
}


void EncodersMonitor::updateEncoderStatus()
{
    // assuming that robot has differential drive and robot_width is the distance between provided twist vector and  wheels position
    // the wheels are parallel to X axes of the twist coordinate frame
    double left_velocity_twist = twist_robot_0_.linear.x-robot_width_*twist_robot_0_.angular.z;
    //assuming that  motor status data arrive much faster then motor dynamics. In other words no transient effect are taken into account.
    double left_velocity_motor = (motor_status_.left_voltage-R_*motor_status_.left_current)/K_v_*wheel_rad_;
    //The assumptions above are made based on the  provided sample data. These in general are not true as it requires knowledge of the models of the robot and motors.
    double left_residual_twist = left_velocity_-left_velocity_twist;
    double left_residual_motor = left_velocity_-left_velocity_motor;


    //same comments as above
    double right_velocity_twist = twist_robot_0_.linear.x+robot_width_*twist_robot_0_.angular.z;
    double right_velocity_motor = (motor_status_.right_voltage-R_*motor_status_.right_current)/K_v_*wheel_rad_;
    double right_residual_twist = right_velocity_-right_velocity_twist;
    double right_residual_motor = right_velocity_-right_velocity_motor;

    // The concept is to estimate the variance of data obtained from various sources. If the variance exceed a predefined threshold the encoders a deemed broken.
    // No data voting or probabilistic checks are implemented as not data is available for it.
    // The encoders are  only considered broken while estimated variance is too high. In real life application it is not viable to assume self-repairing encoders, thus
    // encoders should be considered failed after first positive detection until repair reset.

    // using sum of variances for simplicity
    double left_sample=left_residual_twist*left_residual_twist+left_residual_motor*left_residual_motor;
    variance_left.add(left_sample);

    double right_sample=right_residual_twist*right_residual_twist+right_residual_motor*right_residual_motor;
    variance_right.add(right_sample);

    left_enc_ok=variance_left.getVariance()<encoder_variance;
    right_enc_ok=variance_right.getVariance()<encoder_variance;

#ifndef DEBUG
    // if compiled with debug  publish all calculations for plotting
    std::vector<double> debug_data(12);
    debug_data[0]=left_velocity_;
    debug_data[1]=left_velocity_twist;
    debug_data[2]=left_velocity_motor;
    debug_data[3]=left_residual_twist;
    debug_data[4]=left_residual_motor;
    debug_data[5]= left_enc_ok ? 0.0 : 1.0;

    debug_data[6]=right_velocity_;
    debug_data[7]=right_velocity_twist;
    debug_data[8]=right_velocity_motor;
    debug_data[9]=right_residual_twist;
    debug_data[10]=right_residual_motor;
    debug_data[11]= right_enc_ok ? 0.0 : 1.0;

    std_msgs::Float64MultiArray msg_debug;
    msg_debug.data=debug_data;
    pub_debug_.publish(msg_debug);
#endif /*DEBUG*/

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
