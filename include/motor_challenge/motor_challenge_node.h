/**
 * 
Software License Agreement (BSD)

\file      motor_challenge_node.h
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

#ifndef MOTOR_CHALLENGE_MOTOR_CHALLENGE_NODE_H
#define MOTOR_CHALLENGE_MOTOR_CHALLENGE_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "motor_challenge/EncoderData.h"
#include "motor_challenge/MotorStatus.h"
#include <queue>

/**
*  Following class implements an estimation of the variance of the data set based on n samples from the data.
*/
class SampleVarianceFilter
{
private:
    size_t population_size_;
    std::queue<double> samples;
    double sum;
public:
    SampleVarianceFilter():
        population_size_(1)
    {}
    void setSize(size_t population_size)
    {
        population_size_=population_size;
    }
    void add(double sample)
    {
        //time window sum using a queue
        samples.push(sample);
        sum+=samples.back();

         if (samples.size()>population_size_)
         {
         sum-=samples.front();
         samples.pop();
         }
    }

    double getVariance()
    {
        return sum/(static_cast<double>(samples.size())-1.0);

    }
};


class EncodersMonitor {
public:
  EncodersMonitor() {}
  EncodersMonitor(ros::NodeHandle* nh);

  // Callbacks receive inbound data
  void encodersCallback(const motor_challenge::EncoderDataConstPtr&);
  void motorsCallback(const motor_challenge::MotorStatusConstPtr&);
  void twistCallback(const geometry_msgs::TwistConstPtr&);

protected:
  double robot_width_;     // [m]
  double wheel_rad_;       // [m]
  int enc_ticks_;       // [ticks/rev] Encoder ticks per revolution
  double K_v_;             // [V/rad/s] Motor constant, assuming K_t is equal when using SI units
  double R_;               // [ohm] Motor winding resistance
  motor_challenge::EncoderData enc_data_;
  double time_enc_state_;  // [s] used for determining velocity
  motor_challenge::MotorStatus motor_status_;
  geometry_msgs::Twist twist_robot_0_;
  double left_velocity_;   // [m/s] linear velocity of the left wheel at point of contact with the ground
  double right_velocity_;  // [m/s] linear velocity of the right wheel at point of contact with  the ground


  double num_samples;       // number of samples used to estimate variance of the encoder data
  double encoder_variance;  // maximum allowed variance of the encoder data

  SampleVarianceFilter variance_left;
  SampleVarianceFilter variance_right;

  bool right_enc_ok;
  bool left_enc_ok;
  void updateEncoderStatus();

  ros::Subscriber sub_encoders_; 
  ros::Subscriber sub_motors_; 
  ros::Subscriber sub_twist_; 
  ros::Publisher  pub_status_;
#ifndef DEBUG
  ros::Publisher  pub_debug_;
#endif /*DEBUG*/
};


#endif
