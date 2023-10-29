/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the MiniBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef MINIBOT_CONTROL__MINIBOT_HW_INTERFACE_H
#define MINIBOT_CONTROL__MINIBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/asio/serial_port.hpp>
using namespace::boost::asio;

namespace minibot_control
{

enum JointType { motor, servo };

class MiniBotJoint {
public:
  JointType type;
  MiniBotJoint(JointType t);
  MiniBotJoint();

  virtual std::string setPosition(double cmd) {
    return "";
  }

  virtual std::string setVelocity(double cmd) {
    return "";
  }
};

class MiniBotMotorJoint : public MiniBotJoint {
public:
  bool inverted;
  uint8_t port;
  double velocity_mult;
  double velocity_feed_forward;
  MiniBotMotorJoint(uint8_t port, double velocity_mult, double velocity_feed_forward, bool inverted = false);
  std::string setVelocity(double cmd);
};

class MiniBotServoJoint : public MiniBotJoint {
public:
  uint8_t port;
  MiniBotServoJoint(uint8_t port);
  std::string setPosition(double cmd);
};

/// \brief Hardware interface for a robot
class MiniBotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  MiniBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  std::map<std::string, std::shared_ptr<MiniBotJoint>> joints_;
  // CJH serial_port *p;
};  // class

}  // namespace minibot_control

#endif
