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
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <minibot_control/minibot_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <minibot_control/interpolating_map.h>
#include <boost/asio/io_service.hpp>
#include <angles/angles.h>
#include <regex> //probably overkill but eh

using namespace::boost::asio;
using boost::asio::serial_port;

namespace minibot_control
{

  MiniBotJoint::MiniBotJoint(JointType t) {
    type = t;
  }

  MiniBotJoint::MiniBotJoint() {

  }

  // Motors

  MiniBotMotorJoint::MiniBotMotorJoint(uint8_t port, double velocity_mult, double velocity_feed_forward, bool inverted) : MiniBotJoint(motor) {
    this->type = motor;
    this->inverted = inverted;
    this->port = port;
    this->velocity_mult = velocity_mult;
    this->velocity_feed_forward = velocity_feed_forward;
  }

  std::string MiniBotMotorJoint::setVelocity(double cmd) {
    double output = (inverted ? -1.0 : 1.0) * ((cmd >= 0 ? 1 : -1) * velocity_feed_forward + velocity_mult * cmd);
    if (cmd == 0) {
      output = 0.0;
    }
    std::string toWrite = "m" + std::to_string(port) + ";" + std::to_string(output) + ";";
    return toWrite;
  }

  // Servos

  MiniBotServoJoint::MiniBotServoJoint(uint8_t port, double offset, bool inverted, double scale, double initial_position) : MiniBotJoint(motor), port(port), offset(offset), inverted(inverted), scale(scale) {
    this->initial_position = initial_position;
  }

  std::string MiniBotServoJoint::setPosition(double cmd) {
    std::string toWrite = "s" + std::to_string(port) + ";" + std::to_string(angles::to_degrees(((inverted ? -1.0 : 1.0) * cmd) + offset) / scale) + ";";
    return toWrite;
  }

  MiniBotDIOJoint::MiniBotDIOJoint(uint8_t pin) : MiniBotJoint(dio), pin(pin) {}

  std::string MiniBotDIOJoint::setPosition(double cmd) {
    std::string toWrite = "d" + std::to_string(pin) + ";" + std::to_string(cmd) + ";";
    return toWrite;
  }

  std::string MiniBotDIOJoint::getPosition() {
    std::string toWrite = "dio?" + std::to_string(pin);
    return toWrite;
  }

  JointType getType(std::string s) {
    ROS_INFO_STREAM("MOTOR TYPE " << s);
    if (s == "motor") {
      return motor;
    } else if (s == "servo") {
      return servo;
    } else if (s == "dio") {
      return dio;
    } else {
      ROS_WARN_STREAM("DEFAULTING TO MOTOR TYPE");
      return motor;
    }
  }

  MiniBotJoint* parseJoint(ros::NodeHandle nh, const std::string &n) {
    ros::NodeHandle rpnh(
        nh, n);
    std::size_t error = 0;
    std::string jointType;
    error += !rosparam_shortcuts::get(n, rpnh, "type", jointType);
    JointType t;
    t = getType(jointType);
    if (t == motor) {
      bool inverted;
      int port;
      double velocity_mult;
      double vyi;
      error += !rosparam_shortcuts::get(n, rpnh, "inverted", inverted);
      error += !rosparam_shortcuts::get(n, rpnh, "port", port);
      error += !rosparam_shortcuts::get(n, rpnh, "velocity_mult", velocity_mult);
      error += !rosparam_shortcuts::get(n, rpnh, "velocity_feed_forward", vyi);
      MiniBotMotorJoint* j = new MiniBotMotorJoint(port, velocity_mult, vyi, inverted);
      ROS_INFO_STREAM("Motor joint, name = " << n << ", port = " << port << ", inverted = " << inverted << ", vmult = " << velocity_mult);
      rosparam_shortcuts::shutdownIfError(n, error);
      return j;
    }
    else if (t == servo) {
      int port;
      double offset;
      double scale;
      bool inverted;
      double initial_position;
      error += !rosparam_shortcuts::get(n, rpnh, "inverted", inverted);
      error += !rosparam_shortcuts::get(n, rpnh, "port", port);
      error += !rosparam_shortcuts::get(n, rpnh, "offset", offset);
      error += !rosparam_shortcuts::get(n, rpnh, "scale", scale);
      error += !rosparam_shortcuts::get(n, rpnh, "initial_position", initial_position);
      MiniBotServoJoint* j = new MiniBotServoJoint(port, offset, inverted, scale, initial_position);
      j->setPosition(initial_position);
      ROS_INFO_STREAM("Servo joint, name = " << n << ", port = " << port << ", offset = " << offset << ", scale = " << scale << ", inverted = " << inverted << ", initial_position = " << initial_position);
      rosparam_shortcuts::shutdownIfError(n, error);
      return j;
    }
    else if (t == dio) {
      int pin;
      error += !rosparam_shortcuts::get(n, rpnh, "pin", pin);
      MiniBotDIOJoint* j = new MiniBotDIOJoint(pin);
      ROS_INFO_STREAM("DIO joint, name = " << n << ", pin = " << pin);
      rosparam_shortcuts::shutdownIfError(n, error);
      return j;
    }
    return new MiniBotJoint(t);
  }


  MiniBotHWInterface::MiniBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    io_service ios;

    ros::NodeHandle hwnh(nh, "hardware_interface");
    voltage_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(hwnh, "voltage", 100);
    dist_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(hwnh, "distance_sensor", 100);
    dio_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(hwnh, "dio", 100);

    imu_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::Imu>>(hwnh, "imu/data_raw", 100);
    mag_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::MagneticField>>(hwnh, "imu/mag", 100);

    std::size_t error = 0;
    std::string serial_port;
    error += !rosparam_shortcuts::get("hardware_interface", hwnh, "write_proto", write_proto);
    error += !rosparam_shortcuts::get("hardware_interface", hwnh, "websocket_logging_level", log_ws_);
    // ===== BT serial START =====
    if (write_proto == 0) {
      ROS_WARN_STREAM("Using bluetooth serial");
      error += !rosparam_shortcuts::get("hardware_interface", hwnh, "port", serial_port);
      p = new boost::asio::serial_port(ios, serial_port);

      try {
        p->set_option(boost::asio::serial_port_base::baud_rate(921600));
      } catch (boost::system::system_error::exception e) {
        ROS_ERROR_STREAM("error setting serial port baud rate");
      }
    } 
    // ===== BT serial END =====
    // ===== Websockets START ===== 
    else if (write_proto == 1) {
      ROS_WARN_STREAM("Using websockets");
      use_websocket_ = true;
      error += !rosparam_shortcuts::get("hardware_interface", hwnh, "websocket_uri", ws_uri_);
      ws_thread_ = std::thread(&MiniBotHWInterface::websocketConnect, this, ws_uri_);
    } 
    // ===== Websockets END =====
    else {
      ROS_ERROR_STREAM("Exiting: Invalid write_proto value " << write_proto);
      exit(1);
    }

    rosparam_shortcuts::shutdownIfError("MiniBotHWInterface", error);
    for (std::string n : joint_names_) {
      std::shared_ptr<MiniBotJoint> jp = std::shared_ptr<MiniBotJoint>(parseJoint(nh, n));
      joints_[n] = jp;
    }
    ROS_INFO_NAMED("minibot_hw_interface", "MiniBotHWInterface Ready.");
  }

  void MiniBotHWInterface::read(ros::Duration& elapsed_time)
  {
    if (use_websocket_ && ws_connected_) {
      
      // ROS_INFO_STREAM("Querying voltage");
      
      ws_client_.send(ws_hdl_, "voltage?", websocketpp::frame::opcode::text);
      ws_client_.send(ws_hdl_, "dio?5", websocketpp::frame::opcode::text);
      //ws_client_.send(ws_hdl_, "dist?", websocketpp::frame::opcode::text);

      // ROS_INFO_STREAM("Querying IMU");
      //ws_client_.send(ws_hdl_, "imu?", websocketpp::frame::opcode::text);
    }
  }

  void MiniBotHWInterface::setupWebsocketLogging() {
    using namespace websocketpp::log;
    
    if (!log_ws_) {
      ws_client_.clear_access_channels(alevel::all);
      ws_client_.set_access_channels(alevel::connect | alevel::disconnect);
    }

    ROS_INFO_STREAM("Set logging channels for WS");
  }

  void MiniBotHWInterface::websocketConnect(const std::string& uri) {
      setupWebsocketLogging();
      ws_client_.init_asio();
      ws_client_.set_open_handler(std::bind(&MiniBotHWInterface::on_open, this, std::placeholders::_1));
      ws_client_.set_message_handler(std::bind(&MiniBotHWInterface::on_message, this, std::placeholders::_1, std::placeholders::_2));
      ws_client_.set_close_handler(std::bind(&MiniBotHWInterface::on_close, this, std::placeholders::_1));
      
      websocketpp::lib::error_code ec;
      ROS_INFO_STREAM("URI IS " << uri);
      WebSocketClient::connection_ptr con = ws_client_.get_connection(uri, ec);
      if (ec) {
        ROS_ERROR_STREAM("WebSocket Error: " << ec.message());
        return;
      }
      
      ws_client_.connect(con);
      ws_client_.run();
  }

  void MiniBotHWInterface::on_open(websocketpp::connection_hdl hdl) {
      ws_hdl_ = hdl;
      ws_connected_ = true;
      ROS_INFO_STREAM("WebSocket connection opened.");
      const std::string inital_cmd = "ping"; 
      ws_client_.send(ws_hdl_, inital_cmd, websocketpp::frame::opcode::text);
  }

  void MiniBotHWInterface::on_message(websocketpp::connection_hdl hdl, WebSocketClient::message_ptr msg) {
      const std::string msg_str = msg->get_payload();
      // data is published as a string of {field};{value};
      const std::regex pattern("([a-z]+);([-\\d.]+);");

      // ROS_INFO_STREAM("Received message: " << msg_str);

      // modified from https://stackoverflow.com/a/35026140
      std::smatch match;
      std::string::const_iterator search_start(msg_str.cbegin());
      bool voltage_found = false;
      bool imu_found = false;
      bool dist_found = false; 
      bool dio_found = false;
      sensor_msgs::Imu imu_msg;
      sensor_msgs::MagneticField mag_msg;
      while (std::regex_search(search_start, msg_str.cend(), match, pattern))
      {
        // ROS_INFO_STREAM("Found a regex match of size " << match.size() << " with first entry " << match[0].str() << " " << match[1].str() << " " << match[2].str());
        if (match.size() == 3) {
          std::string type = match[1].str();
          float value = std::stof(match[2].str());
          if (type == "v") {
            voltage_pub_->msg_.data = value;
            voltage_found = true;
          } 
          else if (type == "dist") {
            dist_pub_->msg_.data = value;
            dist_found = true;
          } 
          else if (type == "pin") {
            // only one DIO, DO NOT MAKE MULTIPLE
            dio_pub_->msg_.data = value;
            dio_found = true;
          } else if (type == "imut") {
            // assuming ax always is included first with IMU data
            imu_found = true;
            unsigned long long time_value = std::stoull(match[2].str());
            imu_msg.header.stamp = ros::Time(time_value / 1000000.0d); // this is not the real ROS timestamp, it's the ESP32's time since boot
            // so we can integrate velocity correctly
            // TODO tune covariances
            imu_msg.header.frame_id = "base_link";
            imu_msg.angular_velocity_covariance[0*3+0] = 0.1;
            imu_msg.angular_velocity_covariance[1*3+1] = 0.1;
            imu_msg.angular_velocity_covariance[2*3+2] = 0.1;
            imu_msg.linear_acceleration_covariance[0*3+0] = 0.1;
            imu_msg.linear_acceleration_covariance[1*3+1] = 0.1;
            imu_msg.linear_acceleration_covariance[2*3+2] = 0.1;

            mag_msg.header.stamp = ros::Time(time_value / 1000000.0d);
            mag_msg.header.frame_id = "base_link";
            mag_msg.magnetic_field_covariance[0*3+0] = 1e-3;
            mag_msg.magnetic_field_covariance[1*3+1] = 1e-3;
            mag_msg.magnetic_field_covariance[2*3+2] = 1e-3;
          } else if (type == "ax") {
            // reported value is in g's, we need to convert to m/s^2
            imu_msg.linear_acceleration.x = value * 9.80665;
          } else if (type == "ay") {
            // reported value is in g's, we need to convert to m/s^2
            imu_msg.linear_acceleration.y = value * 9.80665;
          } else if (type == "az") {
            // reported value is in g's, we need to convert to m/s^2
            imu_msg.linear_acceleration.z = value * 9.80665;
          } else if (type == "vx") {
            // reported value is in deg/s, we need to convert to rad/s
            imu_msg.angular_velocity.x = value * (M_PI / 180.0f);
          } else if (type == "vy") {
            // reported value is in deg/s, we need to convert to rad/s
            imu_msg.angular_velocity.y = value * (M_PI / 180.0f);
          } else if (type == "vz") {
            // reported value is in deg/s, we need to convert to rad/s
            imu_msg.angular_velocity.z = value * (M_PI / 180.0f);
            // ROS_INFO_STREAM("this message is " << value * (M_PI / 180.0f) << ", ang vel z = " << imu_msg.angular_velocity.z);
          } else if (type == "mx") {
            // reported value is in uTeslas, we need Teslas (divide by 1000000)
            mag_msg.magnetic_field.x = value / 1000000.0f;
          } else if (type == "my") {
            // reported value is in uTeslas, we need Teslas (divide by 1000000)
            mag_msg.magnetic_field.y = value / 1000000.0f;
          } else if (type == "mz") {
            // reported value is in uTeslas, we need Teslas (divide by 1000000)
            mag_msg.magnetic_field.z = value / 1000000.0f;
          } else {
            ROS_ERROR_STREAM("Unknown data: " << type);
          }
        }
        search_start = match.suffix().first;
      }
      // ROS_INFO_STREAM("imu_msg is " << imu_msg.angular_velocity);
      // ROS_INFO_STREAM("Trying to lock voltage pub");
      if (voltage_found && voltage_pub_ && voltage_pub_->trylock()) {
        voltage_pub_->unlockAndPublish();
      }
      if (dist_found && dist_pub_ && dist_pub_->trylock()) {
        dist_pub_->unlockAndPublish();
      }
      if (dio_found && dio_pub_ && dio_pub_->trylock()) {
        dio_pub_->unlockAndPublish();
      }
      // ROS_INFO_STREAM("Trying to lock IMU pub");
      if (imu_found && imu_pub_ && imu_pub_->trylock()) {
        imu_pub_->msg_ = imu_msg;
        imu_pub_->unlockAndPublish();
      }
      if (imu_found && mag_pub_ && mag_pub_->trylock()) {
        mag_pub_->msg_ = mag_msg;
        mag_pub_->unlockAndPublish();
      }
  }

  void MiniBotHWInterface::on_close(websocketpp::connection_hdl hdl) {
    ws_connected_ = false;
    ROS_ERROR_STREAM("WebSocket connection closed. Attempting to reconnect...");
  }

  void MiniBotHWInterface::write(ros::Duration& elapsed_time)
  {
    ROS_INFO_STREAM_THROTTLE(10, "HW Interface running");
    //ROS_INFO_STREAM("Ran write function"); 
    // Safety
    enforceLimits(elapsed_time);

    std::lock_guard<std::mutex> lock(command_mutex_);
    //ROS_INFO_STREAM("Mutex locked");
    // Write to serial for each joint
    std::string command;
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
      if (joint_velocity_[joint_id] != joint_velocity_command_[joint_id]) {
        // polymorphism FTW
        auto thisJoint = joints_[joint_names_[joint_id]];
        command += thisJoint->setVelocity(joint_velocity_command_[joint_id]); // + " "
        joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
      }
      joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec(); // if doing position control, velocity command MUST BE ZERO

      if (joint_position_[joint_id] != joint_position_command_[joint_id]) {
        // polymorphism FTW
        auto thisJoint = joints_[joint_names_[joint_id]];
        command += thisJoint->setPosition(joint_position_command_[joint_id]); // + " "
        joint_position_[joint_id] = joint_position_command_[joint_id];
      }
    }
    if(command.find_first_not_of(' ') != std::string::npos) {
      ROS_INFO_STREAM("Sending command " << command << ")"); // TODO replace with actual write
    }
    else {
      return;
    }

    if (use_websocket_) {
      if (!ws_connected_) {
        ROS_ERROR_STREAM_THROTTLE(2, "Websocket not connected");
      }
      // connected and using websockets
      else {
        ws_client_.send(ws_hdl_, command, websocketpp::frame::opcode::text);
      }
    }  
    else if (write_proto == 0) {
      ROS_WARN_STREAM_THROTTLE(2, "Using BT serial");
      p->write_some(boost::asio::buffer(command));
    }

  }

  void MiniBotHWInterface::enforceLimits(ros::Duration& period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

}  // namespace minibot_control
