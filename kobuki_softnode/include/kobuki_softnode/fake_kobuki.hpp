/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */ 

/* Modified for ROS 2 */

#ifndef _FAKE_KOBUKI_H_
#define _FAKE_KOBUKI_H_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kobuki_ros_interfaces/msg/version_info.hpp>

// place holder for future support
//#include <sensor_msgs/Imu.h>
//#include <kobuki_ros_interfaces/ButtonEvent.h>
//#include <kobuki_ros_interfaces/BumperEvent.h>
//#include <kobuki_ros_interfaces/CliffEvent.h>
//#include <kobuki_ros_interfaces/DigitalOutput.h>
//#include <kobuki_ros_interfaces/DigitalInputEvent.h>
//#include <kobuki_ros_interfaces/ExternalPower.h>
//#include <kobuki_ros_interfaces/DockInfraRed.h>
//#include <kobuki_ros_interfaces/Led.h>
//#include <kobuki_ros_interfaces/MotorPower.h>
//#include <kobuki_ros_interfaces/PowerSystemEvent.h>
//#include <kobuki_ros_interfaces/RobotStateEvent.h>
//#include <kobuki_ros_interfaces/SensorState.h>
//#include <kobuki_ros_interfaces/Sound.h>
//#include <kobuki_ros_interfaces/WheelDropEvent.h>

namespace kobuki
{
  enum {
    LEFT=0,
    RIGHT=1
  };

  class FakeKobuki
  {
    public:
      FakeKobuki(rclcpp::Node* p_node);

      // variables
      kobuki_ros_interfaces::msg::VersionInfo    versioninfo;

      sensor_msgs::msg::JointState               joint_states;
      nav_msgs::msg::Odometry                    odom;
      float odom_pose[3];
      float odom_vel[3];
      double pose_cov[36];

      std::string wheel_joint_name[2];
      
      float wheel_speed_cmd[2];
      float wheel_separation;
      float wheel_diameter;

      bool motor_enabled;
      double cmd_vel_timeout;

      // events
//      kobuki_msgs::BumperEvent        bumper_event;
//      kobuki_msgs::DigitalInputEvent  digital_input;
//      kobuki_msgs::WheelDropEvent     wheel_drop;
//      kobuki_msgs::PowerSystemEvent   power_system;
//      kobuki_msgs::CliffEvent         cliff;
//      kobuki_msgs::ButtonEvent        button;

      // sensors
//      kobuki_msgs::SensorState        core;
//      kobuki_msgs::DockInfraRed       dock_ir;
//      sensor_msgs::Imu                imu_data;

  };
}
#endif