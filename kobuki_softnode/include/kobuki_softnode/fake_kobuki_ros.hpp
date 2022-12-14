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

#ifndef _FAKE_KOBUKI_NODE_H_
#define _FAKE_KOBUKI_NODE_H_

#include <chrono>
#include <memory>
//#include <time.hpp>
//#include <duration.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <sensor_msgs/JointState.h>

// place holder for future support
#include <sensor_msgs/msg/imu.h>
#include <kobuki_ros_interfaces/msg/button_event.hpp>
#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/digital_output.hpp>
#include <kobuki_ros_interfaces/msg/digital_input_event.hpp>
#include <kobuki_ros_interfaces/msg/external_power.hpp>
#include <kobuki_ros_interfaces/msg/dock_infra_red.hpp>
#include <kobuki_ros_interfaces/msg/led.hpp>
#include <kobuki_ros_interfaces/msg/motor_power.hpp>
#include <kobuki_ros_interfaces/msg/power_system_event.hpp>
#include <kobuki_ros_interfaces/msg/robot_state_event.hpp>
#include <kobuki_ros_interfaces/msg/sensor_state.hpp>
#include <kobuki_ros_interfaces/msg/sound.hpp>
#include <kobuki_ros_interfaces/msg/version_info.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

#include "fake_kobuki.hpp"

namespace kobuki
{
  class FakeKobukiRos : public rclcpp::Node
  {
    public:
      FakeKobukiRos(const std::string &node_name);
      ~FakeKobukiRos();

      void update();

    private:
      //void timer_callback();
      void advertiseTopics();
      void subscribeTopics();
      void publishVersionInfoOnce();

      // subscriber callbacks
      void subscribeVelocityCommand(const geometry_msgs::msg::Twist & msg);
      void subscribeMotorPowerCommand(const kobuki_ros_interfaces::msg::MotorPower & msg);

      void updateJoint(unsigned int index,double& w, rclcpp::Duration step_time);
      void updateOdometry(double w_left,double w_right, rclcpp::Duration step_time);
      void updateTF(geometry_msgs::msg::TransformStamped& odom_tf);

      ///////////////////////////
      // Variables 
      ///////////////////////////
      const std::string name;
      rclcpp::Time last_cmd_vel_time;
      rclcpp::Time prev_update_time;

      rclcpp::TimerBase::SharedPtr timer_;
      size_t count_;

      // version_info, joint_states
      //std::map<std::string,rclcpp::Publisher> publisher;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publish_joint_states_;
      rclcpp::Publisher<kobuki_ros_interfaces::msg::VersionInfo>::SharedPtr publish_version_info_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publish_odom_;
      // button, bumper, cliff, wheel_drop, power_system, digital_input, robot_state
      //std::map<std::string,ros::Publisher> event_publisher;
      // sensor_core, dock_ir, imu_data
      //std::map<std::string,ros::Publisher> sensor_publisher;
      // no debug publisher
      //tf::TransformBroadcaster        tf_broadcaster;
      tf2_ros::TransformBroadcaster        tf2_broadcaster;

      // command subscribers
      //std::map<std::string,ros::Subscriber> subscriber;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_velocity_;
      rclcpp::Subscription<kobuki_ros_interfaces::msg::MotorPower>::SharedPtr subscription_motor_power_;

      FakeKobuki kobuki;

  };
}
#endif