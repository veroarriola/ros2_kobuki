#include <kobuki_softnode/fake_kobuki_ros.hpp>
#include <tf2/transform_datatypes.h>

using std::placeholders::_1;

namespace kobuki
{
  auto createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  FakeKobukiRos::FakeKobukiRos(const std::string &node_name)
  : Node(node_name), name(node_name), tf2_broadcaster(this), kobuki(this) //, count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Creating fake. ['%s']", this->name.c_str());

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&FakeKobukiRos::update, this));

    // initialize publishers
    advertiseTopics();

    // initialize subscribers
    subscribeTopics();

    publishVersionInfoOnce(); 

    this->prev_update_time = this->now();   // TODO
  }

  FakeKobukiRos::~FakeKobukiRos()
  {
  }

  void FakeKobukiRos::advertiseTopics() 
  {
    // turtlebot required
    this->publish_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
    //nh.advertise<sensor_msgs::JointState>("joint_states",100);

    // kobuki esoterics
    this->publish_version_info_ = this->create_publisher<kobuki_ros_interfaces::msg::VersionInfo>("version_info", 100);
    //this->publisher["version_info"] = nh.advertise<kobuki_msgs::VersionInfo>("version_info",100,true); //no latch in ROS2

    // odometry
    this->publish_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    //this->publisher["odom"] = nh.advertise<nav_msgs::Odometry>("odom",100);


    /*
    // event publishers
    std::string evt = "events/";
    event_publisher["button"]         = nh.advertise<kobuki_msgs::SensorState>        (evt + "button",        100);
    event_publisher["bumper"]         = nh.advertise<kobuki_msgs::BumperEvent>        (evt + "bumper",        100);
    event_publisher["cliff"]          = nh.advertise<kobuki_msgs::CliffEvent>         (evt + "cliff",         100);
    event_publisher["wheel_drop"]     = nh.advertise<kobuki_msgs::WheelDropEvent>     (evt + "wheel_drop",    100);
    event_publisher["power_system"]   = nh.advertise<kobuki_msgs::PowerSystemEvent>   (evt + "power_system",  100);
    event_publisher["digital_input"]  = nh.advertise<kobuki_msgs::DigitalInputEvent>  (evt + "digital_input", 100);
    event_publisher["robot_state"]    = nh.advertise<kobuki_msgs::RobotStateEvent>    (evt + "robot_state",   100,true); // latched
  // sensor publishers
    std::string sen = "sensors/";
    sensor_publisher["core"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "core", 100); 
    sensor_publisher["dock_ir"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "dock_ir", 100); 
    sensor_publisher["imu_data"]  = nh.advertise<kobuki_msgs::SensorState> (sen + "imu_data", 100); 
    */
  }

  void FakeKobukiRos::subscribeTopics()
  {
    std::string cmd = "commands/";
    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "velocity", 10, std::bind(&FakeKobukiRos::subscribeVelocityCommand, this, _1));
    subscription_motor_power_ = this->create_subscription<kobuki_ros_interfaces::msg::MotorPower>(
      "motor_power", 10, std::bind(&FakeKobukiRos::subscribeMotorPowerCommand, this, _1));
    //this->subscriber["velocity"] = nh.subscribe(cmd + "velocity", 10, &FakeKobukiRos::subscribeVelocityCommand, this);
    //this->subscriber["motor_power"] = nh.subscribe(cmd + "motor_power", 10, &FakeKobukiRos::subscribeMotorPowerCommand,this);
  }

  void FakeKobukiRos::publishVersionInfoOnce()
  {
    this->publish_version_info_->publish(this->kobuki.versioninfo);
    //this->publisher["version_info"].publish(this->kobuki.versioninfo);
  }

  void FakeKobukiRos::subscribeVelocityCommand(const geometry_msgs::msg::Twist & msg)
  {
    this->last_cmd_vel_time = this->now();
    this->kobuki.wheel_speed_cmd[LEFT]  = msg.linear.x - msg.angular.z * this->kobuki.wheel_separation / 2;
    this->kobuki.wheel_speed_cmd[RIGHT] = msg.linear.x + msg.angular.z * this->kobuki.wheel_separation / 2;
  }

  void FakeKobukiRos::subscribeMotorPowerCommand(const kobuki_ros_interfaces::msg::MotorPower & msg)
  {
    if((msg.state == kobuki_ros_interfaces::msg::MotorPower::ON) && (!this->kobuki.motor_enabled))
    {
      this->kobuki.motor_enabled = true;
      //ROS_INFO_STREAM("Motors fire up. [" << this->name << "]");
      RCLCPP_INFO(this->get_logger(), "Motors fire up. ['%s']", this->name.c_str());
    }
    else if((msg.state == kobuki_ros_interfaces::msg::MotorPower::OFF) && (this->kobuki.motor_enabled))
    {
      this->kobuki.motor_enabled = false;
      //ROS_INFO_STREAM("Motors take a break. [" << this->name << "]");
      RCLCPP_INFO(this->get_logger(), "Motors take a break. ['%s']", this->name.c_str());
    }
  }

  void FakeKobukiRos::updateJoint(unsigned int index,double& w, rclcpp::Duration step_time)
  {
    double v; 
    v = this->kobuki.wheel_speed_cmd[index]; 
    w = v / (this->kobuki.wheel_diameter / 2);
    this->kobuki.joint_states.velocity[index] = w;
    this->kobuki.joint_states.position[index]= this->kobuki.joint_states.position[index] + w * step_time.seconds();
  }

  void FakeKobukiRos::updateOdometry(double w_left,double w_right,rclcpp::Duration step_time)
  {
    double d1,d2;
    double dr,da;
    d1 = d2 = 0;
    dr = da = 0;

    d1 = step_time.seconds() * (this->kobuki.wheel_diameter / 2) * w_left; 
    d2 = step_time.seconds() * (this->kobuki.wheel_diameter / 2) * w_right; 

    if(std::isnan(d1))
    {
      d1 = 0;
    }
    if(std::isnan(d2))
    {
      d2 = 0;
    }

    dr = (d1 + d2) / 2;
    da = (d2 - d1) / this->kobuki.wheel_separation;

    // compute odometric pose
    this->kobuki.odom_pose[0] += dr * cos(this->kobuki.odom_pose[2]);
    this->kobuki.odom_pose[1] += dr * sin(this->kobuki.odom_pose[2]);
    this->kobuki.odom_pose[2] += da;

    // compute odometric instantaneouse velocity
    this->kobuki.odom_vel[0] = dr / step_time.seconds();
    this->kobuki.odom_vel[1] = 0.0;
    this->kobuki.odom_vel[2] = da / step_time.seconds();

    this->kobuki.odom.pose.pose.position.x = this->kobuki.odom_pose[0];
    this->kobuki.odom.pose.pose.position.y = this->kobuki.odom_pose[1];
    this->kobuki.odom.pose.pose.position.z = 0;
    this->kobuki.odom.pose.pose.orientation = createQuaternionMsgFromYaw(this->kobuki.odom_pose[2]);

    // We should update the twist of the odometry
    this->kobuki.odom.twist.twist.linear.x = this->kobuki.odom_vel[0];
    this->kobuki.odom.twist.twist.angular.z = this->kobuki.odom_vel[2];
  }

  void FakeKobukiRos::updateTF(geometry_msgs::msg::TransformStamped& odom_tf)
  {
    odom_tf.header = this->kobuki.odom.header;
    odom_tf.child_frame_id = this->kobuki.odom.child_frame_id;
    odom_tf.transform.translation.x = this->kobuki.odom.pose.pose.position.x;
    odom_tf.transform.translation.y = this->kobuki.odom.pose.pose.position.y;
    odom_tf.transform.translation.z = this->kobuki.odom.pose.pose.position.z;
    odom_tf.transform.rotation = this->kobuki.odom.pose.pose.orientation;
  }


  void FakeKobukiRos::update()
  {
    //RCLCPP_INFO(this->get_logger(), "Update. ['%s']", this->name.c_str());
    
    rclcpp::Time time_now = this->now();
    rclcpp::Duration step_time = time_now - this->prev_update_time;
    this->prev_update_time = time_now;
    
    // zero-ing after timeout
    
    //rclcpp::Duration since_command = time_now - this->last_cmd_vel_time;
    //if((since_command.seconds() > this->kobuki.cmd_vel_timeout) || !this->kobuki.motor_enabled)
    if(!this->kobuki.motor_enabled)
    {
      this->kobuki.wheel_speed_cmd[LEFT] = 0.0;
      this->kobuki.wheel_speed_cmd[RIGHT] = 0.0;
    }
    
    
    // joint_states
    double w_left,w_right;
    updateJoint(LEFT,w_left,step_time);
    updateJoint(RIGHT,w_right,step_time);
    this->kobuki.joint_states.header.stamp = time_now;
    this->publish_joint_states_->publish(this->kobuki.joint_states);

    // odom
    updateOdometry(w_left,w_right,step_time);
    this->kobuki.odom.header.stamp = time_now;
    this->publish_odom_->publish(this->kobuki.odom);
    
    // tf
    geometry_msgs::msg::TransformStamped odom_tf;
    updateTF(odom_tf);
    this->tf2_broadcaster.sendTransform(odom_tf);
    
  }
} 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kobuki::FakeKobukiRos>("mobile_base"));
  rclcpp::shutdown();
  return 0;
}