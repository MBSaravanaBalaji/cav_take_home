#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <novatel_oem7_msgs/msg/rawimu.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>


#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

//for sliding window:
#include <deque>
#include <utility>  



class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);
  // Task A
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_callback(const raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void compute_and_publish_slip();

  // Task B
  //void imu_top_callback(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);


  // Task C
  void curv_dist_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg);


 private:

  // Subscribers and Publishers- task A
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;


  // my added subscribers: - task A
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;

  // my added publishers: - task A
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;

  // Subscribers and Publishers- task B
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_top_jitter_publisher_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;

  // Subscribers and Publishers- task C
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curv_dist_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  float last_curv_dist_{0.0f};
  rclcpp::Time last_lap_start_time_;

  float max_dist_seen_{0.0f};
  bool ready_for_wrap_{false};

  // stored messages
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_msg_;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr latest_wheel_speed_msg_;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr latest_steering_msg_;

  const float w_f_ = 1.638f;
  const float w_r_ = 1.523f;
  const float l_f_ = 1.7238f;

  // Sliding window for IMU jitter
  std::deque<std::pair<rclcpp::Time, double>> imu_dt_window_;
  rclcpp::Time last_imu_time_;
  
};
