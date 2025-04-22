#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <numeric>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <builtin_interfaces/msg/time.hpp> 



TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (TakeHome::odometry_callback in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);

    // subscriber for wheel speed:
    wheel_speed_subscriber_ = this ->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report", qos_profile, std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    // subscriber for steering:
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "raptor_dbw_interface/steering_extended_report", qos_profile, std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    // to publish the slip angle:
    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    // Task B: IMU jitter
    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile, std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));

    imu_top_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);

    // Task C: Curvature distance
    last_curv_dist_=0.0f;
    last_lap_start_time_ = this->now();

    curv_dist_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile, std::bind(&TakeHome::curv_dist_callback, this, std::placeholders::_1));
      lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);

  }

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running ros2 bag info on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */

void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  latest_odometry_msg_ = odom_msg;

  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation

  metric_publisher_->publish(metric_msg);

  // call the function to compute the wheel slip:
  compute_and_publish_slip(); 
}

void TakeHome::wheel_speed_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
  latest_wheel_speed_msg_ = wheel_speed_msg;
  compute_and_publish_slip(); 
 
}

void TakeHome::steering_callback(const raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  latest_steering_msg_ = steering_msg;
  compute_and_publish_slip(); 
}

void TakeHome::compute_and_publish_slip() {
  if (!latest_odometry_msg_ || !latest_wheel_speed_msg_ || !latest_steering_msg_) {
    return;  // wait until all data has arrived at least once
  }

  // Extract odometry values linear velocities and yaw rate
  float v_x = latest_odometry_msg_->twist.twist.linear.x;
  float v_y = latest_odometry_msg_->twist.twist.linear.y;
  float omega = latest_odometry_msg_->twist.twist.angular.z;
  
  // Convert wheel speeds from km/h to m/s 
  float ws_fr = latest_wheel_speed_msg_->front_right / 3.6f;
  float ws_fl = latest_wheel_speed_msg_->front_left / 3.6f;
  float ws_rr = latest_wheel_speed_msg_->rear_right / 3.6f;
  float ws_rl = latest_wheel_speed_msg_->rear_left / 3.6f;
  
  // Rear Right slip:
  float v_x_rr = v_x - 0.5f * omega * w_r_;
  float slip_rr = (ws_rr - v_x_rr) / v_x_rr;
  
  // Rear Left slip:
  float v_x_rl = v_x + 0.5f * omega * w_r_;
  float slip_rl = (ws_rl - v_x_rl) / v_x_rl;
  
  // Front Wheels:
  // Convert steering angle: divide by steering ratio 15.0 and convert from degrees to radians.
  float steering_angle_deg = latest_steering_msg_->primary_steering_angle_fbk;
  float delta = (steering_angle_deg / 15.0f) * (static_cast<float>(M_PI) / 180.0f);
  
  // Front Right:
  float v_x_fr = v_x - 0.5f * omega * w_f_;
  float v_y_fr = v_y + omega * l_f_;
  float v_x_fr_delta = std::cos(delta) * v_x_fr - std::sin(delta) * v_y_fr;
  float slip_fr = (ws_fr - v_x_fr_delta) / v_x_fr_delta;
  
  // Front Left:
  float v_x_fl = v_x + 0.5f * omega * w_f_;
  float v_y_fl = v_y + omega * l_f_;
  float v_x_fl_delta = std::cos(delta) * v_x_fl - std::sin(delta) * v_y_fl;
  float slip_fl = (ws_fl - v_x_fl_delta) / v_x_fl_delta;
  
  // Publish each computed slip ratio using std_msgs::msg::Float32 messages:

  std_msgs::msg::Float32 rr_msg;
  rr_msg.data = slip_rr;
  slip_rr_publisher_->publish(rr_msg);

  std_msgs::msg::Float32 rl_msg;
  rl_msg.data = slip_rl;
  slip_rl_publisher_->publish(rl_msg);
  
  std_msgs::msg::Float32 fr_msg;
  fr_msg.data = slip_fr;
  slip_fr_publisher_->publish(fr_msg);
  
  std_msgs::msg::Float32 fl_msg;
  fl_msg.data = slip_fl;
  slip_fl_publisher_->publish(fl_msg);
}

// Task B: Sliding window for IMU jitter
void TakeHome::imu_top_callback(
  const novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg)
{
  // get time stamp
  rclcpp::Time now = imu_msg->header.stamp;

  //If we ever see the clock go backwards, clear our window
  if (!imu_dt_window_.empty()) {
    double rewind = (now - imu_dt_window_.back().first).seconds();
    if (rewind < -1.0) {
      imu_dt_window_.clear();
    }
  }

  // 3) Compute from the last message and append
  if (last_imu_time_.nanoseconds() != 0) {
    double dt = (now - last_imu_time_).seconds();
    imu_dt_window_.emplace_back(now, dt);
  }
  last_imu_time_ = now;

  //akeout any entries older than 1 second
  rclcpp::Time cutoff = now - rclcpp::Duration::from_seconds(1.0);
  while (!imu_dt_window_.empty() && imu_dt_window_.front().first < cutoff) {
    imu_dt_window_.pop_front();
  }

  // 5) Compute variance for window
  double var = 0.0;
  const size_t N = imu_dt_window_.size();
  if (N > 1) {
    double sum = 0.0, sum_sq = 0.0;
    for (auto &p : imu_dt_window_) {
      sum    += p.second;
      sum_sq += p.second * p.second;
    }
    double mean = sum / static_cast<double>(N);
    var = (sum_sq / static_cast<double>(N)) - (mean * mean);
  }

  // 6) Publish
  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = static_cast<float>(var);
  imu_top_jitter_publisher_->publish(jitter_msg);
}


// Task C: Lap time
void TakeHome::curv_dist_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg){
  const float MIN_FOR_WRAP = 0.1f;          
  const float MAX_BEFORE_WRAP = 1000.0f;    

  // In curv_dist_callback:
  float cur_dist = dist_msg->data;
  auto now = this->now();

  //track the maximum distance seen so far in this lap
  if (cur_dist > max_dist_seen_) {
    max_dist_seen_ = cur_dist;
    
    if (max_dist_seen_ > MAX_BEFORE_WRAP) {
      ready_for_wrap_ = true;
    }
  }

  // count a lap when  fall back below the start threshold
  if ( ready_for_wrap_ && cur_dist < MIN_FOR_WRAP ) {
    float lap_secs = (now - last_lap_start_time_).seconds();
    std_msgs::msg::Float32 lap_msg;
    lap_msg.data = lap_secs;
    lap_time_publisher_->publish(lap_msg);

    // reset for the next lap
    last_lap_start_time_ = now;
    max_dist_seen_ = 0.0f;
    ready_for_wrap_ = false;
  }

  last_curv_dist_= cur_dist;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
