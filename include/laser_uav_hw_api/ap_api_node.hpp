#ifndef LASER_UAV_HW_API__AP_API_NODE_HPP_
#define LASER_UAV_HW_API__AP_API_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <regex>
#include <string>

#include <Eigen/Dense>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>

#include <ardupilot_msgs/msg/attitude_target.hpp>
#include <ardupilot_msgs/msg/status.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_hw_api
{

/**
 * @class ApApiNode
 * @brief Hardware API Lifecycle Node for ArduPilot-based UAVs.
 * Handles coordinate conversions between ROS (ENU/FLU) and ArduPilot (NED/FRD),
 * manages flight mode switches, and publishes/subscribes to telemetry/control commands.
 */
class ApApiNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ApApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ApApiNode() override;

private:
  // --- Coordinate Frame Transformation Methods ---
  Eigen::Vector3d enu_to_ned(Eigen::Vector3d p);
  Eigen::Vector3d frd_to_flu(Eigen::Vector3d p);
  Eigen::Quaterniond enu_to_ned_orientation(Eigen::Quaterniond q);

  // --- Lifecycle Node Callbacks ---
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // --- Initialization Helpers ---
  void get_node_parameters();
  void config_pub_sub();
  void config_timers();
  void config_services();

  // --- ROS 2 Subscriptions & Callbacks ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr sub_estimated_pose_ap_;
  void on_estimated_pose_ap(const geometry_msgs::msg::PoseStamped & msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr sub_estimated_twist_ap_;
  void on_estimated_twist_ap(const geometry_msgs::msg::TwistStamped & msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_ap_;
  void on_imu_ap(const sensor_msgs::msg::Imu & msg);

  rclcpp::Subscription<ardupilot_msgs::msg::Status>::ConstSharedPtr sub_vehicle_status_ap_;
  void on_vehicle_status_ap(const ardupilot_msgs::msg::Status & msg);

  rclcpp::Subscription<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr
    sub_attitude_rates_and_thrust_reference_;
  void on_attitude_rates_and_thrust_reference(const laser_msgs::msg::AttitudeRatesAndThrust & msg);

  // --- Lifecycle Publishers ---
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_nav_odometry_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp_lifecycle::LifecyclePublisher<ardupilot_msgs::msg::AttitudeTarget>::SharedPtr
    pub_attitude_rates_reference_ap_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::ApiPx4Diagnostics>::SharedPtr
    pub_api_diagnostics_;

  // --- ROS 2 Services & Clients ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void on_srv_arm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void on_srv_disarm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr clt_arm_motors_ap_;
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr clt_mode_switch_ap_;

  // --- Timers ---
  rclcpp::TimerBase::SharedPtr tmr_pub_api_diagnostics_;
  void on_timer_pub_api_diagnostics();

  // --- Internal State & Transformation Matrices ---
  Eigen::Quaterniond ned_enu_quaternion_rotation_;
  Eigen::Quaterniond frd_flu_rotation_;
  Eigen::Affine3d frd_flu_affine_;
  Eigen::PermutationMatrix<3> ned_enu_reflection_xy_;
  Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z_;

  geometry_msgs::msg::TwistStamped twist_stamped_ap_;
  sensor_msgs::msg::Imu imu_;
  laser_msgs::msg::ApiPx4Diagnostics api_diagnostics_;

  // --- Node Parameters ---
  double rate_pub_api_diagnostics_{10.0};
  std::string uav_name_{""};
  bool publish_tf_odom_{false};
  bool inverted_tf_{true};
  bool real_uav_{false};
  bool offboard_is_enabled_{false};
  bool is_active_{false};
};

}  // namespace laser_uav_hw_api

#endif  // LASER_UAV_HW_API__AP_API_NODE_HPP_
