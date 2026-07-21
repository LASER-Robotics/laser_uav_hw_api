#ifndef LASER_UAV_HW_API__PX4_API_NODE_HPP_
#define LASER_UAV_HW_API__PX4_API_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <regex>
#include <string>

#include <Eigen/Dense>

#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/motor_speed.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/esc_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_accel.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/sensor_gyro.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_hw_api
{

/**
 * @class Px4ApiNode
 * @brief Hardware API Lifecycle Node for PX4-based UAVs.
 * Bridges PX4 uORB topics (exposed via micro-ROS/XRCE-DDS) with standard ROS 2
 * messages. Handles coordinate transformations, offboard control heartbeats,
 * and RC override logic.
 */
class Px4ApiNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Px4ApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Px4ApiNode() override;

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
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr sub_rc_px4_;
  void on_rc_px4(const px4_msgs::msg::ManualControlSetpoint & msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::ConstSharedPtr sub_control_mode_px4_;
  void on_control_mode_px4(const px4_msgs::msg::VehicleControlMode & msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::ConstSharedPtr sub_vehicle_odometry_px4_;
  void on_vehicle_odometry_px4(const px4_msgs::msg::VehicleOdometry & msg);

  rclcpp::Subscription<px4_msgs::msg::SensorGps>::ConstSharedPtr sub_vehicle_gps_position_px4_;
  void on_vehicle_gps_position_px4(const px4_msgs::msg::SensorGps & msg);

  rclcpp::Subscription<px4_msgs::msg::SensorGyro>::ConstSharedPtr sub_sensor_gyro_px4_;
  void on_sensor_gyro_px4(const px4_msgs::msg::SensorGyro & msg);

  rclcpp::Subscription<px4_msgs::msg::SensorAccel>::ConstSharedPtr sub_sensor_accel_px4_;
  void on_sensor_accel_px4(const px4_msgs::msg::SensorAccel & msg);

  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::ConstSharedPtr sub_distance_sensor_px4_;
  void on_distance_sensor_px4(const px4_msgs::msg::DistanceSensor & msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::ConstSharedPtr sub_vehicle_status_px4_;
  void on_vehicle_status_px4(const px4_msgs::msg::VehicleStatus & msg);

  rclcpp::Subscription<px4_msgs::msg::EscStatus>::ConstSharedPtr sub_esc_status_px4_;
  void on_esc_status_px4(const px4_msgs::msg::EscStatus & msg);

  rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::ConstSharedPtr
    sub_control_manager_diagnostics_;
  void on_control_manager_diagnostics(const laser_msgs::msg::UavControlDiagnostics & msg);

  rclcpp::Subscription<laser_msgs::msg::MotorSpeed>::ConstSharedPtr sub_motor_speed_reference_;
  void on_motor_speed_reference(const laser_msgs::msg::MotorSpeed & msg);

  rclcpp::Subscription<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr
    sub_attitude_rates_and_thrust_reference_;
  void on_attitude_rates_and_thrust_reference(const laser_msgs::msg::AttitudeRatesAndThrust & msg);

  // --- Lifecycle Publishers ---
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::PoseWithHeading>::SharedPtr pub_rc_to_goto_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_nav_odometry_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr pub_garmin_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr
    pub_motor_speed_estimation_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::ActuatorMotors>::SharedPtr
    pub_motor_speed_reference_px4_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    pub_vehicle_command_px4_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    pub_offboard_control_mode_px4_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
    pub_attitude_rates_reference_px4_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::ApiPx4Diagnostics>::SharedPtr
    pub_api_diagnostics_;

  void pub_vehicle_command_px4(
    int command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0,
    float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

  // --- ROS 2 Services & Clients ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void on_srv_arm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void on_srv_disarm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_land_;

  // --- Timers ---
  rclcpp::TimerBase::SharedPtr tmr_pub_offboard_control_mode_px4_;
  void on_timer_pub_offboard_control_mode_px4();

  rclcpp::TimerBase::SharedPtr tmr_pub_motor_speed_reference_px4_;
  void on_timer_pub_motor_speed_reference_px4();

  rclcpp::TimerBase::SharedPtr tmr_pub_api_diagnostics_;
  void on_timer_pub_api_diagnostics();

  // --- Internal State & Transformation Matrices ---
  Eigen::Quaterniond ned_enu_quaternion_rotation_;
  Eigen::Quaterniond frd_flu_rotation_;
  Eigen::Affine3d frd_flu_affine_;
  Eigen::PermutationMatrix<3> ned_enu_reflection_xy_;
  Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z_;

  sensor_msgs::msg::Imu imu_;
  laser_msgs::msg::ApiPx4Diagnostics api_diagnostics_;
  laser_msgs::msg::UavControlDiagnostics control_manager_diagnostics_;
  px4_msgs::msg::ActuatorMotors actuator_motors_reference_;

  Eigen::Vector3d position_offset_;
  Eigen::Quaterniond quaternion_offset_;

  // --- Node Parameters ---
  std::string control_input_mode_{""};
  std::string uav_name_{""};
  double rate_pub_offboard_control_mode_px4_{100.0};
  double rate_pub_api_diagnostics_{10.0};
  int target_system_{1};
  int count_rc_aux_{0};
  float last_rc_aux_{-1.0f};
  double last_rc_timestamp_{0.0};
  bool publish_tf_odom_{false};
  bool inverted_tf_{true};
  bool rc_aux_logics_{true};
  bool has_px4_odometry_offset_{false};
  bool real_uav_{false};
  bool offboard_is_enabled_{false};
  bool fw_preflight_checks_pass_{false};
  bool activate_goto_rc_{false};
  bool is_active_{false};
};

}  // namespace laser_uav_hw_api

#endif  // LASER_UAV_HW_API__PX4_API_NODE_HPP_
