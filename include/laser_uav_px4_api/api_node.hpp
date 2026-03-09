#ifndef LASER_UAV_API_PX4__API_NODE_HPP
#define LASER_UAV_API_PX4__API_NODE_HPP

#include <Eigen/Dense>
#include <cstdlib>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/motor_speed.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/esc_status.hpp>
#include <px4_msgs/msg/sensor_gyro.hpp>
#include <px4_msgs/msg/sensor_accel.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_px4_api
{
class ApiNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ApiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ApiNode() override;

private:
  Eigen::Vector3d    enuToNed(Eigen::Vector3d p);
  Eigen::Vector3d    frdToFlu(Eigen::Vector3d p);
  Eigen::Quaterniond enuToNedOrientation(Eigen::Quaterniond q);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr sub_rc_px4_;
  void                                                                  subRcPx4(const px4_msgs::msg::ManualControlSetpoint &msg);

  rclcpp::Publisher<laser_msgs::msg::PoseWithHeading>::SharedPtr pub_rc_to_goto_;

  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::ConstSharedPtr sub_control_mode_px4_;
  void                                                                    subControlModePx4(const px4_msgs::msg::VehicleControlMode &msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::ConstSharedPtr     sub_vehicle_odometry_px4_;
  void                                                                     subVehicleOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg);
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_nav_odometry_;

  rclcpp::Subscription<px4_msgs::msg::SensorGps>::ConstSharedPtr sub_vehicle_gps_position_px4_;
  void                                                           subVehicleGpsPositionPx4(const px4_msgs::msg::SensorGps &msg);

  rclcpp::Subscription<px4_msgs::msg::SensorGyro>::ConstSharedPtr        sub_sensor_gyro_px4_;
  void                                                                   subSensorGyroPx4(const px4_msgs::msg::SensorGyro &msg);
  rclcpp::Subscription<px4_msgs::msg::SensorAccel>::ConstSharedPtr       sub_sensor_accel_px4_;
  void                                                                   subSensorAccelPx4(const px4_msgs::msg::SensorAccel &msg);
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::ConstSharedPtr      sub_distance_sensor_px4_;
  void                                                                     subDistanceSensorPx4(const px4_msgs::msg::DistanceSensor &msg);
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr pub_garmin_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::ConstSharedPtr sub_vehicle_status_px4_;
  void                                                               subVehicleStatusPx4(const px4_msgs::msg::VehicleStatus &msg);

  rclcpp::Subscription<px4_msgs::msg::EscStatus>::ConstSharedPtr               sub_esc_status_px4_;
  void                                                                         subEscStatusPx4(const px4_msgs::msg::EscStatus &msg);
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr pub_motor_speed_estimation_;

  rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::ConstSharedPtr sub_control_manager_diagnostics_;
  void                                                                         subControlManagerDiagnostics(const laser_msgs::msg::UavControlDiagnostics &msg);

  rclcpp::Subscription<laser_msgs::msg::MotorSpeed>::ConstSharedPtr              sub_motor_speed_reference_;
  void                                                                           subMotorSpeedReference(const laser_msgs::msg::MotorSpeed &msg);
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::ActuatorMotors>::SharedPtr pub_motor_speed_reference_px4_;
  rclcpp::TimerBase::SharedPtr                                                   tmr_pub_motor_speed_reference_px4_;
  void                                                                           tmrPubMotorSpeedReferencePx4();

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_px4_;
  void pubVehicleCommandPx4(int command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0,
                            float param7 = 0.0);

  double                                                                              _rate_pub_offboard_control_mode_px4_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_control_mode_px4_;
  rclcpp::TimerBase::SharedPtr                                                        tmr_pub_offboard_control_mode_px4_;
  void                                                                                tmrPubOffboardControlModePx4();

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr pub_attitude_rates_reference_px4_;
  rclcpp::Subscription<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr             sub_attitude_rates_and_thrust_reference_;
  void subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void srvArm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void srvDisarm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  double                                                                              _rate_pub_api_diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::ApiPx4Diagnostics>::SharedPtr pub_api_diagnostics_;
  rclcpp::TimerBase::SharedPtr                                                        tmr_pub_api_diagnostics_;
  void                                                                                tmrPubApiDiagnostics();

  Eigen::Quaterniond               ned_enu_quaternion_rotation_;
  Eigen::Quaterniond               frd_flu_rotation_;
  Eigen::Affine3d                  frd_flu_affine_;
  Eigen::PermutationMatrix<3>      ned_enu_reflection_xy_;
  Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z_;

  sensor_msgs::msg::Imu                  imu_;
  laser_msgs::msg::ApiPx4Diagnostics     api_diagnostics_;
  laser_msgs::msg::UavControlDiagnostics control_manager_diagnostics_;
  px4_msgs::msg::ActuatorMotors          actuator_motors_reference_;

  int target_system_;

  std::string _control_input_mode_;

  int count_rc_aux_;
  int last_rc_aux_;
  double last_rc_timestamp_;

  bool real_uav_{false};
  bool offboard_is_enabled_{false};
  bool fw_preflight_checks_pass_{false};
  bool is_active_{false};
};
}  // namespace laser_uav_px4_api

#endif
