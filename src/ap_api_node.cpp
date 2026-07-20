#include "laser_uav_hw_api/ap_api_node.hpp"

#include <cmath>

#include <algorithm>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace laser_uav_hw_api
{

/* ApApiNode() //{ */
ApApiNode::ApApiNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ap_api_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating ApApiNode");

  // Declare parameters to replace OS environment variable calls
  declare_parameter("uav_name", rclcpp::ParameterValue("uav1"));
  declare_parameter("real_uav", rclcpp::ParameterValue(false));
  declare_parameter("rate.pub_api_diagnostics", rclcpp::ParameterValue(10.0));
  declare_parameter("tf.publish", rclcpp::ParameterValue(false));
  declare_parameter("tf.inverted", rclcpp::ParameterValue(true));

  // Load basic parameters
  uav_name_ = get_parameter("uav_name").as_string();
  real_uav_ = get_parameter("real_uav").as_bool();

  RCLCPP_INFO(
    get_logger(), "Loaded Parameters -> UAV_NAME: %s, REAL_UAV: %s", uav_name_.c_str(),
    real_uav_ ? "true" : "false");

  // Configure geometric rotation matrices to bridge ROS (ENU/FLU) and ArduPilot (NED/FRD) frames.
  // ENU: East-North-Up (ROS World), NED: North-East-Down (Autopilot World)
  ned_enu_quaternion_rotation_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  // FRD: Forward-Right-Down (Autopilot Body), FLU: Forward-Left-Up (ROS Body)
  frd_flu_rotation_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_affine_ = Eigen::Affine3d(frd_flu_rotation_);

  ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_ = Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, -1.0);
}
/* //} */

/* ~ApApiNode() //{ */
ApApiNode::~ApApiNode() = default;
/* //} */

/* on_configure() //{ */
CallbackReturn ApApiNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  get_node_parameters();
  config_pub_sub();
  config_timers();
  config_services();
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_activate() //{ */
CallbackReturn ApApiNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  pub_api_diagnostics_->on_activate();
  pub_nav_odometry_->on_activate();
  pub_imu_->on_activate();
  pub_attitude_rates_reference_ap_->on_activate();
  is_active_ = true;
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_deactivate() //{ */
CallbackReturn ApApiNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  pub_nav_odometry_->on_deactivate();
  pub_imu_->on_deactivate();
  pub_api_diagnostics_->on_deactivate();
  pub_attitude_rates_reference_ap_->on_deactivate();
  is_active_ = false;
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_cleanup() //{ */
CallbackReturn ApApiNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  sub_estimated_pose_ap_.reset();
  sub_estimated_twist_ap_.reset();
  sub_imu_ap_.reset();
  sub_vehicle_status_ap_.reset();
  pub_imu_.reset();
  pub_api_diagnostics_.reset();
  tmr_pub_api_diagnostics_.reset();
  pub_attitude_rates_reference_ap_.reset();
  sub_attitude_rates_and_thrust_reference_.reset();
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_shutdown() //{ */
CallbackReturn ApApiNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}
/* //} */

/* get_node_parameters() //{ */
void ApApiNode::get_node_parameters()
{
  get_parameter("rate.pub_api_diagnostics", rate_pub_api_diagnostics_);
  get_parameter("tf.publish", publish_tf_odom_);
  get_parameter("tf.inverted", inverted_tf_);
}
/* //} */

/* config_pub_sub() //{ */
void ApApiNode::config_pub_sub()
{
  RCLCPP_INFO(get_logger(), "Initializing Publishers and Subscribers");

  // ArduPilot telemetry inputs (using SensorDataQoS for best-effort UDP/serial streaming)
  sub_estimated_pose_ap_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "estimated_pose_ap_in", rclcpp::SensorDataQoS(),
    std::bind(&ApApiNode::on_estimated_pose_ap, this, std::placeholders::_1));

  sub_estimated_twist_ap_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "estimated_twist_ap_in", rclcpp::SensorDataQoS(),
    std::bind(&ApApiNode::on_estimated_twist_ap, this, std::placeholders::_1));

  sub_imu_ap_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu_ap_in", rclcpp::SensorDataQoS(),
    std::bind(&ApApiNode::on_imu_ap, this, std::placeholders::_1));

  sub_vehicle_status_ap_ = create_subscription<ardupilot_msgs::msg::Status>(
    "vehicle_status_ap_in", rclcpp::SensorDataQoS(),
    std::bind(&ApApiNode::on_vehicle_status_ap, this, std::placeholders::_1));

  // ROS 2 System outputs
  pub_api_diagnostics_ =
    create_publisher<laser_msgs::msg::ApiPx4Diagnostics>("api_diagnostics", 10);
  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  // Reference command inputs/outputs
  pub_attitude_rates_reference_ap_ =
    create_publisher<ardupilot_msgs::msg::AttitudeTarget>("attitude_rates_reference_ap_out", 10);
  sub_attitude_rates_and_thrust_reference_ =
    create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
      "attitude_rates_thrust_in", 1,
      std::bind(&ApApiNode::on_attitude_rates_and_thrust_reference, this, std::placeholders::_1));
}
/* //} */

/* config_timers() //{ */
void ApApiNode::config_timers()
{
  RCLCPP_INFO(get_logger(), "Initializing Timers");
  tmr_pub_api_diagnostics_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_pub_api_diagnostics_),
    std::bind(&ApApiNode::on_timer_pub_api_diagnostics, this));
}
/* //} */

/* config_services() //{ */
void ApApiNode::config_services()
{
  RCLCPP_INFO(get_logger(), "Initializing Services");
  srv_arm_ = create_service<std_srvs::srv::Trigger>(
    "arm", std::bind(&ApApiNode::on_srv_arm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>(
    "disarm",
    std::bind(&ApApiNode::on_srv_disarm, this, std::placeholders::_1, std::placeholders::_2));

  clt_arm_motors_ap_ = create_client<ardupilot_msgs::srv::ArmMotors>("arm_motors_ap");
  clt_mode_switch_ap_ = create_client<ardupilot_msgs::srv::ModeSwitch>("mode_switch_ap");
}
/* //} */

/* on_imu_ap() //{ */
void ApApiNode::on_imu_ap(const sensor_msgs::msg::Imu & msg)
{
  if (!is_active_) {
    return;
  }

  // Convert angular velocity and linear acceleration from FRD (Autopilot Body) to FLU (ROS Body)
  Eigen::Vector3d frd_to_flu_vec;
  frd_to_flu_vec << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
  frd_to_flu_vec = frd_to_flu(frd_to_flu_vec);

  imu_.angular_velocity.x = frd_to_flu_vec(0);
  imu_.angular_velocity.y = frd_to_flu_vec(1);
  imu_.angular_velocity.z = frd_to_flu_vec(2);

  frd_to_flu_vec << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
  frd_to_flu_vec = frd_to_flu(frd_to_flu_vec);

  imu_.linear_acceleration.x = frd_to_flu_vec(0);
  imu_.linear_acceleration.y = frd_to_flu_vec(1);
  imu_.linear_acceleration.z = frd_to_flu_vec(2);

  imu_.header.stamp = get_clock()->now();
  imu_.header.frame_id = uav_name_ + "/fcu";
  pub_imu_->publish(imu_);
}
/* //} */

/* on_vehicle_status_ap() //{ */
void ApApiNode::on_vehicle_status_ap(const ardupilot_msgs::msg::Status & msg)
{
  if (!is_active_) {
    return;
  }

  // Check arming state and identify if ArduPilot is in GUIDED mode (Mode 4 represents
  // GUIDED/Offboard)
  api_diagnostics_.armed = msg.armed;
  offboard_is_enabled_ = (msg.mode == 4);
  api_diagnostics_.offboard_mode = offboard_is_enabled_;
}
/* //} */

/* on_estimated_pose_ap() //{ */
void ApApiNode::on_estimated_pose_ap(const geometry_msgs::msg::PoseStamped & msg)
{
  if (!is_active_) {
    return;
  }

  // Construct standard ROS 2 Odometry message combining estimated pose and twist
  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.child_frame_id = uav_name_ + "/hw_api_odometry";
  current_nav_odometry.header.stamp = get_clock()->now();
  current_nav_odometry.header.frame_id = uav_name_ + "/fcu";
  current_nav_odometry.pose.pose = msg.pose;

  // Rotate world velocity into the UAV body frame using the inverse orientation quaternion
  Eigen::Quaterniond q(
    msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  Eigen::Vector3d twist_body_frame(
    twist_stamped_ap_.twist.linear.x, twist_stamped_ap_.twist.linear.y,
    twist_stamped_ap_.twist.linear.z);

  q.coeffs() *= -1.0;
  q.normalize();
  twist_body_frame = q.conjugate().toRotationMatrix() * twist_body_frame;

  current_nav_odometry.twist.twist.linear.x = twist_body_frame(0);
  current_nav_odometry.twist.twist.linear.y = twist_body_frame(1);
  current_nav_odometry.twist.twist.linear.z = twist_body_frame(2);

  // Convert angular rates to Body Frame (FLU)
  Eigen::Vector3d flu_to_frd_vec(
    twist_stamped_ap_.twist.angular.x, twist_stamped_ap_.twist.angular.y,
    twist_stamped_ap_.twist.angular.z);

  current_nav_odometry.twist.twist.angular.x = flu_to_frd_vec(0);
  current_nav_odometry.twist.twist.angular.y = flu_to_frd_vec(1);
  current_nav_odometry.twist.twist.angular.z = flu_to_frd_vec(2);

  pub_nav_odometry_->publish(current_nav_odometry);

  // Publish Odometry TF without changing the frame order (header.frame_id ->
  // child_frame_id)
  if (publish_tf_odom_) {
    static auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_nav_odometry.header.stamp;
    transform_stamped.header.frame_id = current_nav_odometry.header.frame_id;
    transform_stamped.child_frame_id = current_nav_odometry.child_frame_id;
    transform_stamped.transform.translation.x = inverted_tf_
                                                  ? -current_nav_odometry.pose.pose.position.x
                                                  : current_nav_odometry.pose.pose.position.x;
    transform_stamped.transform.translation.y = inverted_tf_
                                                  ? -current_nav_odometry.pose.pose.position.y
                                                  : current_nav_odometry.pose.pose.position.y;
    transform_stamped.transform.translation.z = inverted_tf_
                                                  ? -current_nav_odometry.pose.pose.position.z
                                                  : current_nav_odometry.pose.pose.position.z;
    transform_stamped.transform.rotation = current_nav_odometry.pose.pose.orientation;
    tf_broadcaster->sendTransform(transform_stamped);
  }
}
/* //} */

/* on_estimated_twist_ap() //{ */
void ApApiNode::on_estimated_twist_ap(const geometry_msgs::msg::TwistStamped & msg)
{
  if (!is_active_) {
    return;
  }
  twist_stamped_ap_ = msg;
}
/* //} */

/* on_timer_pub_api_diagnostics() //{ */
void ApApiNode::on_timer_pub_api_diagnostics()
{
  if (!is_active_) {
    return;
  }
  pub_api_diagnostics_->publish(api_diagnostics_);
}
/* //} */

/* on_attitude_rates_and_thrust_reference() //{ */
void ApApiNode::on_attitude_rates_and_thrust_reference(
  const laser_msgs::msg::AttitudeRatesAndThrust & msg)
{
  if (!is_active_ || !offboard_is_enabled_ || std::isnan(msg.total_thrust_normalized)) {
    return;
  }

  // Type mask 128 ignores attitude orientation and commands body rates + thrust directly
  ardupilot_msgs::msg::AttitudeTarget attitude_rates_reference{};
  attitude_rates_reference.type_mask = 128;

  // Convert requested angular rates from ROS FLU frame to ArduPilot FRD frame
  Eigen::Vector3d flu_to_frd_vec(msg.roll_rate, msg.pitch_rate, msg.yaw_rate);
  flu_to_frd_vec = frd_to_flu(flu_to_frd_vec);

  attitude_rates_reference.body_rate.x = flu_to_frd_vec(0);
  attitude_rates_reference.body_rate.y = flu_to_frd_vec(1);
  attitude_rates_reference.body_rate.z = flu_to_frd_vec(2);

  attitude_rates_reference.use_raw_ang_reference = true;
  attitude_rates_reference.thrust = msg.total_thrust_normalized;
  attitude_rates_reference.header.stamp = get_clock()->now();

  pub_attitude_rates_reference_ap_->publish(attitude_rates_reference);
}
/* //} */

/* on_srv_arm() //{ */
void ApApiNode::on_srv_arm(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  // Safety interlock: Prevent programmatic arming via ROS service if running on physical hardware
  if (real_uav_) {
    response->success = false;
    response->message = "Arm request failed: For real hardware, use the RC transmitter to arm.";
    return;
  }

  // Step 1: Switch flight mode to GUIDED (Mode 4 in ArduPilot)
  auto mode_request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
  mode_request->mode = 4;
  auto response_mode_switch = clt_mode_switch_ap_->async_send_request(mode_request);
  response_mode_switch.share().wait_for(std::chrono::milliseconds(500));

  // Step 2: Send arm command
  auto arm_request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  arm_request->arm = true;
  auto response_arm_motors = clt_arm_motors_ap_->async_send_request(arm_request);
  response_arm_motors.share().wait_for(std::chrono::milliseconds(500));

  response->success = true;
  response->message = "Arming sequence executed successfully.";
}
/* //} */

/* on_srv_disarm() //{ */
void ApApiNode::on_srv_disarm(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  // Safety interlock: Prevent programmatic disarming via ROS service on physical hardware
  if (real_uav_) {
    response->success = false;
    response->message =
      "Disarm request failed: For real hardware, use the RC transmitter to disarm.";
    return;
  }

  auto arm_request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  arm_request->arm = false;
  auto response_arm_motors = clt_arm_motors_ap_->async_send_request(arm_request);
  response_arm_motors.share().wait_for(std::chrono::milliseconds(500));

  response->success = true;
  response->message = "Disarm sequence executed successfully.";
}
/* //} */

/* enu_to_ned() //{ */
Eigen::Vector3d ApApiNode::enu_to_ned(Eigen::Vector3d p)
{
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
/* //} */

/* frd_to_flu() //{ */
Eigen::Vector3d ApApiNode::frd_to_flu(Eigen::Vector3d p)
{
  return frd_flu_affine_ * p;
}
/* //} */

/* enu_to_ned_orientation() //{ */
Eigen::Quaterniond ApApiNode::enu_to_ned_orientation(Eigen::Quaterniond q)
{
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
/* //} */

}  // namespace laser_uav_hw_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_hw_api::ApApiNode)
