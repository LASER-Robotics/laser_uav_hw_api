#include "laser_uav_hw_api/px4_api_node.hpp"

#include <cmath>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace laser_uav_hw_api
{

/* Px4ApiNode() //{ */
Px4ApiNode::Px4ApiNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("px4_api_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating Px4ApiNode");

  // Declare parameters to replace OS environment variable calls
  declare_parameter("uav_name", rclcpp::ParameterValue("uav1"));
  declare_parameter("real_uav", rclcpp::ParameterValue(false));
  declare_parameter("control_input_mode", rclcpp::ParameterValue(""));
  declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
  declare_parameter("rate.pub_api_diagnostics", rclcpp::ParameterValue(10.0));
  declare_parameter("rc.aux_logics", rclcpp::ParameterValue(true));
  declare_parameter("tf.publish", rclcpp::ParameterValue(false));
  declare_parameter("tf.inverted", rclcpp::ParameterValue(true));

  // Load standard parameters
  uav_name_ = get_parameter("uav_name").as_string();
  real_uav_ = get_parameter("real_uav").as_bool();

  // Extract MAVLink target system ID by matching digits at the end of the UAV
  // name parameter. Example: If uav_name = "uav3", target_system_ becomes 3.
  target_system_ = 1;
  if (!real_uav_) {
    std::smatch match;
    std::regex re("(\\d+)$");
    if (std::regex_search(uav_name_, match, re)) {
      target_system_ = std::stoi(match[1]);
    }
  }

  RCLCPP_INFO(
    get_logger(), "Loaded Parameters -> UAV_NAME: %s, REAL_UAV: %s, Target System: %d",
    uav_name_.c_str(), real_uav_ ? "true" : "false", target_system_);

  // Configure geometric rotation matrices to bridge ROS (ENU/FLU) and PX4
  // (NED/FRD) frames
  ned_enu_quaternion_rotation_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  frd_flu_rotation_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_affine_ = Eigen::Affine3d(frd_flu_rotation_);

  ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_ = Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, -1.0);
}
/* //} */

/* ~Px4ApiNode() //{ */
Px4ApiNode::~Px4ApiNode() = default;
/* //} */

/* on_configure() //{ */
CallbackReturn Px4ApiNode::on_configure(const rclcpp_lifecycle::State &)
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
CallbackReturn Px4ApiNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  pub_vehicle_command_px4_->on_activate();
  pub_offboard_control_mode_px4_->on_activate();
  pub_api_diagnostics_->on_activate();
  pub_nav_odometry_->on_activate();
  pub_imu_->on_activate();
  pub_garmin_->on_activate();

  if (rc_aux_logics_) {
    pub_rc_to_goto_->on_activate();
  }

  if (control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_->on_activate();
  } else if (control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_->on_activate();
  }

  if (real_uav_) {
    pub_motor_speed_estimation_->on_activate();
  }

  is_active_ = true;
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_deactivate() //{ */
CallbackReturn Px4ApiNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  pub_vehicle_command_px4_->on_deactivate();
  pub_offboard_control_mode_px4_->on_deactivate();
  pub_nav_odometry_->on_deactivate();
  pub_imu_->on_deactivate();
  pub_garmin_->on_deactivate();
  pub_api_diagnostics_->on_deactivate();

  if (rc_aux_logics_) {
    pub_rc_to_goto_->on_deactivate();
  }

  if (control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_->on_deactivate();
  } else if (control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_->on_deactivate();
  }

  if (real_uav_) {
    pub_motor_speed_estimation_->on_deactivate();
  }

  is_active_ = false;
  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_cleanup() //{ */
CallbackReturn Px4ApiNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  sub_vehicle_odometry_px4_.reset();
  sub_vehicle_gps_position_px4_.reset();
  sub_sensor_gyro_px4_.reset();
  sub_sensor_accel_px4_.reset();
  sub_distance_sensor_px4_.reset();
  sub_vehicle_status_px4_.reset();
  sub_control_mode_px4_.reset();
  sub_control_manager_diagnostics_.reset();
  sub_esc_status_px4_.reset();

  pub_offboard_control_mode_px4_.reset();
  pub_nav_odometry_.reset();
  pub_imu_.reset();
  pub_garmin_.reset();
  pub_api_diagnostics_.reset();
  pub_motor_speed_estimation_.reset();

  tmr_pub_offboard_control_mode_px4_.reset();
  tmr_pub_motor_speed_reference_px4_.reset();
  tmr_pub_api_diagnostics_.reset();

  if (rc_aux_logics_) {
    sub_rc_px4_.reset();
    pub_rc_to_goto_.reset();
  }

  if (control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_.reset();
    sub_motor_speed_reference_.reset();
  } else if (control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_.reset();
    sub_attitude_rates_and_thrust_reference_.reset();
  }

  return CallbackReturn::SUCCESS;
}
/* //} */

/* on_shutdown() //{ */
CallbackReturn Px4ApiNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}
/* //} */

/* get_node_parameters() //{ */
void Px4ApiNode::get_node_parameters()
{
  get_parameter("control_input_mode", control_input_mode_);
  get_parameter("rate.pub_offboard_control_mode", rate_pub_offboard_control_mode_px4_);
  get_parameter("rate.pub_api_diagnostics", rate_pub_api_diagnostics_);
  get_parameter("rc.aux_logics", rc_aux_logics_);
  get_parameter("tf.publish", publish_tf_odom_);
  get_parameter("tf.inverted", inverted_tf_);
}
/* //} */

/* config_pub_sub() //{ */
void Px4ApiNode::config_pub_sub()
{
  RCLCPP_INFO(get_logger(), "Initializing Publishers and Subscribers");

  // PX4 uORB sensor inputs (SensorDataQoS required for micro-ROS bridge
  // compatibility)
  sub_vehicle_odometry_px4_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_vehicle_odometry_px4, this, std::placeholders::_1));

  sub_vehicle_gps_position_px4_ = create_subscription<px4_msgs::msg::SensorGps>(
    "vehicle_gps_position_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_vehicle_gps_position_px4, this, std::placeholders::_1));

  sub_sensor_gyro_px4_ = create_subscription<px4_msgs::msg::SensorGyro>(
    "sensor_gyro_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_sensor_gyro_px4, this, std::placeholders::_1));

  sub_sensor_accel_px4_ = create_subscription<px4_msgs::msg::SensorAccel>(
    "sensor_accel_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_sensor_accel_px4, this, std::placeholders::_1));

  sub_distance_sensor_px4_ = create_subscription<px4_msgs::msg::DistanceSensor>(
    "distance_sensor_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_distance_sensor_px4, this, std::placeholders::_1));

  if (rc_aux_logics_) {
    sub_rc_px4_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "px4_rc_in", rclcpp::SensorDataQoS(),
      std::bind(&Px4ApiNode::on_rc_px4, this, std::placeholders::_1));
    pub_rc_to_goto_ = create_publisher<laser_msgs::msg::PoseWithHeading>("rc_to_goto_out", 10);
  }

  sub_vehicle_status_px4_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "vehicle_status_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_vehicle_status_px4, this, std::placeholders::_1));

  sub_control_mode_px4_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
    "vehicle_control_mode_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&Px4ApiNode::on_control_mode_px4, this, std::placeholders::_1));

  if (real_uav_) {
    sub_esc_status_px4_ = create_subscription<px4_msgs::msg::EscStatus>(
      "esc_status_px4_in", rclcpp::SensorDataQoS(),
      std::bind(&Px4ApiNode::on_esc_status_px4, this, std::placeholders::_1));
    pub_motor_speed_estimation_ =
      create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_out", 10);
  }

  // PX4 uORB control command publishers
  pub_vehicle_command_px4_ =
    create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command_px4_out", 10);
  pub_offboard_control_mode_px4_ =
    create_publisher<px4_msgs::msg::OffboardControlMode>("offboard_control_mode_px4_out", 10);

  // ROS 2 System outputs
  pub_api_diagnostics_ =
    create_publisher<laser_msgs::msg::ApiPx4Diagnostics>("api_diagnostics", 10);
  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  pub_garmin_ = create_publisher<sensor_msgs::msg::Range>("garmin", 10);

  sub_control_manager_diagnostics_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>(
    "control_manager_diagnostics_in", 1,
    std::bind(&Px4ApiNode::on_control_manager_diagnostics, this, std::placeholders::_1));

  if (control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_ =
      create_publisher<px4_msgs::msg::ActuatorMotors>("motor_speed_reference_px4_out", 10);
    sub_motor_speed_reference_ = create_subscription<laser_msgs::msg::MotorSpeed>(
      "motor_speed_reference_in", 1,
      std::bind(&Px4ApiNode::on_motor_speed_reference, this, std::placeholders::_1));
  } else if (control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_ =
      create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("attitude_rates_reference_px4_out", 10);
    sub_attitude_rates_and_thrust_reference_ =
      create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
        "attitude_rates_thrust_in", 1,
        std::bind(
          &Px4ApiNode::on_attitude_rates_and_thrust_reference, this, std::placeholders::_1));
  }
}
/* //} */

/* config_timers() //{ */
void Px4ApiNode::config_timers()
{
  RCLCPP_INFO(get_logger(), "Initializing Timers");
  tmr_pub_offboard_control_mode_px4_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_pub_offboard_control_mode_px4_),
    std::bind(&Px4ApiNode::on_timer_pub_offboard_control_mode_px4, this));

  tmr_pub_api_diagnostics_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_pub_api_diagnostics_),
    std::bind(&Px4ApiNode::on_timer_pub_api_diagnostics, this));

  if (control_input_mode_ == "individual_thrust") {
    // 500 Hz control loop for direct ESC/actuator control
    tmr_pub_motor_speed_reference_px4_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 500.0),
      std::bind(&Px4ApiNode::on_timer_pub_motor_speed_reference_px4, this));
  }
}
/* //} */

/* config_services() //{ */
void Px4ApiNode::config_services()
{
  RCLCPP_INFO(get_logger(), "Initializing Services");
  srv_arm_ = create_service<std_srvs::srv::Trigger>(
    "arm", std::bind(&Px4ApiNode::on_srv_arm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>(
    "disarm",
    std::bind(&Px4ApiNode::on_srv_disarm, this, std::placeholders::_1, std::placeholders::_2));
  clt_land_ = create_client<std_srvs::srv::Trigger>("land");
}
/* //} */

/* on_control_mode_px4() //{ */
void Px4ApiNode::on_control_mode_px4(const px4_msgs::msg::VehicleControlMode & msg)
{
  if (!is_active_) {
    return;
  }
  api_diagnostics_.armed = msg.flag_armed;
  offboard_is_enabled_ = msg.flag_control_offboard_enabled;
  api_diagnostics_.offboard_mode = offboard_is_enabled_;
}
/* //} */

/* on_esc_status_px4() //{ */
void Px4ApiNode::on_esc_status_px4(const px4_msgs::msg::EscStatus & msg)
{
  if (!is_active_ || !real_uav_) {
    return;
  }

  // Convert RPM reported by ESC telemetry into angular speed (rad/s).
  // Conversion factor: 2 * PI / 60 ≈ 0.1047
  laser_msgs::msg::MotorSpeed motor_speed_estimation;
  for (int i = 0; i < static_cast<int>(msg.esc_count); i++) {
    motor_speed_estimation.data.push_back(msg.esc[i].esc_rpm * 0.1047f);
  }
  motor_speed_estimation.unit_of_measurement = "rad/s";
  pub_motor_speed_estimation_->publish(motor_speed_estimation);
}
/* //} */

/* on_rc_px4() //{ */
void Px4ApiNode::on_rc_px4(const px4_msgs::msg::ManualControlSetpoint & msg)
{
  if (!is_active_) {
    return;
  }

  // Auxiliary RC logic: Translates stick deflections into position setpoint
  // increments (GoTo commands)
  if (activate_goto_rc_) {
    auto rc_msg = laser_msgs::msg::PoseWithHeading();
    rc_msg.position.x =
      std::abs(msg.pitch) > 0.4f ? 0.1f * (msg.pitch / std::abs(msg.pitch)) : 0.0f;
    rc_msg.position.y = std::abs(msg.roll) > 0.4f ? 0.1f * (msg.roll / std::abs(msg.roll)) : 0.0f;
    rc_msg.position.z =
      std::abs(msg.throttle) > 0.4f ? 0.1f * (msg.throttle / std::abs(msg.throttle)) : 0.0f;
    rc_msg.heading = std::abs(msg.yaw) > 0.4f ? 0.1f * (msg.yaw / std::abs(msg.yaw)) : 0.0f;
    pub_rc_to_goto_->publish(rc_msg);
  }

  // Detect rising edge on RC AUX1 switch
  if (msg.aux1 != last_rc_aux_ && msg.aux1 == 1.0f) {
    count_rc_aux_++;
    last_rc_timestamp_ = msg.timestamp;
  }

  // Single toggle within 0.5s: Enable/Disable RC override control
  if (count_rc_aux_ == 1 && (msg.timestamp - last_rc_timestamp_) / 1000000.0 > 0.5) {
    activate_goto_rc_ = !activate_goto_rc_;
    RCLCPP_INFO(
      get_logger(), "%s RC to control the LUS!", activate_goto_rc_ ? "Activating" : "Deactivating");
    count_rc_aux_ = 0;
  }

  // Double toggle within 1.0s: Emergency/Manual landing request via ROS client
  if (count_rc_aux_ == 2 && (msg.timestamp - last_rc_timestamp_) / 1000000.0 > 1.0) {
    RCLCPP_INFO(get_logger(), "Calling landing via RC trigger");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    clt_land_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(
          this->get_logger(), "Land Response: [%s] %s", response->success ? "Success" : "Failed",
          response->message.c_str());
      });
    count_rc_aux_ = 0;
  }

  last_rc_aux_ = msg.aux1;
}
/* //} */

/* on_sensor_gyro_px4() //{ */
void Px4ApiNode::on_sensor_gyro_px4(const px4_msgs::msg::SensorGyro & msg)
{
  if (!is_active_) {
    return;
  }
  Eigen::Vector3d frd_to_flu_vec(msg.x, msg.y, msg.z);
  frd_to_flu_vec = frd_to_flu(frd_to_flu_vec);

  imu_.angular_velocity.x = frd_to_flu_vec(0);
  imu_.angular_velocity.y = frd_to_flu_vec(1);
  imu_.angular_velocity.z = frd_to_flu_vec(2);

  imu_.header.stamp = get_clock()->now();
  imu_.header.frame_id = uav_name_ + "/fcu";
  pub_imu_->publish(imu_);
}
/* //} */

/* on_sensor_accel_px4() //{ */
void Px4ApiNode::on_sensor_accel_px4(const px4_msgs::msg::SensorAccel & msg)
{
  if (!is_active_) {
    return;
  }
  Eigen::Vector3d frd_to_flu_vec(msg.x, msg.y, msg.z);
  frd_to_flu_vec = frd_to_flu(frd_to_flu_vec);

  imu_.linear_acceleration.x = frd_to_flu_vec(0);
  imu_.linear_acceleration.y = frd_to_flu_vec(1);
  imu_.linear_acceleration.z = frd_to_flu_vec(2);
}
/* //} */

/* on_distance_sensor_px4() //{ */
void Px4ApiNode::on_distance_sensor_px4(const px4_msgs::msg::DistanceSensor & msg)
{
  if (!is_active_) {
    return;
  }
  sensor_msgs::msg::Range garmin_msg;
  garmin_msg.header.stamp = get_clock()->now();
  garmin_msg.header.frame_id = uav_name_ + "/fcu";
  garmin_msg.range = msg.current_distance;
  garmin_msg.field_of_view = 0.0f;
  garmin_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  garmin_msg.min_range = msg.min_distance;
  garmin_msg.max_range = msg.max_distance;
  pub_garmin_->publish(garmin_msg);
}
/* //} */

/* on_vehicle_status_px4() //{ */
void Px4ApiNode::on_vehicle_status_px4(const px4_msgs::msg::VehicleStatus & msg)
{
  if (!is_active_) {
    return;
  }
  if (!fw_preflight_checks_pass_) {
    fw_preflight_checks_pass_ = msg.pre_flight_checks_pass;
  }
  api_diagnostics_.preflight_checks_passed = msg.pre_flight_checks_pass;
}
/* //} */

/* on_vehicle_gps_position_px4() //{ */
void Px4ApiNode::on_vehicle_gps_position_px4(const px4_msgs::msg::SensorGps & msg)
{
  if (!is_active_) {
    return;
  }
  api_diagnostics_.qty_satellites = msg.satellites_used;
  api_diagnostics_.rf_jamming = msg.jamming_indicator;
}
/* //} */

/* on_vehicle_odometry_px4() //{ */
void Px4ApiNode::on_vehicle_odometry_px4(const px4_msgs::msg::VehicleOdometry & msg)
{
  if (!is_active_ || !fw_preflight_checks_pass_) {
    return;
  }

  // Calculate alignment offsets on the first received odometry message
  if (!has_px4_odometry_offset_) {
    Eigen::Vector3d ned_to_enu_tf(msg.position[0], msg.position[1], msg.position[2]);
    position_offset_ = enu_to_ned(ned_to_enu_tf);

    Eigen::Quaterniond ned_to_enu_orientation_tf(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    quaternion_offset_ = enu_to_ned_orientation(ned_to_enu_orientation_tf).normalized();

    has_px4_odometry_offset_ = true;
    return;
  }

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.child_frame_id = uav_name_ + "/hw_api_odometry";
  current_nav_odometry.header.stamp = get_clock()->now();
  current_nav_odometry.header.frame_id = uav_name_ + "/fcu";

  // Apply position offset and rotate coordinate system from PX4 NED to ROS ENU
  Eigen::Vector3d ned_to_enu_tf(msg.position[0], msg.position[1], msg.position[2]);
  ned_to_enu_tf = enu_to_ned(ned_to_enu_tf) - position_offset_;
  ned_to_enu_tf = quaternion_offset_.inverse().normalized() * ned_to_enu_tf;

  current_nav_odometry.pose.pose.position.x = ned_to_enu_tf(0);
  current_nav_odometry.pose.pose.position.y = ned_to_enu_tf(1);
  current_nav_odometry.pose.pose.position.z = ned_to_enu_tf(2);

  Eigen::Quaterniond absolute_orientation_tf(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  absolute_orientation_tf = enu_to_ned_orientation(absolute_orientation_tf).normalized();
  Eigen::Quaterniond relative_orientation_tf =
    quaternion_offset_.inverse() * absolute_orientation_tf;
  relative_orientation_tf.coeffs() *= -1;

  current_nav_odometry.pose.pose.orientation.x = relative_orientation_tf.x();
  current_nav_odometry.pose.pose.orientation.y = relative_orientation_tf.y();
  current_nav_odometry.pose.pose.orientation.z = relative_orientation_tf.z();
  current_nav_odometry.pose.pose.orientation.w = relative_orientation_tf.w();

  current_nav_odometry.pose.covariance = {
    msg.position_variance[0],    0, 0, 0, 0, 0, 0, msg.position_variance[1],    0, 0, 0, 0, 0, 0,
    msg.position_variance[2],    0, 0, 0, 0, 0, 0, msg.orientation_variance[0], 0, 0, 0, 0, 0, 0,
    msg.orientation_variance[1], 0, 0, 0, 0, 0, 0, msg.orientation_variance[2]};

  // Convert velocities from NED world frame into body FLU frame
  Eigen::Vector3d vel_ned_tf(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
  Eigen::Vector3d vel_enu_tf = enu_to_ned(vel_ned_tf);
  Eigen::Vector3d vel_body_flu = absolute_orientation_tf.inverse() * vel_enu_tf;

  current_nav_odometry.twist.twist.linear.x = vel_body_flu(0);
  current_nav_odometry.twist.twist.linear.y = vel_body_flu(1);
  current_nav_odometry.twist.twist.linear.z = vel_body_flu(2);

  Eigen::Vector3d frd_to_flu_vec(
    msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
  frd_to_flu_vec = frd_to_flu(frd_to_flu_vec);

  current_nav_odometry.twist.twist.angular.x = frd_to_flu_vec(0);
  current_nav_odometry.twist.twist.angular.y = frd_to_flu_vec(1);
  current_nav_odometry.twist.twist.angular.z = frd_to_flu_vec(2);

  const double default_angular_var = 0.01;
  current_nav_odometry.twist.covariance = {
    msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
    msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, default_angular_var,      0, 0, 0, 0, 0, 0,
    default_angular_var,      0, 0, 0, 0, 0, 0, default_angular_var};

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

/* on_timer_pub_offboard_control_mode_px4() //{ */
void Px4ApiNode::on_timer_pub_offboard_control_mode_px4()
{
  if (!is_active_) {
    return;
  }

  // PX4 Offboard Control Heartbeat: Must be streamed continuously (>2Hz) to
  // maintain offboard mode
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.thrust_and_torque = false;

  if (control_input_mode_ == "individual_thrust") {
    msg.body_rate = false;
    msg.direct_actuator = true;
  } else if (control_input_mode_ == "angular_rates_and_thrust") {
    msg.body_rate = true;
    msg.direct_actuator = false;
  }

  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  pub_offboard_control_mode_px4_->publish(msg);
}
/* //} */

/* pub_vehicle_command_px4() //{ */
void Px4ApiNode::pub_vehicle_command_px4(
  int command, float param1, float param2, float param3, float param4, float param5, float param6,
  float param7)
{
  if (!is_active_) {
    return;
  }
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = target_system_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_px4_->publish(msg);
}
/* //} */

/* on_timer_pub_api_diagnostics() //{ */
void Px4ApiNode::on_timer_pub_api_diagnostics()
{
  if (!is_active_) {
    return;
  }
  pub_api_diagnostics_->publish(api_diagnostics_);
}
/* //} */

/* on_attitude_rates_and_thrust_reference() //{ */
void Px4ApiNode::on_attitude_rates_and_thrust_reference(
  const laser_msgs::msg::AttitudeRatesAndThrust & msg)
{
  if (!is_active_ || !offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Preflight checks failed in Firmware!");
    return;
  }

  px4_msgs::msg::VehicleRatesSetpoint attitude_rates_reference{};

  // Convert angular rates from ROS FLU to PX4 FRD
  Eigen::Vector3d flu_to_frd_vec(msg.roll_rate, msg.pitch_rate, msg.yaw_rate);
  flu_to_frd_vec = frd_to_flu(flu_to_frd_vec);

  attitude_rates_reference.roll = flu_to_frd_vec(0);
  attitude_rates_reference.pitch = flu_to_frd_vec(1);
  attitude_rates_reference.yaw = flu_to_frd_vec(2);

  // PX4 body thrust convention: Negative Z points upwards towards the sky in
  // FRD frame
  attitude_rates_reference.thrust_body[0] = 0.0f;
  attitude_rates_reference.thrust_body[1] = 0.0f;
  attitude_rates_reference.thrust_body[2] = -msg.total_thrust_normalized;

  attitude_rates_reference.timestamp = get_clock()->now().nanoseconds() / 1000;
  pub_attitude_rates_reference_px4_->publish(attitude_rates_reference);
}
/* //} */

/* on_motor_speed_reference() //{ */
void Px4ApiNode::on_motor_speed_reference(const laser_msgs::msg::MotorSpeed & msg)
{
  if (!is_active_ || !offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Preflight checks failed in Firmware!");
    return;
  }

  for (size_t i = 0; i < msg.data.size() && i < actuator_motors_reference_.control.size(); ++i) {
    actuator_motors_reference_.control[i] = msg.data[i];
  }

  actuator_motors_reference_.timestamp = get_clock()->now().nanoseconds() / 1000;
  actuator_motors_reference_.timestamp_sample = 0;
}
/* //} */

/* on_control_manager_diagnostics() //{ */
void Px4ApiNode::on_control_manager_diagnostics(const laser_msgs::msg::UavControlDiagnostics & msg)
{
  if (!is_active_) {
    return;
  }
  control_manager_diagnostics_ = msg;
}
/* //} */

/* on_timer_pub_motor_speed_reference_px4() //{ */
void Px4ApiNode::on_timer_pub_motor_speed_reference_px4()
{
  if (
    !is_active_ || !offboard_is_enabled_ || !fw_preflight_checks_pass_ ||
    control_input_mode_ != "individual_thrust") {
    return;
  }
  pub_motor_speed_reference_px4_->publish(actuator_motors_reference_);
}
/* //} */

/* on_srv_arm() //{ */
void Px4ApiNode::on_srv_arm(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    response->success = false;
    response->message = "Arm request failed: Preflight checks did not pass in PX4 firmware.";
    return;
  }

  // Set offboard mode first if operating in simulation
  if (!real_uav_) {
    pub_vehicle_command_px4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  // Only allow arming if the drone is currently disarmed and not flying
  if (!control_manager_diagnostics_.is_fly && !api_diagnostics_.armed) {
    pub_vehicle_command_px4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    response->success = true;
    response->message = "Arm command sent successfully.";
  } else {
    response->success = false;
    response->message = "Arm request rejected: UAV is already armed or in flight.";
  }
}
/* //} */

/* on_srv_disarm() //{ */
void Px4ApiNode::on_srv_disarm(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  // Safety check: Only allow disarming if the drone is on the ground and has no
  // active trajectory goal
  if (
    !control_manager_diagnostics_.is_fly && !control_manager_diagnostics_.have_goal &&
    api_diagnostics_.armed) {
    pub_vehicle_command_px4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    response->success = true;
    response->message = "Disarm command sent successfully.";
  } else {
    response->success = false;
    response->message = "Disarm request rejected: UAV is still flying or already disarmed.";
  }
}
/* //} */

/* enu_to_ned() //{ */
Eigen::Vector3d Px4ApiNode::enu_to_ned(Eigen::Vector3d p)
{
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
/* //} */

/* frd_to_flu() //{ */
Eigen::Vector3d Px4ApiNode::frd_to_flu(Eigen::Vector3d p)
{
  return frd_flu_affine_ * p;
}
/* //} */

/* enu_to_ned_orientation() //{ */
Eigen::Quaterniond Px4ApiNode::enu_to_ned_orientation(Eigen::Quaterniond q)
{
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
/* //} */

}  // namespace laser_uav_hw_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_hw_api::Px4ApiNode)
