#include "laser_uav_px4_api/api_node.hpp"

namespace laser_uav_px4_api
{
/* ApiNode() //{ */
ApiNode::ApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("control_input_mode", rclcpp::ParameterValue(""));
  declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
  declare_parameter("rate.pub_api_diagnostics", rclcpp::ParameterValue(10.0));
  declare_parameter("rc.aux_logics", rclcpp::ParameterValue(true));

  ned_enu_quaternion_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_rotation_            = Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_affine_              = Eigen::Affine3d(frd_flu_rotation_);

  ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_  = Eigen::DiagonalMatrix<double, 3>(1, 1, -1);

  const char *real_uav = std::getenv("REAL_UAV");
  target_system_       = 1;
  if (real_uav != nullptr) {
    std::string real_uav_str = std::string(real_uav);
    if (real_uav_str == "false") {
      real_uav_            = false;
      const char *uav_name = std::getenv("UAV_NAME");
      if (uav_name != nullptr) {
        std::smatch match;
        std::regex  re("(\\d+)$");

        std::string uav_name_str = std::string(uav_name);
        if (std::regex_search(uav_name_str, match, re)) {
          std::string numero_str = match[1];
          target_system_         = std::stoi(std::string(match[1]));
        }
      }
    } else {
      real_uav_ = true;
    }
  }

  count_rc_aux_ = 0;
  last_rc_aux_  = -1.0;
}
//}

/* ~ApiNode() //{ */
ApiNode::~ApiNode() {
}
//}

/* on_configure() //{ */
CallbackReturn ApiNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_vehicle_command_px4_->on_activate();
  pub_offboard_control_mode_px4_->on_activate();
  pub_api_diagnostics_->on_activate();
  pub_nav_odometry_->on_activate();
  pub_imu_->on_activate();
  pub_garmin_->on_activate();

  if (_rc_aux_logics_) {
    pub_rc_to_goto_->on_activate();
  }

  if (_control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_->on_activate();
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_->on_activate();
  }

  if (real_uav_) {
    pub_motor_speed_estimation_->on_activate();
  }

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_vehicle_command_px4_->on_deactivate();
  pub_offboard_control_mode_px4_->on_deactivate();
  pub_nav_odometry_->on_deactivate();
  pub_imu_->on_deactivate();
  pub_garmin_->on_deactivate();
  pub_api_diagnostics_->on_deactivate();

  if (_rc_aux_logics_) {
    pub_rc_to_goto_->on_deactivate();
  }

  if (_control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_->on_deactivate();
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_->on_deactivate();
  }

  if (real_uav_) {
    pub_motor_speed_estimation_->on_deactivate();
  }

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
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

  if (_rc_aux_logics_) {
    sub_rc_px4_.reset();
    pub_rc_to_goto_.reset();
  }

  if (_control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_.reset();
    sub_motor_speed_reference_.reset();
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_.reset();
    sub_attitude_rates_and_thrust_reference_.reset();
  }

  if (real_uav_) {
    sub_esc_status_px4_.reset();
    pub_motor_speed_estimation_.reset();
  }

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn ApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void ApiNode::getParameters() {
  get_parameter("control_input_mode", _control_input_mode_);
  get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_px4_);
  get_parameter("rate.pub_api_diagnostics", _rate_pub_api_diagnostics_);
  get_parameter("rc.aux_logics", _rc_aux_logics_);
}
//}

/* configPubSub() //{ */
void ApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for Px4 topics
  sub_vehicle_odometry_px4_ = create_subscription<px4_msgs::msg::VehicleOdometry>("vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
                                                                                  std::bind(&ApiNode::subVehicleOdometryPx4, this, std::placeholders::_1));

  sub_vehicle_gps_position_px4_ = create_subscription<px4_msgs::msg::SensorGps>("vehicle_gps_position_px4_in", rclcpp::SensorDataQoS(),
                                                                                std::bind(&ApiNode::subVehicleGpsPositionPx4, this, std::placeholders::_1));

  sub_sensor_gyro_px4_     = create_subscription<px4_msgs::msg::SensorGyro>("sensor_gyro_px4_in", rclcpp::SensorDataQoS(),
                                                                        std::bind(&ApiNode::subSensorGyroPx4, this, std::placeholders::_1));
  sub_sensor_accel_px4_    = create_subscription<px4_msgs::msg::SensorAccel>("sensor_accel_px4_in", rclcpp::SensorDataQoS(),
                                                                          std::bind(&ApiNode::subSensorAccelPx4, this, std::placeholders::_1));
  sub_distance_sensor_px4_ = create_subscription<px4_msgs::msg::DistanceSensor>("distance_sensor_px4_in", rclcpp::SensorDataQoS(),
                                                                                std::bind(&ApiNode::subDistanceSensorPx4, this, std::placeholders::_1));

  if (_rc_aux_logics_) {
    sub_rc_px4_     = create_subscription<px4_msgs::msg::ManualControlSetpoint>("px4_rc_in", rclcpp::SensorDataQoS(),
                                                                            std::bind(&ApiNode::subRcPx4, this, std::placeholders::_1));
    pub_rc_to_goto_ = create_publisher<laser_msgs::msg::PoseWithHeading>("rc_to_goto_out", 10);
  }

  sub_vehicle_status_px4_ = create_subscription<px4_msgs::msg::VehicleStatus>("vehicle_status_px4_in", rclcpp::SensorDataQoS(),
                                                                              std::bind(&ApiNode::subVehicleStatusPx4, this, std::placeholders::_1));
  sub_control_mode_px4_   = create_subscription<px4_msgs::msg::VehicleControlMode>("vehicle_control_mode_px4_in", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&ApiNode::subControlModePx4, this, std::placeholders::_1));
  if (real_uav_) {
    sub_esc_status_px4_         = create_subscription<px4_msgs::msg::EscStatus>("esc_status_px4_in", rclcpp::SensorDataQoS(),
                                                                        std::bind(&ApiNode::subEscStatusPx4, this, std::placeholders::_1));
    pub_motor_speed_estimation_ = create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_out", 10);
  }

  pub_vehicle_command_px4_       = create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command_px4_out", 10);
  pub_offboard_control_mode_px4_ = create_publisher<px4_msgs::msg::OffboardControlMode>("offboard_control_mode_px4_out", 10);


  // Pubs and Subs for System topics
  pub_api_diagnostics_ = create_publisher<laser_msgs::msg::ApiPx4Diagnostics>("api_diagnostics", 10);

  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);


  pub_garmin_ = create_publisher<sensor_msgs::msg::Range>("garmin", 10);

  sub_control_manager_diagnostics_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>(
      "control_manager_diagnostics_in", 1, std::bind(&ApiNode::subControlManagerDiagnostics, this, std::placeholders::_1));
  pub_motor_speed_estimation_ = create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_out", 10);

  if (_control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_ = create_publisher<px4_msgs::msg::ActuatorMotors>("motor_speed_reference_px4_out", 10);
    sub_motor_speed_reference_     = create_subscription<laser_msgs::msg::MotorSpeed>("motor_speed_reference_in", 1,
                                                                                  std::bind(&ApiNode::subMotorSpeedReference, this, std::placeholders::_1));
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_        = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("attitude_rates_reference_px4_out", 10);
    sub_attitude_rates_and_thrust_reference_ = create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
        "attitude_rates_thrust_in", 1, std::bind(&ApiNode::subAttitudeRatesAndThrustReference, this, std::placeholders::_1));
  }
}
//}

/* configTimers() //{ */
void ApiNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_pub_offboard_control_mode_px4_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_px4_),
                                                         std::bind(&ApiNode::tmrPubOffboardControlModePx4, this), nullptr);
  tmr_pub_api_diagnostics_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_api_diagnostics_), std::bind(&ApiNode::tmrPubApiDiagnostics, this), nullptr);
  if (_control_input_mode_ == "individual_thrust") {
    tmr_pub_motor_speed_reference_px4_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / 500), std::bind(&ApiNode::tmrPubMotorSpeedReferencePx4, this), nullptr);
  }
}
//}

/* configServices() //{ */
void ApiNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_arm_    = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));
  clt_land_   = create_client<std_srvs::srv::Trigger>("land");
}
//}

/* subControlModePx4() //{ */
void ApiNode::subControlModePx4(const px4_msgs::msg::VehicleControlMode &msg) {
  if (!is_active_) {
    return;
  }

  api_diagnostics_.armed         = msg.flag_armed;
  offboard_is_enabled_           = msg.flag_control_offboard_enabled;
  api_diagnostics_.offboard_mode = offboard_is_enabled_;
}
//}

/* subEscStatusPx4() //{ */
void ApiNode::subEscStatusPx4(const px4_msgs::msg::EscStatus &msg) {
  if (!is_active_) {
    return;
  }

  if (!real_uav_) {
    return;
  }

  laser_msgs::msg::MotorSpeed motor_speed_estimation;

  for (auto i = 0; i < (int)msg.esc_count; i++) {
    motor_speed_estimation.data.push_back(msg.esc[i].esc_rpm * 0.1047);
  }
  motor_speed_estimation.unit_of_measurement = "rad/s";

  pub_motor_speed_estimation_->publish(motor_speed_estimation);
}
//}

/* subRcPx4() //{ */
void ApiNode::subRcPx4(const px4_msgs::msg::ManualControlSetpoint &msg) {
  if (!is_active_) {
    return;
  }

  if (activate_goto_rc_) {
    auto rc_msg       = laser_msgs::msg::PoseWithHeading();
    rc_msg.position.x = std::abs(msg.pitch) > 0.4 ? 0.1 * (msg.pitch / std::abs(msg.pitch)) : 0.0;
    rc_msg.position.y = std::abs(msg.roll) > 0.4 ? 0.1 * (msg.roll / std::abs(msg.roll)) : 0.0;
    rc_msg.position.z = std::abs(msg.throttle) > 0.4 ? 0.1 * (msg.throttle / std::abs(msg.throttle)) : 0.0;
    rc_msg.heading    = std::abs(msg.yaw) > 0.4 ? 0.1 * (msg.yaw / std::abs(msg.yaw)) : 0.0;

    pub_rc_to_goto_->publish(rc_msg);
  }

  if (msg.aux1 != last_rc_aux_ && msg.aux1 == 1.0) {
    count_rc_aux_++;
    last_rc_timestamp_ = msg.timestamp;
  }

  if (count_rc_aux_ == 1 && (msg.timestamp - last_rc_timestamp_) / 1000000 > 0.5) {
    activate_goto_rc_ = !activate_goto_rc_;
    RCLCPP_INFO(get_logger(), "%s RC to control the LUS!", activate_goto_rc_ ? "Activating" : "Deactivating");
    count_rc_aux_ = 0;
  }

  if (count_rc_aux_ == 2 && (msg.timestamp - last_rc_timestamp_) / 1000000 > 1.0) {
    RCLCPP_INFO(get_logger(), "Calling landing via RC");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    clt_land_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Response: [%s] %s", response->success ? "Success" : "Failed", response->message.c_str());
    });
    count_rc_aux_ = 0;
  }

  last_rc_aux_ = msg.aux1;
}
//}

/* subSensorGyroPx4() //{ */
void ApiNode::subSensorGyroPx4(const px4_msgs::msg::SensorGyro &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.x, msg.y, msg.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.angular_velocity.x = frd_to_flu(0);
  imu_.angular_velocity.y = frd_to_flu(1);
  imu_.angular_velocity.z = frd_to_flu(2);

  imu_.header.stamp    = get_clock()->now();
  imu_.header.frame_id = "fcu";
  pub_imu_->publish(imu_);
}
//}

/* subSensorAccelPx4() //{ */
void ApiNode::subSensorAccelPx4(const px4_msgs::msg::SensorAccel &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.x, msg.y, msg.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.linear_acceleration.x = frd_to_flu(0);
  imu_.linear_acceleration.y = frd_to_flu(1);
  imu_.linear_acceleration.z = frd_to_flu(2);
}
//}

/* subDistanceSensorPx4() //{ */
void ApiNode::subDistanceSensorPx4(const px4_msgs::msg::DistanceSensor &msg) {
  if (!is_active_) {
    return;
  }

  sensor_msgs::msg::Range garmin_msg;
  garmin_msg.header.stamp    = get_clock()->now();
  garmin_msg.header.frame_id = "fcu";
  garmin_msg.range           = msg.current_distance;
  garmin_msg.field_of_view   = 0;
  garmin_msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
  garmin_msg.min_range       = msg.min_distance;
  garmin_msg.max_range       = msg.max_distance;
  pub_garmin_->publish(garmin_msg);
}
//}

/* subVehicleStatusPx4() //{ */
void ApiNode::subVehicleStatusPx4(const px4_msgs::msg::VehicleStatus &msg) {
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    fw_preflight_checks_pass_ = msg.pre_flight_checks_pass;
  }
  api_diagnostics_.preflight_checks_passed = msg.pre_flight_checks_pass;
}
//}

/* subVehicleGpsPositionPx4() //{ */
void ApiNode::subVehicleGpsPositionPx4(const px4_msgs::msg::SensorGps &msg) {
  if (!is_active_) {
    return;
  }

  api_diagnostics_.qty_satellites = msg.satellites_used;
  api_diagnostics_.rf_jamming     = msg.jamming_indicator;
}
//}

/* subVehicleOdometryPx4() //{ */
void ApiNode::subVehicleOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg) {
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    return;
  }

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.header.frame_id = "odom";
  current_nav_odometry.header.stamp    = get_clock()->now();
  current_nav_odometry.child_frame_id  = "fcu";

  Eigen::Vector3d ned_to_enu_tf(msg.position[0], msg.position[1], msg.position[2]);
  ned_to_enu_tf = enuToNed(ned_to_enu_tf);

  current_nav_odometry.pose.pose.position.x = ned_to_enu_tf(0);
  current_nav_odometry.pose.pose.position.y = ned_to_enu_tf(1);
  current_nav_odometry.pose.pose.position.z = ned_to_enu_tf(2);

  Eigen::Quaterniond ned_to_enu_orientation_tf(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  ned_to_enu_orientation_tf = enuToNedOrientation(ned_to_enu_orientation_tf);
  ned_to_enu_orientation_tf = ned_to_enu_orientation_tf.normalized();
  ned_to_enu_orientation_tf.coeffs() *= -1;

  // --- Multiply by -1 for adjust rotation
  current_nav_odometry.pose.pose.orientation.x = ned_to_enu_orientation_tf.x();
  current_nav_odometry.pose.pose.orientation.y = ned_to_enu_orientation_tf.y();
  current_nav_odometry.pose.pose.orientation.z = ned_to_enu_orientation_tf.z();
  current_nav_odometry.pose.pose.orientation.w = ned_to_enu_orientation_tf.w();

  current_nav_odometry.pose.covariance = {msg.position_variance[0],    0, 0, 0, 0, 0, 0, msg.position_variance[1],    0, 0, 0, 0, 0, 0,
                                          msg.position_variance[2],    0, 0, 0, 0, 0, 0, msg.orientation_variance[0], 0, 0, 0, 0, 0, 0,
                                          msg.orientation_variance[1], 0, 0, 0, 0, 0, 0, msg.orientation_variance[2]};

  ned_to_enu_tf(0) = msg.velocity[0];
  ned_to_enu_tf(1) = msg.velocity[1];
  ned_to_enu_tf(2) = msg.velocity[2];
  ned_to_enu_tf    = enuToNed(ned_to_enu_tf);
  ned_to_enu_tf    = ned_to_enu_orientation_tf.conjugate().normalized().toRotationMatrix() * ned_to_enu_tf;

  current_nav_odometry.twist.twist.linear.x = ned_to_enu_tf(0);
  current_nav_odometry.twist.twist.linear.y = ned_to_enu_tf(1);
  current_nav_odometry.twist.twist.linear.z = ned_to_enu_tf(2);

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2];
  frd_to_flu = frdToFlu(frd_to_flu);

  current_nav_odometry.twist.twist.angular.x = frd_to_flu(0);
  current_nav_odometry.twist.twist.angular.y = frd_to_flu(1);
  current_nav_odometry.twist.twist.angular.z = frd_to_flu(2);

  current_nav_odometry.twist.covariance = {msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, msg.velocity_variance[0], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[1], 0, 0, 0, 0, 0, 0, msg.velocity_variance[2]};

  pub_nav_odometry_->publish(current_nav_odometry);
}
//}

/* tmrPubOffboardControlModePx4() //{ */
void ApiNode::tmrPubOffboardControlModePx4() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = false;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.thrust_and_torque = false;
  if (_control_input_mode_ == "individual_thrust") {
    msg.body_rate       = false;
    msg.direct_actuator = true;
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    msg.body_rate       = true;
    msg.direct_actuator = false;
  }
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;


  pub_offboard_control_mode_px4_->publish(msg);
}
//}

/* pubVehicleCommandPx4() //{ */
void ApiNode::pubVehicleCommandPx4(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::VehicleCommand msg{};
  msg.param1           = param1;
  msg.param2           = param2;
  msg.param3           = param3;
  msg.param4           = param4;
  msg.param5           = param5;
  msg.param6           = param6;
  msg.param7           = param7;
  msg.command          = command;
  msg.target_system    = target_system_;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_px4_->publish(msg);
}
//}

/* tmrPubApiDiagnostic() //{ */
void ApiNode::tmrPubApiDiagnostics() {
  if (!is_active_) {
    return;
  }

  pub_api_diagnostics_->publish(api_diagnostics_);
}
//}

/* subAttitudeRatesAndThrustReference() //{ */
void ApiNode::subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg) {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!");
    return;
  }

  px4_msgs::msg::VehicleRatesSetpoint attitude_rates_reference{};

  Eigen::Vector3d flu_to_frd;
  flu_to_frd << msg.roll_rate, msg.pitch_rate, msg.yaw_rate;
  flu_to_frd = frdToFlu(flu_to_frd);

  attitude_rates_reference.roll  = flu_to_frd(0);
  attitude_rates_reference.pitch = flu_to_frd(1);
  attitude_rates_reference.yaw   = flu_to_frd(2);

  attitude_rates_reference.thrust_body[0] = 0;
  attitude_rates_reference.thrust_body[1] = 0;
  attitude_rates_reference.thrust_body[2] = -msg.total_thrust_normalized;

  attitude_rates_reference.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  pub_attitude_rates_reference_px4_->publish(attitude_rates_reference);
}
//}

/* subMotorSpeedReference() //{ */
void ApiNode::subMotorSpeedReference(const laser_msgs::msg::MotorSpeed &msg) {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!");
    return;
  }

  for (auto i = 0; i < (int)msg.data.size(); i++) {
    actuator_motors_reference_.control[i] = msg.data[i];
  }

  actuator_motors_reference_.timestamp        = get_clock()->now().nanoseconds() / 1000;
  actuator_motors_reference_.timestamp_sample = 0;
}
//}

/* subControlManagerDiagnostics() //{ */
void ApiNode::subControlManagerDiagnostics(const laser_msgs::msg::UavControlDiagnostics &msg) {
  if (!is_active_) {
    return;
  }

  control_manager_diagnostics_ = msg;
}
//}

/* tmrPubMotorSpeedReferencePx4() //{ */
void ApiNode::tmrPubMotorSpeedReferencePx4() {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    return;
  }

  if (_control_input_mode_ != "individual_thrust") {
    return;
  }

  pub_motor_speed_reference_px4_->publish(actuator_motors_reference_);
}
//}

/* srvArm() //{ */
void ApiNode::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    response->success = false;
    response->message = "arm requested failed, preflight checks don't pass in firmware, try again in a few seconds";
    return;
  }

  if (!real_uav_) {
    pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  }

  if (!control_manager_diagnostics_.is_fly && !api_diagnostics_.armed) {
    pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  } else {
    response->success = false;
    response->message = "arm request failed: UAV is already armed or in flight";
    return;
  }

  response->success = true;
  response->message = "arm requested success";
}
//}

/* srvDisarm() //{ */
void ApiNode::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if ((!control_manager_diagnostics_.is_fly && !control_manager_diagnostics_.have_goal) && api_diagnostics_.armed) {
    pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  } else {
    response->success = false;
    response->message = "arm request failed: UAV is already disarmed";
    return;
  }

  response->success = true;
  response->message = "disarm requested success";
}
//}

/* enuToNed() //{ */
Eigen::Vector3d ApiNode::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
//}

/* frdToFlu() //{ */
Eigen::Vector3d ApiNode::frdToFlu(Eigen::Vector3d p) {
  return frd_flu_affine_ * p;
}
//}

/* enuToNedOrientation() //{ */
Eigen::Quaterniond ApiNode::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
//}
}  // namespace laser_uav_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_px4_api::ApiNode)
