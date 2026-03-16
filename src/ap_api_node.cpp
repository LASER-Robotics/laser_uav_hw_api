#include "laser_uav_hw_api/ap_api_node.hpp"

namespace laser_uav_hw_api
{
/* ApApiNode() //{ */
ApApiNode::ApApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("ap_api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("control_input_mode", rclcpp::ParameterValue(""));
  declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
  declare_parameter("rate.pub_api_diagnostics", rclcpp::ParameterValue(10.0));

  ned_enu_quaternion_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_rotation_            = Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  frd_flu_affine_              = Eigen::Affine3d(frd_flu_rotation_);

  ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_  = Eigen::DiagonalMatrix<double, 3>(1, 1, -1);

  const char *real_uav = std::getenv("REAL_UAV");
  /* target_system_       = 1; */
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
          /* target_system_         = std::stoi(std::string(match[1])); */
        }
      }
    } else {
      real_uav_ = true;
    }
  }
}
//}

/* ~ApApiNode() //{ */
ApApiNode::~ApApiNode() {
}
//}

/* on_configure() //{ */
CallbackReturn ApApiNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ApApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  /* pub_vehicle_command_ap_->on_activate(); */
  /* pub_offboard_control_mode_ap_->on_activate(); */
  pub_api_diagnostics_->on_activate();
  pub_nav_odometry_->on_activate();
  pub_imu_->on_activate();
  /* pub_motor_speed_estimation_->on_activate(); */

  /* if (_control_input_mode_ == "individual_thrust") { */
  /*   pub_motor_speed_reference_ap_->on_activate(); */
  /* } else if (_control_input_mode_ == "angular_rates_and_thrust") { */
  pub_attitude_rates_reference_ap_->on_activate();
  /* } */

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ApApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  /* pub_vehicle_command_ap_->on_deactivate(); */
  /* pub_offboard_control_mode_ap_->on_deactivate(); */
  pub_nav_odometry_->on_deactivate();
  pub_imu_->on_deactivate();
  pub_api_diagnostics_->on_deactivate();
  /* pub_motor_speed_estimation_->on_deactivate(); */

  /* if (_control_input_mode_ == "individual_thrust") { */
  /*   pub_motor_speed_reference_ap_->on_deactivate(); */
  /* } else if (_control_input_mode_ == "angular_rates_and_thrust") { */
  pub_attitude_rates_reference_ap_->on_deactivate();
  /* } */

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ApApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  sub_estimated_pose_ap_.reset();
  sub_estimated_twist_ap_.reset();
  sub_imu_ap_.reset();
  sub_vehicle_status_ap_.reset();
  /* sub_control_mode_ap_.reset(); */
  /* sub_esc_status_ap_.reset(); */

  /* pub_offboard_control_mode_ap_.reset(); */
  /* pub_nav_odometry_.reset(); */
  pub_imu_.reset();
  pub_api_diagnostics_.reset();
  /* pub_motor_speed_estimation_.reset(); */

  /* tmr_pub_offboard_control_mode_ap_.reset(); */
  /* tmr_pub_motor_speed_reference_ap_.reset(); */
  tmr_pub_api_diagnostics_.reset();

  /* if (_control_input_mode_ == "individual_thrust") { */
  /*   pub_motor_speed_reference_ap_.reset(); */
  /*   sub_motor_speed_reference_.reset(); */
  /* } else if (_control_input_mode_ == "angular_rates_and_thrust") { */
  pub_attitude_rates_reference_ap_.reset();
  sub_attitude_rates_and_thrust_reference_.reset();
  /* } */

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn ApApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void ApApiNode::getParameters() {
  /* get_parameter("control_input_mode", _control_input_mode_); */
  /* get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_ap_); */
  get_parameter("rate.pub_api_diagnostics", _rate_pub_api_diagnostics_);
}
//}

/* configPubSub() //{ */
void ApApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for ap topics
  sub_estimated_pose_ap_  = create_subscription<geometry_msgs::msg::PoseStamped>("estimated_pose_ap_in", rclcpp::SensorDataQoS(),
                                                                                std::bind(&ApApiNode::subEstimatedPoseAp, this, std::placeholders::_1));
  sub_estimated_twist_ap_ = create_subscription<geometry_msgs::msg::TwistStamped>("estimated_twist_ap_in", rclcpp::SensorDataQoS(),
                                                                                  std::bind(&ApApiNode::subEstimatedTwistAp, this, std::placeholders::_1));

  sub_imu_ap_ = create_subscription<sensor_msgs::msg::Imu>("imu_ap_in", rclcpp::SensorDataQoS(), std::bind(&ApApiNode::subImuAp, this, std::placeholders::_1));

  sub_vehicle_status_ap_ = create_subscription<ardupilot_msgs::msg::Status>("vehicle_status_ap_in", rclcpp::SensorDataQoS(),
                                                                            std::bind(&ApApiNode::subVehicleStatusAp, this, std::placeholders::_1));

  // Pubs and Subs for System topics
  pub_api_diagnostics_ = create_publisher<laser_msgs::msg::ApiPx4Diagnostics>("api_diagnostics", 10);

  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  /* pub_motor_speed_estimation_ = create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_out", 10); */

  /* if (_control_input_mode_ == "individual_thrust") { */
  /* pub_motor_speed_reference_ap_ = create_publisher<ap_msgs::msg::ActuatorMotors>("motor_speed_reference_ap_out", 10); */
  /* sub_motor_speed_reference_    = create_subscription<laser_msgs::msg::MotorSpeed>("motor_speed_reference_in", 1, */
  /*                                                                               std::bind(&ApApiNode::subMotorSpeedReference, this, std::placeholders::_1));
   */
  /* } else if (_control_input_mode_ == "angular_rates_and_thrust") { */
  pub_attitude_rates_reference_ap_         = create_publisher<ardupilot_msgs::msg::AttitudeTarget>("attitude_rates_reference_ap_out", 10);
  sub_attitude_rates_and_thrust_reference_ = create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
      "attitude_rates_thrust_in", 1, std::bind(&ApApiNode::subAttitudeRatesAndThrustReference, this, std::placeholders::_1));
  /* } */
}
//}

/* configTimers() //{ */
void ApApiNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  /* tmr_pub_offboard_control_mode_ap_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_ap_), */
  /* std::bind(&ApApiNode::tmrPubOffboardControlModeap, this), nullptr); */
  tmr_pub_api_diagnostics_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_api_diagnostics_), std::bind(&ApApiNode::tmrPubApiDiagnostics, this), nullptr);
  /* if (_control_input_mode_ == "individual_thrust") { */
  /*   tmr_pub_motor_speed_reference_ap_ = */
  /*       create_wall_timer(std::chrono::duration<double>(1.0 / 500), std::bind(&ApApiNode::tmrPubMotorSpeedReferenceap, this), nullptr); */
  /* } */
}
//}

/* configServices() //{ */
void ApApiNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_arm_    = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));

  clt_arm_motors_ap_  = create_client<ardupilot_msgs::srv::ArmMotors>("arm_motors_ap");
  clt_mode_switch_ap_ = create_client<ardupilot_msgs::srv::ModeSwitch>("mode_switch_ap");
}
//}

/* /1* subEscStatusap() //{ *1/ */
/* void ApApiNode::subEscStatusap(const ap_msgs::msg::EscStatus &msg) { */
/*   if (!is_active_) { */
/*     return; */
/*   } */

/*   laser_msgs::msg::MotorSpeed motor_speed_estimation; */

/*   for (auto i = 0; i < (int)msg.esc_count; i++) { */
/*     motor_speed_estimation.data.push_back(msg.esc[i].esc_rpm * 0.1047); */
/*   } */
/*   motor_speed_estimation.unit_of_measurement = "rad/s"; */

/*   pub_motor_speed_estimation_->publish(motor_speed_estimation); */
/* } */
/* //} */

/* subImuAp() //{ */
void ApApiNode::subImuAp(const sensor_msgs::msg::Imu &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.angular_velocity.x = frd_to_flu(0);
  imu_.angular_velocity.y = frd_to_flu(1);
  imu_.angular_velocity.z = frd_to_flu(2);

  frd_to_flu << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.linear_acceleration.x = frd_to_flu(0);
  imu_.linear_acceleration.y = frd_to_flu(1);
  imu_.linear_acceleration.z = frd_to_flu(2);

  imu_.header.stamp    = get_clock()->now();
  imu_.header.frame_id = "fcu";
  pub_imu_->publish(imu_);
}
//}

/* subVehicleStatusAp() //{ */
void ApApiNode::subVehicleStatusAp(const ardupilot_msgs::msg::Status &msg) {
  if (!is_active_) {
    return;
  }

  api_diagnostics_.armed         = msg.armed;
  offboard_is_enabled_           = msg.mode == 4 ? true : false;
  api_diagnostics_.offboard_mode = offboard_is_enabled_;

  /* if (!fw_preflight_checks_pass_) { */
  /* fw_preflight_checks_pass_ = msg.pre_flight_checks_pass; */

  /* RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current situation of the preflight checks on Autopilot: %s", */
  /*                      fw_preflight_checks_pass_ ? "True" : "False"); */
  /* } else { */
  /* RCLCPP_INFO_ONCE(this->get_logger(), "Current situation of the preflight checks on Autopilot: %s", fw_preflight_checks_pass_ ? "True" : "False"); */
  /* } */
}
//}

/* subEstimatedPoseAp() //{ */
void ApApiNode::subEstimatedPoseAp(const geometry_msgs::msg::PoseStamped &msg) {
  if (!is_active_) {
    return;
  }

  /* if (!fw_preflight_checks_pass_) { */
  /*   return; */
  /* } */

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.header.frame_id = "odom";
  current_nav_odometry.header.stamp    = get_clock()->now();
  current_nav_odometry.child_frame_id  = "fcu";

  current_nav_odometry.pose.pose = msg.pose;

  Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  Eigen::Vector3d twist_body_frame;
  twist_body_frame(0) = twist_stamped_ap_.twist.linear.x;
  twist_body_frame(1) = twist_stamped_ap_.twist.linear.y;
  twist_body_frame(2) = twist_stamped_ap_.twist.linear.z;
  q.normalize();
  twist_body_frame = q.conjugate().toRotationMatrix() * twist_body_frame;

  current_nav_odometry.twist.twist.linear.x = twist_body_frame(0);
  current_nav_odometry.twist.twist.linear.y = twist_body_frame(1);
  current_nav_odometry.twist.twist.linear.z = twist_body_frame(2);

  Eigen::Vector3d flu_to_frd;
  flu_to_frd << twist_stamped_ap_.twist.angular.x, twist_stamped_ap_.twist.angular.y, twist_stamped_ap_.twist.angular.z;

  current_nav_odometry.twist.twist.angular.x = flu_to_frd(0);
  current_nav_odometry.twist.twist.angular.y = flu_to_frd(1);
  current_nav_odometry.twist.twist.angular.z = flu_to_frd(2);

  pub_nav_odometry_->publish(current_nav_odometry);
}
//}

/* subEstimatedTwistAp() //{ */
void ApApiNode::subEstimatedTwistAp(const geometry_msgs::msg::TwistStamped &msg) {
  if (!is_active_) {
    return;
  }

  /* if (!fw_preflight_checks_pass_) { */
  /*   return; */
  /* } */

  twist_stamped_ap_ = msg;
}
//}

/* /1* tmrPubOffboardControlModeap() //{ *1/ */
/* void ApApiNode::tmrPubOffboardControlModeap() { */
/*   if (!is_active_) { */
/*     return; */
/*   } */

/*   ap_msgs::msg::OffboardControlMode msg{}; */

/*   msg.position          = false; */
/*   msg.velocity          = false; */
/*   msg.acceleration      = false; */
/*   msg.attitude          = false; */
/*   msg.thrust_and_torque = false; */
/*   if (_control_input_mode_ == "individual_thrust") { */
/*     msg.body_rate       = false; */
/*     msg.direct_actuator = true; */
/*   } else if (_control_input_mode_ == "angular_rates_and_thrust") { */
/*     msg.body_rate       = true; */
/*     msg.direct_actuator = false; */
/*   } */
/*   msg.timestamp = get_clock()->now().nanoseconds() / 1000; */


/*   pub_offboard_control_mode_ap_->publish(msg); */
/* } */
/* //} */

/* /1* pubVehicleCommandap() //{ *1/ */
/* void ApApiNode::pubVehicleCommandap(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) { */
/*   if (!is_active_) { */
/*     return; */
/*   } */

/*   ap_msgs::msg::VehicleCommand msg{}; */
/*   msg.param1           = param1; */
/*   msg.param2           = param2; */
/*   msg.param3           = param3; */
/*   msg.param4           = param4; */
/*   msg.param5           = param5; */
/*   msg.param6           = param6; */
/*   msg.param7           = param7; */
/*   msg.command          = command; */
/*   msg.target_system    = target_system_; */
/*   msg.target_component = 1; */
/*   msg.source_system    = 1; */
/*   msg.source_component = 1; */
/*   msg.from_external    = true; */
/*   msg.timestamp        = get_clock()->now().nanoseconds() / 1000; */
/*   pub_vehicle_command_ap_->publish(msg); */
/* } */
/* //} */

/* tmrPubApiDiagnostic() //{ */
void ApApiNode::tmrPubApiDiagnostics() {
  if (!is_active_) {
    return;
  }

  pub_api_diagnostics_->publish(api_diagnostics_);
}
//}

/* subAttitudeRatesAndThrustReference() //{ */
void ApApiNode::subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg) {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  /* if (!fw_preflight_checks_pass_) { */
  /*   RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!"); */
  /*   return; */
  /* } */

  ardupilot_msgs::msg::AttitudeTarget attitude_rates_reference{};
  attitude_rates_reference.type_mask = 128;

  Eigen::Vector3d flu_to_frd;
  flu_to_frd << msg.roll_rate, msg.pitch_rate, msg.yaw_rate;
  flu_to_frd = frdToFlu(flu_to_frd);

  attitude_rates_reference.body_rate.x = flu_to_frd(0);
  attitude_rates_reference.body_rate.y = flu_to_frd(1);
  attitude_rates_reference.body_rate.z = flu_to_frd(2);
  attitude_rates_reference.thrust      = msg.total_thrust_normalized;

  attitude_rates_reference.header.stamp = get_clock()->now();

  pub_attitude_rates_reference_ap_->publish(attitude_rates_reference);
}
//}

/* /1* subMotorSpeedReference() //{ *1/ */
/* void ApApiNode::subMotorSpeedReference(const laser_msgs::msg::MotorSpeed &msg) { */
/*   if (!is_active_) { */
/*     return; */
/*   } */

/*   if (!offboard_is_enabled_) { */
/*     return; */
/*   } */

/*   if (!fw_preflight_checks_pass_) { */
/*     RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!"); */
/*     return; */
/*   } */

/*   for (auto i = 0; i < (int)msg.data.size(); i++) { */
/*     actuator_motors_reference_.control[i] = msg.data[i]; */
/*   } */

/*   actuator_motors_reference_.timestamp        = get_clock()->now().nanoseconds() / 1000; */
/*   actuator_motors_reference_.timestamp_sample = 0; */
/* } */
/* //} */

/* /1* tmrPubMotorSpeedReferenceap() //{ *1/ */
/* void ApApiNode::tmrPubMotorSpeedReferenceap() { */
/*   if (!is_active_) { */
/*     return; */
/*   } */

/*   if (!offboard_is_enabled_) { */
/*     return; */
/*   } */

/*   if (!fw_preflight_checks_pass_) { */
/*     return; */
/*   } */

/*   if (_control_input_mode_ != "individual_thrust") { */
/*     return; */
/*   } */

/*   pub_motor_speed_reference_ap_->publish(actuator_motors_reference_); */
/* } */
/* //} */

/* srvArm() //{ */
void ApApiNode::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (real_uav_) {
    response->success = false;
    response->message = "arm requested failed, in real drone use the RC to arm";
    return;
  }

  /* if (!fw_preflight_checks_pass_) { */
  /*   response->success = false; */
  /*   response->message = "arm requested failed, preflight checks don't pass in firmware, try again in a few seconds"; */
  /*   return; */
  /* } */

  auto mode_request  = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
  mode_request->mode = 4;

  auto response_mode_switch = clt_mode_switch_ap_->async_send_request(mode_request);
  response_mode_switch.share().wait_for(std::chrono::milliseconds(500));

  auto arm_request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  arm_request->arm = true;

  auto response_arm_motors = clt_arm_motors_ap_->async_send_request(arm_request);
  response_arm_motors.share().wait_for(std::chrono::milliseconds(500));

  response->success = true;
  response->message = "arm requested success";
}
//}

/* srvDisarm() //{ */
void ApApiNode::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (real_uav_) {
    response->success = false;
    response->message = "disarm requested failed, in real drone use the RC to disarm";
    return;
  }

  auto arm_request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
  arm_request->arm = false;

  auto response_arm_motors = clt_arm_motors_ap_->async_send_request(arm_request);
  response_arm_motors.share().wait_for(std::chrono::milliseconds(500));

  response->success = true;
  response->message = "disarm requested success";
}
//}

/* enuToNed() //{ */
Eigen::Vector3d ApApiNode::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
//}

/* frdToFlu() //{ */
Eigen::Vector3d ApApiNode::frdToFlu(Eigen::Vector3d p) {
  return frd_flu_affine_ * p;
}
//}

/* enuToNedOrientation() //{ */
Eigen::Quaterniond ApApiNode::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
//}
}  // namespace laser_uav_hw_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_hw_api::ApApiNode)
