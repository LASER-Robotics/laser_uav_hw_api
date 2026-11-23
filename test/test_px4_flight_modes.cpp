#include <laser_uav_hw_api/px4_api_node.hpp>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_flight_modes(const std::string & control_input_mode)
{
  rclcpp::NodeOptions options;
  const std::string ns = "/flight_mode_test_" + control_input_mode;
  options.arguments(
  {
    "--ros-args", "-r", "__ns:=" + ns, "-r", "land:=hw_api/land",
    "-r", "control_manager_land:=control_manager/land",
    "-r", "vehicle_status_px4_in:=fmu/out/vehicle_status_v1"});
  options.parameter_overrides(
  {
    {"uav_name", "uav3"}, {"control_input_mode", control_input_mode},
    {"rc.aux_logics", false}, {"tf.publish", false}});
  auto api = std::make_shared<laser_uav_hw_api::Px4ApiNode>(options);
  auto probe = std::make_shared<rclcpp::Node>("probe", ns);
  auto client = probe->create_client<std_srvs::srv::Trigger>("hold");
  auto land_client = probe->create_client<std_srvs::srv::Trigger>("hw_api/land");
  size_t control_manager_land_calls = 0;
  auto control_manager_land = probe->create_service<std_srvs::srv::Trigger>(
    "control_manager/land",
    [&](const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      ++control_manager_land_calls;
      response->success = true;
    });
  auto status_pub = probe->create_publisher<px4_msgs::msg::VehicleStatus>(
    "fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS());
  auto control_pub = probe->create_publisher<px4_msgs::msg::VehicleControlMode>(
    "vehicle_control_mode_px4_in", rclcpp::SensorDataQoS());
  auto motor_pub = probe->create_publisher<laser_msgs::msg::MotorSpeedStamped>(
    "motor_speed_reference_in", 10);
  auto rates_pub = probe->create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>(
    "attitude_rates_thrust_in", 10);
  std::vector<px4_msgs::msg::VehicleCommand> commands;
  auto command_sub = probe->create_subscription<px4_msgs::msg::VehicleCommand>(
    "vehicle_command_px4_out", 10,
    [&](const px4_msgs::msg::VehicleCommand & msg) {commands.push_back(msg);});
  size_t heartbeats = 0;
  auto heartbeat_sub = probe->create_subscription<px4_msgs::msg::OffboardControlMode>(
    "offboard_control_mode_px4_out", 10,
    [&](const px4_msgs::msg::OffboardControlMode &) {++heartbeats;});
  size_t controls = 0;
  auto motors_sub = probe->create_subscription<px4_msgs::msg::ActuatorMotors>(
    "motor_speed_reference_px4_out", 10,
    [&](const px4_msgs::msg::ActuatorMotors &) {++controls;});
  auto rates_sub = probe->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
    "attitude_rates_reference_px4_out", 10,
    [&](const px4_msgs::msg::VehicleRatesSetpoint &) {++controls;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(api->get_node_base_interface());
  executor.add_node(probe);
  auto pump_until = [&](auto predicate, std::chrono::milliseconds timeout = 1s) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        if (predicate()) {
          return true;
        }
        std::this_thread::sleep_for(5ms);
      }
      return predicate();
    };
  auto spin_for = [&](std::chrono::milliseconds duration) {
      pump_until([]() {return false;}, duration);
    };
  auto request_hold = [&]() {
      return client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    };
  auto request_land = [&]() {
      return land_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    };
  auto ready = [](const auto & future) {return future.wait_for(0s) == std::future_status::ready;};

  api->configure();
  require(
    pump_until([&]() {return client->service_is_ready();}, 2s),
    std::string("Hold service missing: ") + client->get_service_name());
  require(pump_until([&]() {return land_client->service_is_ready();}, 2s), "Land service missing");
  require(
    pump_until(
      [&]() {
        return command_sub->get_publisher_count() == 1 &&
        status_pub->get_subscription_count() == 1 &&
        control_pub->get_subscription_count() == 1;
      }, 2s), "PX4 topic discovery failed");
  auto inactive = request_hold();
  require(pump_until([&]() {return ready(inactive);}), "Inactive request did not respond");
  require(!inactive.get()->success && commands.empty(), "Inactive API sent a Hold command");
  auto inactive_land = request_land();
  require(pump_until([&]() {return ready(inactive_land);}), "Inactive Land did not respond");
  require(!inactive_land.get()->success && commands.empty(), "Inactive API sent a Land command");

  api->activate();
  px4_msgs::msg::VehicleStatus status;
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status.pre_flight_checks_pass = true;
  px4_msgs::msg::VehicleControlMode control;
  control.flag_control_offboard_enabled = true;
  laser_msgs::msg::MotorSpeedStamped motor;
  motor.data.data = {0.2, 0.2, 0.2, 0.2};
  laser_msgs::msg::AttitudeRatesAndThrust rates;
  rates.total_thrust_normalized = 0.2;
  auto publish_inputs = [&]() {
      status_pub->publish(status);
      control_pub->publish(control);
      motor_pub->publish(motor);
      rates_pub->publish(rates);
    };
  require(
    pump_until([&]() {publish_inputs(); return controls > 0;}),
    "Offboard control inputs were not forwarded");

  auto hold = request_hold();
  require(pump_until([&]() {return commands.size() == 1;}), "No Hold command sent");
  const auto & command = commands.back();
  require(
    command.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
    command.param1 == 1.0f && command.param2 == 4.0f && command.param3 == 3.0f,
    "Hold command does not select AUTO/LOITER");
  require(
    command.target_system == 3 && command.target_component == 1 && command.from_external,
    "Hold command has the wrong PX4 target");
  auto duplicate = request_hold();
  require(pump_until([&]() {return ready(duplicate);}), "Duplicate request did not respond");
  require(
    !duplicate.get()->success && commands.size() == 1,
    "Duplicate request replaced the pending Hold");
  const auto previous_heartbeats = heartbeats;
  publish_inputs();
  spin_for(150ms);
  require(!ready(hold), "Hold succeeded before AUTO_LOITER telemetry");
  require(heartbeats > previous_heartbeats, "Hold request blocked the Offboard heartbeat");

  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
  control.flag_control_offboard_enabled = false;
  require(pump_until([&]() {publish_inputs(); return ready(hold);}), "Hold was not confirmed");
  require(hold.get()->success, "Confirmed AUTO_LOITER reported failure");
  spin_for(100ms);
  const auto held_controls = controls;
  publish_inputs();
  spin_for(100ms);
  require(controls == held_controls, "API forwarded Offboard controls in Hold");

  auto land = request_land();
  require(pump_until([&]() {return commands.size() == 2;}), "No native Land command sent");
  const auto & land_command = commands.back();
  require(
    land_command.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND &&
    land_command.target_system == 3 && land_command.target_component == 1 &&
    land_command.from_external, "Native Land command has the wrong command or target");
  const auto landing_heartbeats = heartbeats;
  publish_inputs();
  spin_for(150ms);
  require(!ready(land), "Hold telemetry incorrectly confirmed Land");
  require(heartbeats > landing_heartbeats, "Land request blocked the heartbeat");
  auto duplicate_land = request_land();
  auto hold_during_land = request_hold();
  require(
    pump_until([&]() {return ready(duplicate_land) && ready(hold_during_land);}),
    "Pending Land did not reject conflicting requests");
  require(
    !duplicate_land.get()->success && !hold_during_land.get()->success && commands.size() == 2,
    "Pending Land was replaced by another request");
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  require(pump_until([&]() {publish_inputs(); return ready(land);}), "Land was not confirmed");
  require(land.get()->success, "Confirmed AUTO_LAND reported failure");

  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status_pub->publish(status);
  spin_for(100ms);
  auto timeout = request_hold();
  require(pump_until([&]() {return ready(timeout);}, 4s), "Unconfirmed Hold did not time out");
  const auto timeout_response = timeout.get();
  require(
    !timeout_response->success && timeout_response->message.find("3 seconds") != std::string::npos,
    "Missing mode confirmation did not report a timeout");

  auto land_timeout = request_land();
  require(pump_until([&]() {return ready(land_timeout);}, 4s), "Unconfirmed Land did not time out");
  require(!land_timeout.get()->success, "Unconfirmed Land reported success");

  const auto before_preemption = commands.size();
  auto preempted_hold = request_hold();
  require(
    pump_until(
      [&]() {
        return commands.size() > before_preemption;
      }), "Hold was not requested");
  auto priority_land = request_land();
  require(
    pump_until(
      [&]() {
        return ready(preempted_hold) && commands.size() == before_preemption + 2;
      }), "Land did not interrupt the pending Hold");
  require(
    !preempted_hold.get()->success &&
    commands.back().command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
    "Land did not take priority over Hold");
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
  publish_inputs();
  spin_for(100ms);
  require(!ready(priority_land), "Delayed Hold telemetry incorrectly confirmed priority Land");
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  require(
    pump_until([&]() {publish_inputs(); return ready(priority_land);}),
    "Priority Land was not confirmed");
  require(priority_land.get()->success, "Priority Land reported failure");
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status_pub->publish(status);
  spin_for(100ms);

  const auto previous_commands = commands.size();
  auto interrupted = request_hold();
  require(
    pump_until([&]() {return commands.size() > previous_commands;}),
    "Could not retry after timeout");
  api->deactivate();
  require(pump_until([&]() {return ready(interrupted);}), "Deactivation left a request pending");
  require(!interrupted.get()->success, "Deactivation reported Hold success");
  api->cleanup();
  api->configure();
  api->activate();
  const auto commands_before_shutdown = commands.size();
  auto shutdown = request_land();
  require(
    pump_until([&]() {return commands.size() > commands_before_shutdown;}, 2s),
    "Land service did not recover after cleanup/configure");
  api->shutdown();
  require(pump_until([&]() {return ready(shutdown);}), "Shutdown left a request pending");
  require(!shutdown.get()->success, "Shutdown reported Land success");
  require(control_manager_land_calls == 0, "Native Land called the external controller's service");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    test_flight_modes("individual_thrust");
    test_flight_modes("angular_rates_and_thrust");
    std::cout << "PASS: PX4 Hold/Land commands, priority, confirmation, heartbeat and lifecycle\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
