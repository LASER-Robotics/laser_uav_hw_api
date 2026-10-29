# LASER UAV Hardware API

`laser_uav_hw_api` connects the LASER UAV System (LUS) to PX4 or ArduPilot. It converts autopilot telemetry into the ROS messages used by the managers and sends the managers' control commands back to the flight controller, including the required frame conversions.

This package is marked as deprecated in the existing project documentation. The interfaces below describe the implementation in this repository.

## Run

Choose the launch file for your autopilot:

```bash
ros2 launch laser_uav_hw_api px4_api.launch.py uav_name:=uav1 real_uav:=false
```

```bash
ros2 launch laser_uav_hw_api ap_api.launch.py uav_name:=uav1 real_uav:=false
```

Each launch configures and activates its lifecycle node. Start the corresponding DDS agent and firmware separately, or use the simulation session that starts them for you.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `uav_name` | `UAV_NAME`, or `uav1` | UAV namespace. |
| `real_uav` | `REAL_UAV`, or `false` | Selects real-hardware behavior. |
| `use_sim_time` | `true` when `real_uav=false` | Uses the ROS simulation clock. |
| `api_file` | Autopilot-specific YAML in `params/` | Path to the node's parameter file. |

## Interfaces

Paths use `/uav1` as an example and include the remappings in the supplied launches. The PX4 and ArduPilot nodes share the `hw_api` output namespace, but use different firmware interfaces.

### Shared LUS interfaces

| Direction | Topic or service | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/control_manager/attitude_rates_thrust` | `laser_msgs/msg/AttitudeRatesAndThrust` | Body-rate and normalized thrust command. PX4 uses it in `angular_rates_and_thrust` mode; ArduPilot uses it directly. |
| Output | `/uav1/hw_api/odometry` | `nav_msgs/msg/Odometry` | Autopilot position, orientation and velocity. |
| Output | `/uav1/hw_api/imu` | `sensor_msgs/msg/Imu` | Autopilot IMU measurements. |
| Output | `/uav1/hw_api/diagnostics` | `laser_msgs/msg/ApiPx4Diagnostics` | Armed/offboard state and available firmware diagnostics. |
| Service | `/uav1/hw_api/arm` | `std_srvs/srv/Trigger` | Requests arming, subject to the API checks. |
| Service | `/uav1/hw_api/disarm` | `std_srvs/srv/Trigger` | Requests disarming, subject to the API checks. |

The ArduPilot API rejects arm and disarm requests when `real_uav=true`. The PX4 API can send these commands on hardware: arming requires firmware preflight checks and a non-flying, disarmed state; disarming requires a non-flying state with no active goal. PX4 switches to offboard automatically during an arm request only in simulation.

### PX4 interfaces

The PX4 node consumes these topics under `/uav1/fmu/out/`:

| Topic suffix | Type |
| --- | --- |
| `vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` |
| `vehicle_gps_position` | `px4_msgs/msg/SensorGps` |
| `sensor_gyro`, `sensor_accel` | `px4_msgs/msg/SensorGyro`, `px4_msgs/msg/SensorAccel` |
| `distance_sensor` | `px4_msgs/msg/DistanceSensor` |
| `vehicle_status`, `vehicle_control_mode` | `px4_msgs/msg/VehicleStatus`, `px4_msgs/msg/VehicleControlMode` |
| `esc_status` | `px4_msgs/msg/EscStatus`; used on real hardware. |
| `manual_control_setpoint` | `px4_msgs/msg/ManualControlSetpoint`; used when `rc.aux_logics=true`. |

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/control_manager/motor_speed_reference` | `laser_msgs/msg/MotorSpeed` | Normalized individual motor commands in `individual_thrust` mode. |
| Input | `/uav1/control_manager/diagnostics` | `laser_msgs/msg/UavControlDiagnostics` | Control state used by the API. |
| Output | `/uav1/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Firmware commands. |
| Output | `/uav1/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Selects and maintains the offboard control interface. |
| Output | `/uav1/fmu/in/actuator_motors` | `px4_msgs/msg/ActuatorMotors` | Individual motor commands. |
| Output | `/uav1/fmu/in/vehicle_rates_setpoint` | `px4_msgs/msg/VehicleRatesSetpoint` | Body-rate and thrust commands. |
| Output | `/uav1/hw_api/motor_speed_estimated` | `laser_msgs/msg/MotorSpeed` | Motor-speed estimates from ESC telemetry on real hardware. |
| Output | `/uav1/hw_api/garmin` | `sensor_msgs/msg/Range` | Rangefinder measurements. |
| Output | `/uav1/control_manager/goto_relative` | `laser_msgs/msg/PoseWithHeading` | RC-derived relative commands when auxiliary logic is enabled. |

The PX4 node also calls `/uav1/control_manager/land` (`std_srvs/srv/Trigger`). Its RC-derived goal topic still uses the older control-manager path; the planning-manager interface uses `/uav1/planning_manager/goto_relative`.

### ArduPilot interfaces

| Direction | Topic or service | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/ap/pose/filtered` | `geometry_msgs/msg/PoseStamped` | Estimated pose. |
| Input | `/ap/twist/filtered` | `geometry_msgs/msg/TwistStamped` | Estimated velocity. |
| Input | `/ap/imu/experimental/data` | `sensor_msgs/msg/Imu` | IMU measurements. |
| Input | `/ap/status` | `ardupilot_msgs/msg/Status` | Armed state and flight mode. |
| Output | `/ap/experimental/target_attitude` | `ardupilot_msgs/msg/AttitudeTarget` | Body-rate and thrust command. |
| Service client | `/ap/arm_motors` | `ardupilot_msgs/srv/ArmMotors` | Firmware arming request. |
| Service client | `/ap/mode_switch` | `ardupilot_msgs/srv/ModeSwitch` | Firmware flight-mode request. |

The supplied ArduPilot launch uses absolute `/ap/...` paths. Changing `uav_name` alone does not give each ArduPilot instance a separate firmware namespace.

## Configuration

Use [params/px4_api.yaml](params/px4_api.yaml) or [params/ap_api.yaml](params/ap_api.yaml).

| Parameter | Meaning |
| --- | --- |
| `control_input_mode` | PX4 only: `individual_thrust` bypasses the firmware control allocation; `angular_rates_and_thrust` uses the firmware rate controller. Match the controller configuration. |
| `rate.pub_offboard_control_mode` | PX4 offboard-mode publication rate in Hz; supplied value is `100.0`. |
| `rate.pub_api_diagnostics` | Diagnostics publication rate in Hz; node default is `10.0`. |
| `rc.aux_logics` | Enables the PX4 RC auxiliary logic. |
| `tf.publish` | Enables odometry TF publication. Supplied files disable it. |
| `tf.inverted` | Selects the TF direction where supported by the implementation. |

Both supplied YAML files spell the diagnostics key `rate.pub_api_diagnostic` without the final `s`. The nodes read `rate.pub_api_diagnostics`; use that spelling in a custom file. The default remains 10 Hz.

Motor-speed feedback also needs matching message types: this PX4 API publishes `MotorSpeed`, while `control_manager` subscribes to `MotorSpeedStamped`. A topic remapping alone cannot connect those types.
