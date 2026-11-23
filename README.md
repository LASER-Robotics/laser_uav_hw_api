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

For PX4 1.17, build against the `px4_msgs` message definitions matching the firmware.
The API subscribes to `vehicle_status_v1`, while the FAST-LIO bridge sends
`VehicleOdometry` to `vehicle_visual_odometry`.

PX4 odometry uses a local origin taken from the first valid odometry sample, with earth-fixed ENU axes (`uav1/odometry`). The initial attitude does not rotate these axes or reset the heading: orientation is the PX4 attitude converted from NED/FRD to ENU/FLU, and twist is expressed in body FLU (`uav1/fcu`). Global ground-truth positions must also account for each UAV's spawn position when compared with this local origin.

The PX4 node consumes these topics under `/uav1/fmu/out/`:

| Topic suffix | Type |
| --- | --- |
| `vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` |
| `vehicle_gps_position` | `px4_msgs/msg/SensorGps` |
| `sensor_gyro`, `sensor_accel` | `px4_msgs/msg/SensorGyro`, `px4_msgs/msg/SensorAccel` |
| `distance_sensor` | `px4_msgs/msg/DistanceSensor` |
| `vehicle_status_v1`, `vehicle_control_mode` | `px4_msgs/msg/VehicleStatus`, `px4_msgs/msg/VehicleControlMode` |
| `esc_status` | `px4_msgs/msg/EscStatus`; used on real hardware. |
| `manual_control_setpoint` | `px4_msgs/msg/ManualControlSetpoint`; used when `rc.aux_logics=true`. |

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/control_manager/motor_speed_reference` | `laser_msgs/msg/MotorSpeedStamped` | Normalized individual motor commands in `individual_thrust` mode. |
| Input | `/uav1/control_manager/diagnostics` | `laser_msgs/msg/UavControlDiagnostics` | Control state used by the API. |
| Output | `/uav1/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Firmware commands. |
| Output | `/uav1/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Selects and maintains the offboard control interface. |
| Output | `/uav1/fmu/in/actuator_motors` | `px4_msgs/msg/ActuatorMotors` | Individual motor commands. |
| Output | `/uav1/fmu/in/vehicle_rates_setpoint` | `px4_msgs/msg/VehicleRatesSetpoint` | Body-rate and thrust commands. |
| Output | `/uav1/hw_api/motor_speed_estimated` | `laser_msgs/msg/MotorSpeedStamped` | Motor-speed estimates from ESC telemetry on real hardware. |
| Output | `/uav1/hw_api/garmin` | `sensor_msgs/msg/Range` | Rangefinder measurements. |
| Output | `/uav1/control_manager/goto_relative` | `laser_msgs/msg/PoseWithHeading` | RC-derived relative commands when auxiliary logic is enabled. |

The PX4 node also calls `/uav1/control_manager/land` (`std_srvs/srv/Trigger`). Its RC-derived goal topic still uses the older control-manager path; the planning-manager interface uses `/uav1/planning_manager/goto_relative`.

### PX4 Hold

Request the firmware's native Hold mode (`AUTO_LOITER`):

```bash
ros2 service call /uav1/hw_api/hold std_srvs/srv/Trigger '{}'
```

This PX4-only service sends `VEHICLE_CMD_DO_SET_MODE` with custom main mode `AUTO` (4)
and submode `LOITER` (3). It returns `success=true` only after a subsequent
`vehicle_status_v1` message reports `AUTO_LOITER`. An inactive API, another pending
Hold request, lifecycle interruption, or no confirmation within 3 seconds returns
`success=false`. A timeout does not cancel the command already sent to PX4.
The wait is asynchronous: the Offboard heartbeat continues while PX4 decides
whether to switch. Existing control commands are gated by PX4's reported
`vehicle_control_mode`; in Hold the firmware controls the vehicle.

The PX4 startup must run `land_detector start multicopter`. Without its telemetry,
the position controller assumes ground contact and repeatedly resets the takeoff
ramp, producing periodic thrust cuts in Hold. For `individual_thrust`, the land
detector must also reject missing or stale `vehicle_thrust_setpoint` data; otherwise
it can report a false landing and disarm in flight. This is addressed upstream in
[PX4 PR #26854](https://github.com/PX4/PX4-Autopilot/pull/26854). The companion remains
responsible for landing detection and disarming during direct actuator control.

Hold requires a valid global position estimate. Without GPS, feed independent
local odometry to `/uav1/fmu/in/vehicle_visual_odometry`, configure EKF2 fusion,
and set the EKF's geographic origin. Changing the ROS estimation-manager source
alone does not change PX4's estimator. This service does not configure EKF2,
inject odometry, arm the vehicle, or request a return to Offboard. See the PX4
[Hold documentation](https://docs.px4.io/v1.17/en/flight_modes_mc/hold) and
[external positioning setup](https://docs.px4.io/v1.15/en/ros/external_position_estimation#enabling-auto-modes-with-a-local-position).
Check `NAV_MIN_LTR_ALT`: Hold can climb to this minimum altitude above Home.

### PX4 Land

Request the firmware's native landing at the current position, including from Hold:

```bash
ros2 service call /uav1/hw_api/land std_srvs/srv/Trigger '{}'
```

This PX4-only service sends `VEHICLE_CMD_NAV_LAND` (21). It returns `success=true`
when a subsequent `vehicle_status_v1` reports `AUTO_LAND` (18), confirming entry into
landing mode rather than touchdown. The firmware manages descent and landing
detection; automatic disarming depends on `COM_DISARM_LAND`.

Like Hold, the response is asynchronous and times out after 3 seconds without mode
confirmation. An inactive API, lifecycle interruption, or another pending Land
request returns an error. Land interrupts a pending Hold request; Hold is rejected
while Land confirmation is pending. A timeout does not cancel the firmware command.

`/uav1/control_manager/land` remains the external controller's landing service and
is still used by the RC auxiliary landing trigger. `/uav1/hw_api/land` uses the
PX4 controller and does not require a return to Offboard. Keep the multicopter
land detector running with the direct-actuator thrust-validity fix described above.

### FAST-LIO to PX4 (experimental SITL setup)

The standalone bridge consumes the existing FAST-LIO output without modifying its
estimator or using the estimation manager. Start it alongside the existing API:

```bash
ros2 launch laser_uav_hw_api fast_lio_px4_bridge.launch.py uav_name:=uav1
```

| Direction | Topic | Type |
| --- | --- | --- |
| Input | `/uav1/fast_lio/odometry_high_freq` | `nav_msgs/msg/Odometry` |
| Input | `/uav1/fast_lio/odometry` | `nav_msgs/msg/Odometry`; LiDAR correction watchdog. |
| Output | `/uav1/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` |

This version targets the existing `livox_simulation` configuration:
`transform.enable=true`, `transform.enable_imu=true`, identity LiDAR/IMU
extrinsics, and `velocity_in_body_frame=true`. Both sensor inputs are already
transformed to the FCU. The bridge accepts `uav1/world` as the parent and either
the current `uav1/livox_up/imu` label or the corrected `uav1/fcu` child label.
Change `uav_name` for another UAV namespace. A different sensor configuration
needs its frame contract checked before using this bridge.

World and body axes are converted from FLU to FRD (`x, -y, -z`), including
orientation and body twist. The outgoing pose uses `POSE_FRAME_FRD`, with an
arbitrary world heading, and velocity uses `VELOCITY_FRAME_BODY_FRD`. The bridge
does not assert that FAST-LIO's world is aligned with geographic north.
Positive covariance diagonals are preserved; absent, zero or invalid variances
become NaN so EKF2 uses its configured measurement noise. Quality remains unknown
(0); use the default `EKF2_EV_QMIN=0`.

The original measurement stamp is sent as `timestamp_sample` in microseconds;
`timestamp` uses the bridge's current ROS time. No samples are repeated or
restamped. Frames, pose, timestamps and LiDAR correction freshness are checked.
Data older than 0.5 seconds, more than 0.05 seconds in the future, duplicate or
out-of-order high-frequency samples are withheld. If LiDAR corrections stop for
0.5 seconds, publishing stops even if IMU propagation continues. PX4's configured
position-loss behavior then applies. Restart FAST-LIO and the bridge together on
the ground after a map reset or simulation clock reset; automatic reset detection
is not implemented.

Configure PX4 on the ground **before starting the bridge**. The local firmware's
default `EKF2_EV_CTRL=15` would otherwise enable velocity and yaw fusion as soon
as data arrive. For the first test, use position and height fusion only:

```text
param set EKF2_EV_CTRL 3
param set EKF2_HGT_REF 3
param set EKF2_EV_POS_X 0
param set EKF2_EV_POS_Y 0
param set EKF2_EV_POS_Z 0
```

These offsets are zero because this setup reports the FCU pose. For GPS-free
position aiding, also set `EKF2_GPS_CTRL=0`. This does not disable the PX4 IMU,
magnetometer or barometer. Without external yaw fusion, EKF2 still needs a valid
heading source. Record the previous parameter values before changing them and
restart PX4 for settings that require a reboot.

The launch uses `use_sim_time=true`; ROS must receive Gazebo's `/clock`. When
PX4 and ROS share that simulation clock, set `UXRCE_DDS_SYNCT=0` in PX4 to avoid
applying another clock offset. For wall-clock operation, use
`use_sim_time:=false` and PX4's normal DDS time synchronization. See the official
[ROS/PX4 clock configuration](https://docs.px4.io/v1.15/en/ros2/user_guide#ros-gazebo-and-px4-time-synchronization).

For native Hold without GPS, set the EKF's geographic origin after startup using
the actual origin of the simulation/map, in the PX4 console:

```text
commander set_ekf_origin <latitude_deg> <longitude_deg> <altitude_AMSL_m>
```

Check `listener vehicle_visual_odometry`, `listener estimator_status_flags`
(`cs_ev_pos=true`, `cs_ev_hgt=true`, `cs_gps=false`) and
`listener vehicle_local_position` (`xy_valid`, `z_valid` and a global reference).
Confirm a valid global position before calling `/uav1/hw_api/hold`; merely
receiving an external odometry message does not prove EKF2 fused it. The same
native Land service remains available. See the official
[external positioning configuration](https://docs.px4.io/v1.15/en/ros/external_position_estimation).

The existing high-frequency integration and covariance limitations in FAST-LIO
remain unchanged. The bridge also forwards its current velocity, but the initial
`EKF2_EV_CTRL=3` setup does not fuse that velocity or external yaw. Flight without
GPS and the position-loss response still need SITL validation.

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
