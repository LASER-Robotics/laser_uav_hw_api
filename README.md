# DEPRECATED

## LASER UAV PX4 API


This API serves as the communication bridge between the autopilot hardware and the high/low-level control systems on the companion computer, managing time synchronization and reference frame conversions.

-   **Subscribed Topics:**
    - `/$UAV_NAME/control_manager/motor_speed_reference`: Used to send normalized individual thrust commands when the controller is in the corresponding mode.
    - `/$UAV_NAME/control_manager/attitude_rates_thrust`: Used to send angular velocity setpoints and a total normalized thrust value when the controller is in the corresponding mode.

-   **Published Topics:**
    -   `/$UAV_NAME/px4_api/imu`: Publishes filtered data from the autopilot's internal IMU (Inertial Measurement Unit).
    -   `/$UAV_NAME/px4_api/motor_speed_estimated`: Publishes the current estimated speed of each motor.
    -   `/$UAV_NAME/px4_api/odometry`: Publishes the estimated state of the UAV, including its position, orientation, and velocities.

-   **Services:**
    - `/$UAV_NAME/px4_api/arm`: Service to arm the UAV's motors. **[SIMULATION ONLY]**
    - `/$UAV_NAME/px4_api/disarm`: Service to disarm the UAV's motors. **[SIMULATION ONLY]**

-   **Configurable Parameters:**
  ```yaml
    # Do not change these parameters
    rate: # [Hz]
      pub_offboard_control_mode: 100.0
      pub_api_diagnostic: 10.0

    # This parameter configures the control interface level with the firmware. 
    # If set to "individual_thrust", the firmware's internal control pipeline is completely bypassed. 
    # If set to "angular_rates_and_thrust", only the firmware's angular rate controller is utilized.
    control_input_mode: "individual_thrust" # Options: "individual_thrust" or "angular_rates_and_thrust"
