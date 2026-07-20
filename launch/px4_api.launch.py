"""
Launch file for the PX4 Hardware API Lifecycle Node.
Uses environment variables as defaults for launch arguments, allowing CLI overrides.
"""

import lifecycle_msgs.msg
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch.events import matches_action
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression


def generate_launch_description():
    # --- Launch Configuration Variables ---
    uav_name = LaunchConfiguration('uav_name')
    real_uav = LaunchConfiguration('real_uav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    api_file = LaunchConfiguration('api_file')

    # --- Declare Launch Arguments ---
    declared_arguments = [
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav1'),
            description='Namespace and target name for the UAV (default: $UAV_NAME or uav1).'
        ),
        DeclareLaunchArgument(
            'real_uav',
            default_value=EnvironmentVariable('REAL_UAV', default_value='false'),
            description='Set to true if running on real hardware; false for simulation.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(['"', LaunchConfiguration('real_uav'), '" == "false"']),
            description='Use simulation (Gazebo) clock if true (auto-evaluates based on real_uav).'
        ),
        DeclareLaunchArgument(
            'api_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('laser_uav_hw_api'),
                'params',
                'px4_api.yaml'
            ]),
            description='Full path to the ROS 2 parameters YAML file.'
        ),
    ]

    # --- Lifecycle Node Definition ---
    api_lifecycle_node = LifecycleNode(
        package='laser_uav_hw_api',
        executable='px4_api',
        name='px4_api',
        namespace=uav_name,
        output='screen',
        parameters=[
            api_file,
            {
                'use_sim_time': use_sim_time,
                'uav_name': uav_name,
                'real_uav': real_uav,
            }
        ],
        remappings=[
            (('/', uav_name, '/vehicle_command_px4_out'), ('/', uav_name, '/fmu/in/vehicle_command')),
            (('/', uav_name, '/attitude_rates_reference_px4_out'),
             ('/', uav_name, '/fmu/in/vehicle_rates_setpoint')),
            (('/', uav_name, '/motor_speed_reference_px4_out'),
             ('/', uav_name, '/fmu/in/actuator_motors')),
            (('/', uav_name, '/offboard_control_mode_px4_out'),
             ('/', uav_name, '/fmu/in/offboard_control_mode')),
            (('/', uav_name, '/vehicle_odometry_px4_in'), ('/', uav_name, '/fmu/out/vehicle_odometry')),
            (('/', uav_name, '/vehicle_gps_position_px4_in'),
             ('/', uav_name, '/fmu/out/vehicle_gps_position')),
            (('/', uav_name, '/sensor_gyro_px4_in'), ('/', uav_name, '/fmu/out/sensor_gyro')),
            (('/', uav_name, '/sensor_accel_px4_in'), ('/', uav_name, '/fmu/out/sensor_accel')),
            (('/', uav_name, '/distance_sensor_px4_in'), ('/', uav_name, '/fmu/out/distance_sensor')),
            (('/', uav_name, '/vehicle_status_px4_in'), ('/', uav_name, '/fmu/out/vehicle_status')),
            (('/', uav_name, '/vehicle_control_mode_px4_in'),
             ('/', uav_name, '/fmu/out/vehicle_control_mode')),
            (('/', uav_name, '/control_manager_diagnostics_in'),
             ('/', uav_name, '/control_manager/diagnostics')),
            (('/', uav_name, '/odometry'), ('/', uav_name, '/hw_api/odometry')),
            (('/', uav_name, '/imu'), ('/', uav_name, '/hw_api/imu')),
            (('/', uav_name, '/garmin'), ('/', uav_name, '/hw_api/garmin')),
            (('/', uav_name, '/arm'), ('/', uav_name, '/hw_api/arm')),
            (('/', uav_name, '/disarm'), ('/', uav_name, '/hw_api/disarm')),
            (('/', uav_name, '/land'), ('/', uav_name, '/control_manager/land')),
            (('/', uav_name, '/api_diagnostics'), ('/', uav_name, '/hw_api/diagnostics')),
            (('/', uav_name, '/motor_speed_estimation_out'),
             ('/', uav_name, '/hw_api/motor_speed_estimated')),
            (('/', uav_name, '/motor_speed_reference_in'),
             ('/', uav_name, '/control_manager/motor_speed_reference')),
            (('/', uav_name, '/attitude_rates_thrust_in'),
             ('/', uav_name, '/control_manager/attitude_rates_thrust')),
            (('/', uav_name, '/esc_status_px4_in'), ('/', uav_name, '/fmu/out/esc_status')),
            (('/', uav_name, '/px4_rc_in'), ('/', uav_name, '/fmu/out/manual_control_setpoint')),
            (('/', uav_name, '/rc_to_goto_out'), ('/', uav_name, '/control_manager/goto_relative')),
        ]
    )

    # --- Lifecycle Event Handlers ---
    event_handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=api_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=api_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    ]

    # --- Assemble Launch Description ---
    ld = LaunchDescription()

    for argument in declared_arguments:
        ld.add_action(argument)

    ld.add_action(api_lifecycle_node)

    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
