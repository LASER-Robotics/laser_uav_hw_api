"""Stream the existing livox_simulation FAST-LIO output to PX4's EKF2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav1')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='laser_uav_hw_api',
            executable='fast_lio_px4_bridge',
            namespace=LaunchConfiguration('uav_name'),
            output='screen',
            parameters=[{
                'uav_name': LaunchConfiguration('uav_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]),
    ])
