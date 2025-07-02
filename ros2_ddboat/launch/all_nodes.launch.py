from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_ddboat', executable='gps_node', name='gps_node'),
        Node(package='ros2_ddboat', executable='arduino_node', name='arduino_node'),
        Node(package='ros2_ddboat', executable='encoders_node', name='encoders_node'),
        Node(package='ros2_ddboat', executable='imu_node', name='imu_node'),
        Node(package='ros2_ddboat', executable='temp_node', name='temp_node'),
        Node(package='ros2_ddboat', executable='radio_node', name='radio_node'),
    ])
