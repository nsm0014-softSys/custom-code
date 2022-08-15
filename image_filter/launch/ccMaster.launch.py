import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
######################################### IMAGE FILTER ########################################################

    # define node to launch and parameters to use
    image_filter_node = Node(
        package='image_filter',
        executable='image_filter_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

######################################### PID CTRL ########################################################
    # Define config file location
    pid_ctrl_config_file = get_share_file(
        package_name='pid_ctrl', file_name='config/pid_ctrl_config.yaml'
    )

    # tell ros we are using a config file
    pid_ctrl_config = DeclareLaunchArgument(
        'pid_ctrl_config_file',
        default_value=pid_ctrl_config_file,
        description='Path to config file for pid ctrl parameters'
    )

    # define node to launch and parameters to use
    pid_ctrl_node = Node(
        package='pid_ctrl',
        executable='pid_ctrl_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[LaunchConfiguration('pid_ctrl_config_file')],
    )

######################################### THROTTLE CTRL ########################################################
    # Define config file location
    throttle_ctrl_config_file = get_share_file(
        package_name='throttle_ctrl', file_name='config/throttle_ctrl_config.yaml'
    )

    # tell ros we are using a config file
    throttle_ctrl_config = DeclareLaunchArgument(
        'throttle_ctrl_config_file',
        default_value=throttle_ctrl_config_file,
        description='Path to config file for throttle ctrl parameters'
    )

    # define node to launch and parameters to use
    throttle_ctrl_node = Node(
        package='throttle_ctrl',
        executable='throttle_ctrl_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[LaunchConfiguration('throttle_ctrl_config_file')],
    )


    return LaunchDescription([
        image_filter_node,
        pid_ctrl_config,
        pid_ctrl_node,
        throttle_ctrl_config,
        throttle_ctrl_node
    ])


