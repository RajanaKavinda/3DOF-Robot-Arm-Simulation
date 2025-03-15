import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )


    task_server_node = Node(
        package="arduinobot_remote",
        executable="task_server_node",
        parameters=[{'use_sim_time': is_sim}]              
    )

    alexa_interface_node = Node(
        package="arduinobot_remote",
        executable="alexa_interface.py",
        parameters=[{'use_sim_time': is_sim}]
    )


    return LaunchDescription(
        [
            is_sim_arg,
            task_server_node,
            alexa_interface_node
        ]
    )

