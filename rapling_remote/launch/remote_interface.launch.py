import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    
    task_server_node = Node(
        package="rapling_remote",
        executable="task_server_node"
    )
    alexa_interface_node = Node(
            package="rapling_remote",
            executable="alexa_interface.py"
      )
    
    return LaunchDescription([
        task_server_node,
        alexa_interface_node
    ])
