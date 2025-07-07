import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rapling_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "False"}.items()
    )
    
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rapling_moveit_2"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "False"}.items()
    )
    
    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rapling_remote"),
                "launch",
                "remote_interface.launch.py"
            )
        )
    )
    
  
    vision_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("learning_topic"),
                "launch",
                "vision_launch.py"
            )
        )
    )

    return LaunchDescription([
        controller,
        moveit,
        remote_interface,
        vision_system
    ])