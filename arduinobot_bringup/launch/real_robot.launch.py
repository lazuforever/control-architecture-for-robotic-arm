import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Include other launch files
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "False"}.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "False"}.items()
    )

    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py"
            )
        )
    )

    # Nodes for the learning_topic package
    webcam_pub_node = Node(
        package='learning_topic',
        executable='topic_webcam_pub',
        name='webcam_pub_node',
        output='screen'
    )

    finger_detector_node = Node(
        package='learning_topic',
        executable='finger_detector',
        name='finger_detector_node',
        output='screen'
    )

    return LaunchDescription([
        webcam_pub_node,          # Start the webcam publisher first
        finger_detector_node,     # Then start the finger detector
        controller,
        moveit,
        remote_interface
    ])