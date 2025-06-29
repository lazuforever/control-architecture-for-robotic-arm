from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para topic_webcam_pub.py
        Node(
            package='learning_topic',
            executable='topic_webcam_pub',
            name='webcam_pub_node',
            output='screen'
        ),
        # Nodo para finger_detector.py
        Node(
            package='learning_topic',
            executable='finger_detector',
            name='finger_detector_node',
            output='screen'
        ),
    ])