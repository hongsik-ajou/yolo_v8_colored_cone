from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov8_cam',
            executable='webcam_publisher',
            name='webcam_publisher',
            output='screen'
        ),
        Node(
            package='yolov8_inference',
            executable='yolo_node',
            name='yolo_detector',
            output='screen'
        )
    ])
