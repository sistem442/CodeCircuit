from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{'image_size': [640, 480]}],
            remappings=[('image_raw', '/image_raw')]
        ),

        Node(
            package='yolo_object_detection',
            executable='approach_with_yolo',
            name='approach_with_yolo',
            parameters=[{'min_distance': 0.10}],
            remappings=[('/image_raw', '/image_raw')]
        ),

        Node(
           package='yolo_object_detection',
           executable='ultraschall_node',
           name='ultraschall_node',
           output='screen'
       ),
    ])

