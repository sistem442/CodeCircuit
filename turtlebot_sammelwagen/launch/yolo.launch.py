from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Kamera-Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen'
        ),

      Node(
           package='yolo_object_detection',
           executable='approach_with_yolo',
           name='approach_yolo_node',
           output='screen',
           parameters=[{'min_distance': 0.1}],
           remappings=[('/image_raw', '/camera/image_raw')]
      )


    ])

