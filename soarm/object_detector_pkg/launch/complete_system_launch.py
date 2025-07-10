import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detector_pkg',
            executable='object_detector',
            name='object_detector_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='object_detector_pkg',
            executable='arm_controller',
            name='arm_controller_node',
            output='screen',
            parameters=[]
        )
    ])
