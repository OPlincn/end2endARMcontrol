import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_action_test',
            default_value='false',
            description='是否启动动作测试节点'
        ),
        
        # 目标检测节点
        Node(
            package='object_detector_pkg',
            executable='object_detector',
            name='object_detector_node',
            output='screen',
            parameters=[]
        ),
        
        # 机械臂控制节点
        Node(
            package='object_detector_pkg',
            executable='arm_controller',
            name='arm_controller_node',
            output='screen',
            parameters=[]
        ),
        
        # 动作测试节点（可选）
        Node(
            package='object_detector_pkg',
            executable='test_arm_actions',
            name='action_test_node',
            output='screen',
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('use_action_test')
            )
        )
    ])
