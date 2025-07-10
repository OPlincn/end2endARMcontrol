# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    image_width_launch_arg = DeclareLaunchArgument(
        "sam_image_width", default_value=TextSubstitution(text="640")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "sam_image_height", default_value=TextSubstitution(text="640")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "sam_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_sam")
    )    
    is_regular_box_launch_arg = DeclareLaunchArgument(
        "sam_is_regular_box", default_value=TextSubstitution(text="0")
    )

    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)
    
    cam_node = None
    camera_type_mipi = None

    
    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format' : 'rgb8',
            'codec_sub_topic': '/camera/camera/color/image_raw',
            'codec_pub_topic': '/image',
            
        }.items()
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            # 'websocket_only_show_image': 'True',
            'websocket_smart_topic': LaunchConfiguration("sam_msg_pub_topic_name")
        }.items()
    )

    # 算法pkg
    sam_node = Node(
        package='mono_mobilesam',
        executable='mono_mobilesam',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_regular_box": LaunchConfiguration(
                "sam_is_regular_box")},
            {"is_shared_mem_sub": 0},
            {"msg_pub_topic_name": LaunchConfiguration(
                "sam_msg_pub_topic_name")},
            {"ros_img_sub_topic_name" : "/camera/camera/color/image_raw"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        image_width_launch_arg,
        image_height_launch_arg,
        msg_pub_topic_name_launch_arg,
        is_regular_box_launch_arg,
        # 图片编解码&发布pkg
        jpeg_codec_node,
        # 启动yoloworld pkg
        sam_node,
        # 启动web展示pkg
        web_node
    ])
