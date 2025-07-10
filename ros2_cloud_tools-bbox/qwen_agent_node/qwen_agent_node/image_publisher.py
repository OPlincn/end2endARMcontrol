"""
python3 image_publisher.py test.jpg

# 指定话题名称
python3 image_publisher.py /app/ros2_dev_ws/output_cropped2.jpg -t /camera/camera/color/image_raw

# 设置发布频率（Hz）
python3 image_publisher.py test.jpg -r 10

# 只发布一次
python3 image_publisher.py test.jpg -o

# 调整图片大小
python3 image_publisher.py test.jpg --resize 640x480
命令行参数

image_path: 要发布的图片路径（必需）
-t, --topic: 发布话题名称（默认: /publish_image）
-r, --rate: 发布频率 Hz（默认: 1.0）
-o, --once: 只发布一次后退出
--resize: 调整图片大小，格式: WIDTHxHEIGHT
"""
#!/usr/bin/env python3
"""
ROS2 图片发布节点
读取本地图片文件并发布到指定话题
"""

import os
import sys
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path


class ImagePublisher(Node):
    def __init__(self, image_path, topic_name='/publish_image', rate=1.0, loop=True):
        super().__init__('image_publisher_node')
        
        # 参数
        self.image_path = image_path
        self.topic_name = topic_name
        self.rate = rate
        self.loop = loop
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 创建发布器
        self.publisher = self.create_publisher(
            Image,
            self.topic_name,
            10  # QoS profile depth
        )
        
        # 读取图片
        self.cv_image = self.load_image()
        
        if self.cv_image is None:
            self.get_logger().error(f'无法读取图片: {self.image_path}')
            sys.exit(1)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # 发布计数
        self.publish_count = 0
        
        self.get_logger().info(f'图片发布节点已启动')
        self.get_logger().info(f'图片路径: {self.image_path}')
        self.get_logger().info(f'图片尺寸: {self.cv_image.shape}')
        self.get_logger().info(f'发布话题: {self.topic_name}')
        self.get_logger().info(f'发布频率: {self.rate} Hz')
        self.get_logger().info(f'循环模式: {"开启" if self.loop else "关闭"}')
    
    def load_image(self):
        """加载图片文件"""
        try:
            # 检查文件是否存在
            if not os.path.exists(self.image_path):
                self.get_logger().error(f'文件不存在: {self.image_path}')
                return None
            
            # 读取图片
            image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().error(f'无法解析图片文件: {self.image_path}')
                return None
            
            # OpenCV 默认是 BGR，确保格式正确
            return image
            
        except Exception as e:
            self.get_logger().error(f'读取图片时发生错误: {str(e)}')
            return None
    
    def timer_callback(self):
        """定时器回调，发布图片"""
        try:
            # 将 OpenCV 图片转换为 ROS Image 消息
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            
            # 设置时间戳和帧ID
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # 发布消息
            self.publisher.publish(msg)
            self.publish_count += 1
            
            # 日志输出（每10次输出一次，避免刷屏）
            if self.publish_count % 10 == 1:
                self.get_logger().info(f'已发布 {self.publish_count} 张图片')
            
            # 如果不循环，发布一次后退出
            if not self.loop and self.publish_count >= 1:
                self.get_logger().info('单次发布完成，退出节点')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'发布图片时发生错误: {str(e)}')


def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='ROS2 图片发布节点')
    parser.add_argument('image_path', type=str, help='要发布的图片路径')
    parser.add_argument('-t', '--topic', type=str, default='/publish_image',
                        help='发布的话题名称 (默认: /publish_image)')
    parser.add_argument('-r', '--rate', type=float, default=1.0,
                        help='发布频率 Hz (默认: 1.0)')
    parser.add_argument('-o', '--once', action='store_true',
                        help='只发布一次后退出 (默认: 循环发布)')
    parser.add_argument('--resize', type=str, default=None,
                        help='调整图片大小，格式: WIDTHxHEIGHT (例如: 640x480)')
    
    # ROS2 参数和自定义参数分离
    ros_args = []
    custom_args = []
    
    if args is None:
        args = sys.argv[1:]
    
    # 查找 ROS 参数分隔符
    if '--' in args:
        idx = args.index('--')
        custom_args = args[:idx]
        ros_args = args[idx+1:]
    else:
        custom_args = args
    
    # 解析自定义参数
    parsed_args = parser.parse_args(custom_args)
    
    # 检查图片文件
    image_path = Path(parsed_args.image_path).resolve()
    if not image_path.exists():
        print(f"错误: 图片文件不存在: {image_path}")
        return
    
    # 初始化 ROS2
    rclpy.init(args=ros_args)
    
    # 创建节点
    try:
        node = ImagePublisher(
            image_path=str(image_path),
            topic_name=parsed_args.topic,
            rate=parsed_args.rate,
            loop=not parsed_args.once
        )
        
        # 如果指定了 resize 参数，调整图片大小
        if parsed_args.resize:
            try:
                width, height = map(int, parsed_args.resize.split('x'))
                node.cv_image = cv2.resize(node.cv_image, (width, height))
                node.get_logger().info(f'图片已调整为: {width}x{height}')
            except Exception as e:
                node.get_logger().warn(f'无法调整图片大小: {str(e)}')
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误: {str(e)}")
    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()