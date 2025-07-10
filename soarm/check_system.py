#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
系统状态检查工具
检查所有组件的运行状态和系统健康度

作者：AI编程助手
日期：2024
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import sys
import os

class SystemChecker(Node):
    """系统状态检查器"""
    
    def __init__(self):
        super().__init__('system_checker')
        self.get_logger().info("系统状态检查器已启动")
    
    def check_ros2_nodes(self):
        """检查ROS2节点状态"""
        print("\n🔍 ROS2节点状态检查")
        print("-" * 40)
        
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                if nodes and nodes[0]:
                    print(f"✅ 发现 {len(nodes)} 个运行中的节点：")
                    for node in nodes:
                        print(f"   • {node}")
                    
                    # 检查关键节点
                    key_nodes = [
                        'realsense_camera',
                        'object_detector', 
                        'arm_controller',
                        'mobilesam_node'
                    ]
                    
                    print("\n🎯 关键节点状态：")
                    for key_node in key_nodes:
                        found = any(key_node in node for node in nodes)
                        status = "✅ 运行中" if found else "❌ 未运行"
                        print(f"   {key_node}: {status}")
                else:
                    print("❌ 未发现运行中的节点")
            else:
                print("❌ 无法获取节点列表")
                
        except subprocess.TimeoutExpired:
            print("❌ 命令超时")
        except Exception as e:
            print(f"❌ 错误: {e}")
    
    def check_topics(self):
        """检查话题状态"""
        print("\n📡 话题状态检查")
        print("-" * 40)
        
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print(f"✅ 发现 {len(topics)} 个活跃话题")
                
                # 检查关键话题
                key_topics = [
                    '/camera/camera/color/image_raw',
                    '/camera/camera/depth/image_rect_raw',
                    '/hobot_sam',
                    '/target_3d_position',
                    '/arm_action_command',
                    '/arm_status'
                ]
                
                print("\n🎯 关键话题状态：")
                for topic in key_topics:
                    found = topic in topics
                    status = "✅ 活跃" if found else "❌ 未找到"
                    print(f"   {topic}: {status}")
                    
            else:
                print("❌ 无法获取话题列表")
                
        except Exception as e:
            print(f"❌ 错误: {e}")
    
    def check_hardware(self):
        """检查硬件状态"""
        print("\n🖥️  硬件状态检查")
        print("-" * 40)
        
        # 检查RealSense设备
        try:
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            if 'Intel' in result.stdout:
                print("✅ RealSense设备已连接")
            else:
                print("❌ 未找到RealSense设备")
        except:
            print("❌ 无法检查USB设备")
        
        # 检查串口设备
        serial_devices = []
        for i in range(10):
            device = f"/dev/ttyUSB{i}"
            if os.path.exists(device):
                serial_devices.append(device)
        
        if serial_devices:
            print(f"✅ 发现串口设备: {', '.join(serial_devices)}")
        else:
            print("❌ 未找到串口设备 (机械臂可能无法连接)")
    
    def check_packages(self):
        """检查Python包状态"""
        print("\n📦 Python包状态检查")
        print("-" * 40)
        
        packages = [
            'lerobot_kinematics',
            'object_detector_pkg', 
            'spatialmath',
            'cv2',
            'numpy',
            'scipy',
            'yaml'
        ]
        
        for package in packages:
            try:
                if package == 'cv2':
                    import cv2
                elif package == 'yaml':
                    import yaml
                else:
                    exec(f"import {package}")
                print(f"✅ {package}: 已安装")
            except ImportError:
                print(f"❌ {package}: 未安装")
    
    def check_topic_data(self):
        """检查话题数据流"""
        print("\n📊 话题数据流检查")
        print("-" * 40)
        
        # 检查相机数据
        try:
            result = subprocess.run([
                'timeout', '3', 'ros2', 'topic', 'hz', 
                '/camera/camera/color/image_raw'
            ], capture_output=True, text=True)
            
            if 'average rate' in result.stdout:
                print("✅ 相机彩色图像数据正常")
            else:
                print("❌ 相机彩色图像无数据")
        except:
            print("❌ 无法检查相机数据")
        
        # 检查深度数据
        try:
            result = subprocess.run([
                'timeout', '3', 'ros2', 'topic', 'hz', 
                '/camera/camera/depth/image_rect_raw'
            ], capture_output=True, text=True)
            
            if 'average rate' in result.stdout:
                print("✅ 相机深度图像数据正常")
            else:
                print("❌ 相机深度图像无数据")
        except:
            print("❌ 无法检查深度数据")
    
    def generate_report(self):
        """生成完整报告"""
        print("="*60)
        print("🤖 完整ROS2视觉-机械臂系统状态报告")
        print("="*60)
        
        self.check_ros2_nodes()
        self.check_topics()
        self.check_hardware()
        self.check_packages()
        self.check_topic_data()
        
        print("\n" + "="*60)
        print("📋 建议操作：")
        print("   1. 如果相机未连接，请检查USB连接")
        print("   2. 如果串口设备未找到，请检查机械臂连接")
        print("   3. 如果包未安装，请运行 ./start_system.sh 选择安装依赖")
        print("   4. 如果节点未运行，请使用启动脚本启动系统")
        print("="*60)


def main():
    """主函数"""
    rclpy.init()
    
    try:
        checker = SystemChecker()
        checker.generate_report()
    except KeyboardInterrupt:
        print("\n👋 检查被用户中断")
    except Exception as e:
        print(f"❌ 检查出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
