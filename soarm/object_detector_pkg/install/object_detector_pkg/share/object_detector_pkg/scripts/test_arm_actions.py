#!/usr/bin/env python3
"""
机械臂动作指令测试脚本
用于发送不同的动作指令到机械臂控制节点
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class ActionCommandPublisher(Node):
    def __init__(self):
        super().__init__('action_command_publisher')
        self.publisher = self.create_publisher(Int32, 'arm_action_command', 10)
        self.get_logger().info('动作指令发布节点已启动')

    def send_action(self, action_id):
        """发送动作指令"""
        msg = Int32()
        msg.data = action_id
        self.publisher.publish(msg)
        
        action_names = {
            0: "抓取模式",
            1: "跳舞",
            2: "左看", 
            3: "右看",
            4: "握手",
            5: "向前看"
        }
        
        self.get_logger().info(f'发送动作指令: {action_id} - {action_names.get(action_id, "未知动作")}')

def main():
    rclpy.init()
    node = ActionCommandPublisher()
    
    try:
        print("机械臂动作测试程序")
        print("动作列表:")
        print("0 - 抓取模式(默认)")
        print("1 - 跳舞")
        print("2 - 左看")
        print("3 - 右看") 
        print("4 - 握手")
        print("5 - 向前看")
        print("输入 'q' 退出")
        print("-" * 30)
        
        while rclpy.ok():
            try:
                user_input = input("请输入动作编号 (0-5) 或 'q' 退出: ").strip()
                
                if user_input.lower() == 'q':
                    break
                    
                action_id = int(user_input)
                if 0 <= action_id <= 5:
                    node.send_action(action_id)
                    time.sleep(0.1)  # 短暂延迟确保消息发送
                else:
                    print("无效的动作编号，请输入 0-5")
                    
            except ValueError:
                print("请输入有效的数字或 'q'")
            except KeyboardInterrupt:
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("程序已退出")

if __name__ == '__main__':
    main()
