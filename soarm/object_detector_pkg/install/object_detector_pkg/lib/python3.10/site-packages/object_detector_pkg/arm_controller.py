# code by LinCC111 Boxjod 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有
# ROS2 Node adaptation for LeRobot SO100 ARM Control

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
import os
import numpy as np
import time
import math
import sys
import yaml
sys.path.append('/root/soarm/lerobot-kinematics-main')
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, feetech_arm
# For Feetech Motors
from lerobot_kinematics.lerobot.feetech import FeetechMotorsBus
import json

np.set_printoptions(linewidth=200)

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        
        # Set up the MuJoCo render backend
        os.environ["MUJOCO_GL"] = "egl"
        
        # Define joint names
        self.JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
        
        self.robot = get_robot('so100')
        
        # Define joint control increment (in radians)
        self.JOINT_INCREMENT = 0.005
        self.POSITION_INSERMENT = 0.0008
        
        # Define joint limits
        self.control_qlimit = [[-1.5, -3.1, -0.0, -1.375,  -1.57, -0.15], 
                              [ 1.5,  0.0,  3.1,  1.475,   3.1,  1.5]]
        self.control_glimit = [[0.125, -0.4,  0.046, -3.1, -0.75, -1.5], 
                              [0.340,  0.4,  0.23, 2.0,  1.57,  1.5]]
        
        # Initialize target joint positions
        self.init_qpos = np.array([0.0, -3.14, 3.14, 0.0, -1.57, 1.5])
        self.target_qpos = self.init_qpos.copy()
        self.init_gpos = lerobot_FK(self.init_qpos[1:5], robot=self.robot)
        self.get_logger().info(f"Initial gpos: {self.init_gpos}")
        
        self.target_qpos_last = self.init_qpos.copy()
        
        # Connect to the robotic arm motors
        self.motors = {"shoulder_pan": (1, "sts3215"),
                      "shoulder_lift": (2, "sts3215"),
                      "elbow_flex": (3, "sts3215"),
                      "wrist_flex": (4, "sts3215"),
                      "wrist_roll": (5, "sts3215"),
                      "gripper": (6, "sts3215")}
        
        try:
            self.follower_arm = feetech_arm(driver_port="/dev/ttyACM0", 
                                          calibration_file="/root/soarm/lerobot-kinematics-main/examples/main_follower.json")
            self.get_logger().info("机械臂连接成功")
        except Exception as e:
            self.get_logger().error(f"机械臂连接失败: {e}")
            self.follower_arm = None
        
        # 加载动作配置文件
        self.load_action_config()
        
        # Set initial target position
        self.home_x = self.config['actions']['grab_home']['position'][0]
        self.home_y = self.config['actions']['grab_home']['position'][1]
        self.home_z = self.config['actions']['grab_home']['position'][2]
        self.target_gpos = self.config['actions']['grab_home']['position'].copy()
        self.target_gpos_last = self.target_gpos.copy()
        self.target_y = self.home_y  # Initialize target yaw angle
        self.target_x = self.home_x
        self.target_z = self.home_z
        
        # 抓取放置位置
        self.drop_x = self.config['drop_position']['x']
        self.drop_y = self.config['drop_position']['y']
        self.drop_z = self.config['drop_position']['z']
        
        # 抓取状态机
        self.pick_state = "IDLE"  # IDLE, APPROACH, DESCEND, GRAB, LIFT, PLACE_APPROACH, PLACE_DOWN, PLACE_LIFT, RETURN
        self.state_timer = 0
        self.state_start_time = 0
        self.gripper_closed = False
        
        # 动作执行状态
        self.current_action = 0  # 当前动作编号
        self.action_state = "IDLE"  # IDLE, EXECUTING, COMPLETED
        self.action_sequence_index = 0  # 动作序列索引
        self.action_repeat_count = 0  # 动作重复次数
        self.action_start_time = 0
        
        # Subscribe to target point
        self.target_point_sub = self.create_subscription(
            PointStamped,
            'target_point_in_camera',
            self.target_point_callback,
            10)
        
        # Subscribe to action command
        self.action_sub = self.create_subscription(
            Int32,
            'arm_action_command',
            self.action_command_callback,
            10)
        
        # Create a timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz control loop
        
        # Control state
        self.has_target = True
        self.t = 0
        self.object_x = 0.0
        self.object_y = 0.0
        self.object_z = 0.0
        
        # 平滑角度控制
        self.current_base_angle = self.follower_arm.feedback()[0]
        self.target_base_angle = 0.0   # 目标基座角度
        self.angle_speed = self.config['motion_params']['angle_speed']  # 角度变化速度
        
        self.get_logger().info('机械臂控制节点已启动，等待目标点和动作指令...')
        print(self.target_gpos)
    def load_action_config(self):
        """加载动作配置文件"""
        try:
            config_path = '/root/soarm/object_detector_pkg/config/arm_actions.yaml'
            with open(config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            self.get_logger().info("动作配置文件加载成功")
        except Exception as e:
            self.get_logger().error(f"加载配置文件失败: {e}")
            # 默认配置
            self.config = {
                'actions': {
                    'grab_home': {'position': [0.20, 0.0, 0.13, -1.57, 1.4, 0], 'gripper_closed': False}
                },
                'drop_position': {'x': 0.05, 'y': -0.15, 'z': 0.13},
                'motion_params': {'angle_speed': 0.015}
            }

    def action_command_callback(self, msg):
        """处理动作指令"""
        action_id = msg.data
        action_names = {
            0: "抓取模式",
            1: "跳舞",
            2: "左看",
            3: "右看",
            4: "握手",
            5: "向前看"
        }
        
        # 只有在空闲状态才接受新的动作指令
        if self.pick_state != "IDLE" and self.action_state != "IDLE":
            self.get_logger().info(f"机械臂正忙，忽略动作指令: {action_names.get(action_id, '未知动作')}")
            return
        
        self.current_action = action_id
        self.get_logger().info(f"接收到动作指令: {action_names.get(action_id, '未知动作')}")
        
        if action_id == 0:
            # 抓取模式，返回home位置
            self.pick_state = "IDLE"
            self.action_state = "IDLE"
        else:
            # 执行其他动作
            self.action_state = "EXECUTING"
            self.action_sequence_index = 0
            self.action_repeat_count = 0
            self.action_start_time = time.time()
            self.state_timer = 0

    def target_point_callback(self, msg):
        """处理接收到的目标点坐标"""
        try:
            # 只有在抓取模式且空闲状态才接受新的目标
            if self.current_action != 0:
                self.get_logger().info("当前不在抓取模式，忽略目标点")
                return
                
            if self.pick_state != "IDLE":
                self.get_logger().info("机械臂正在执行任务，忽略新目标")
                return
                
            # 从相机坐标系转换到机械臂坐标系
            self.object_x = -msg.point.y + 0.2  # 相机Z轴 -> 机械臂X轴（前进）
            self.object_y = -msg.point.x +0.04  # 相机X轴 -> 机械臂Y轴（左右），注意符号
            self.object_z = 0.07  # 抓取高度，稍微低一些
            
            self.get_logger().info(f"收到目标点: 相机坐标({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})")
            self.get_logger().info(f"开始抓取任务: 目标位置 X={self.object_x:.3f}, Y={self.object_y:.3f}, Z={self.object_z:.3f}")
            
            # 开始抓取流程
            self.pick_state = "APPROACH"
            self.state_start_time = time.time()
            self.state_timer = 0
            
        except Exception as e:
            self.get_logger().error(f"处理目标点时发生错误: {e}")

    def control_loop(self):
        """控制循环"""
        if not self.has_target or self.follower_arm is None:
            return
            
        try:
            self.t += 1
            self.state_timer += 1
            
            # 根据当前模式执行相应逻辑
            if self.current_action == 0:
                # 抓取模式
                self.execute_pick_and_place()
            else:
                # 动作模式
                self.execute_action_sequence()
            
            # 更新目标位置
            self.target_gpos[0] = math.sqrt(self.target_x**2 + self.target_y**2)
            self.target_gpos[2] = self.target_z
            
            # 获取当前关节位置反馈
            fd_qpos = self.follower_arm.feedback()[1:5]
            
            # 计算目标基座角度
            if self.target_gpos[0] > 0:
                self.target_base_angle = math.asin(self.target_y / self.target_gpos[0])
            else:
                self.target_base_angle = 0.0
            # print(self.target_gpos)
            # 平滑角度过渡
            self.smooth_base_angle_transition()
            
            # 逆运动学求解
            qpos_inv, IK_success = lerobot_IK(fd_qpos, self.target_gpos, robot=self.robot)
            
            if np.all(qpos_inv != -1.0):  # 检查IK解是否有效
                # 设置夹爪状态
                gripper_pos = 0.12 if self.gripper_closed else 1.5
                # 使用平滑过渡后的基座角度
                self.target_qpos = np.concatenate((np.array([self.current_base_angle]), qpos_inv[:4], [gripper_pos]))
                
                # 关节限制检查
                for i in range(len(self.target_qpos)):
                    if self.target_qpos[i] < self.control_qlimit[0][i]:
                        self.target_qpos[i] = self.control_qlimit[0][i]
                    elif self.target_qpos[i] > self.control_qlimit[1][i]:
                        self.target_qpos[i] = self.control_qlimit[1][i]
                
                # 发送控制命令到机械臂
                self.follower_arm.action(self.target_qpos)
                self.target_gpos_last = self.target_gpos.copy()
                
                if self.t % 100 == 0:  # 每5秒打印一次状态
                    self.get_logger().info(f"状态: {self.pick_state}, 目标位置: X={self.target_x:.3f}, Y={self.target_y:.3f}, Z={self.target_z:.3f}")
                    
            else:
                self.target_gpos = self.target_gpos_last.copy()
                if self.t % 100 == 0:
                    self.get_logger().warn("IK求解失败，保持上一个有效位置")
                    
        except Exception as e:
            self.get_logger().error(f"控制循环中发生错误: {e}")

    def smooth_base_angle_transition(self):
        """平滑基座角度过渡"""
        # 计算角度差
        angle_diff = self.target_base_angle - self.current_base_angle
        
        # 处理角度绕圈问题（-π 到 π）
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 限制每次的角度变化量
        if abs(angle_diff) > self.angle_speed:
            if angle_diff > 0:
                self.current_base_angle += self.angle_speed
            else:
                self.current_base_angle -= self.angle_speed
        else:
            self.current_base_angle = self.target_base_angle
        
        # 确保角度在合理范围内
        if self.current_base_angle > math.pi:
            self.current_base_angle -= 2 * math.pi
        elif self.current_base_angle < -math.pi:
            self.current_base_angle += 2 * math.pi

    def execute_action_sequence(self):
        """执行动作序列"""
        action_map = {
            1: 'dance',
            2: 'look_left', 
            3: 'look_right',
            4: 'handshake',
            5: 'look_forward'
        }
        
        action_key = action_map.get(self.current_action)
        if not action_key or action_key not in self.config['actions']:
            self.get_logger().error(f"未知动作: {self.current_action}")
            self.action_state = "IDLE"
            self.current_action = 0
            return
            
        action_config = self.config['actions'][action_key]
        
        # 检查是否为序列动作
        if 'sequence' in action_config:
            self.execute_sequence_action(action_config)
        else:
            self.execute_single_action(action_config)

    def execute_single_action(self, action_config):
        """执行单个动作"""
        self.target_x = action_config['position'][0]
        self.target_y = action_config['position'][1] 
        self.target_z = action_config['position'][2]
        self.target_gpos[3] = action_config['position'][3]
        self.target_gpos[4] = action_config['position'][4]
        self.target_gpos[5] = action_config['position'][5]
        self.gripper_closed = action_config['gripper_closed']
        # 检查动作完成
        duration_ms = int(action_config['duration'] * 50)  # 转换为控制周期数
        # if self.state_timer > duration_ms:
        #     self.get_logger().info(f"动作 {self.current_action} 完成，返回home位置")
        #     self.action_state = "IDLE"
        #     self.current_action = 0
        #     self.state_timer = 0

    def execute_sequence_action(self, action_config):
        """执行序列动作"""
        sequence = action_config['sequence']
        max_repeats = action_config.get('repeat', 1)
        
        if self.action_sequence_index >= len(sequence):
            # 当前序列完成，检查是否需要重复
            self.action_repeat_count += 1
            if self.action_repeat_count >= max_repeats:
                # 所有重复完成
                self.get_logger().info(f"序列动作 {self.current_action} 完成，返回home位置")
                self.action_state = "IDLE"
                self.current_action = 0
                self.action_sequence_index = 0
                self.action_repeat_count = 0
                self.state_timer = 0
                return
            else:
                # 开始下一轮重复
                self.action_sequence_index = 0
                self.state_timer = 0
                
        # 执行当前序列步骤
        current_step = sequence[self.action_sequence_index]
        self.target_x = current_step['position'][0]
        self.target_y = current_step['position'][1]
        self.target_z = current_step['position'][2] 
        self.target_gpos[3] = current_step['position'][3]
        self.target_gpos[4] = current_step['position'][4]
        self.target_gpos[5] = current_step['position'][5]
        self.gripper_closed = current_step['gripper_closed']
        
        # 检查当前步骤完成
        duration_ms = int(current_step['duration'] * 50)  # 转换为控制周期数
        if self.state_timer > duration_ms:
            self.action_sequence_index += 1
            self.state_timer = 0

    def execute_pick_and_place(self):
        """执行抓取和放置流程的状态机"""
        current_time = time.time()
        
        if self.pick_state == "IDLE":
            # 空闲状态，保持在home位置
            self.target_x = self.home_x
            self.target_y = self.home_y
            self.target_z = self.home_z
            self.target_gpos[4] = 1.4
            self.gripper_closed = False
            
        elif self.pick_state == "APPROACH":
            # 1. 接近目标物体上方
            self.target_x = self.object_x
            self.target_y = self.object_y
            self.target_z = 0.13
            self.gripper_closed = False
            
            # 判断是否接近目标
            if self.is_position_reached(tolerance=self.config['motion_params'].get('position_tolerance', 0.02)) and self.state_timer > 100:  # 2秒
                self.pick_state = "DESCEND"
                self.state_timer = 0
                self.get_logger().info("开始下降到抓取位置")
                
        elif self.pick_state == "DESCEND":
            # 2. 下降到抓取位置
            self.target_x = self.object_x
            self.target_y = self.object_y
            self.target_z = self.object_z  # 下降到抓取高度
            
            self.gripper_closed = False
            
            if self.is_position_reached(tolerance=self.config['motion_params'].get('grab_tolerance', 0.01)) and self.state_timer > 50:  # 1秒
                self.pick_state = "GRAB"
                self.state_timer = 0
                self.get_logger().info("开始抓取")
                
        elif self.pick_state == "GRAB":
            # 3. 抓取物体
            self.gripper_closed = True
            
            if self.state_timer > 100:  # 2秒抓取时间
                self.pick_state = "LIFT"
                self.state_timer = 0
                self.get_logger().info("抓取完成，开始抬起")
                
        elif self.pick_state == "LIFT":
            # 4. 抬起物体
            self.target_x = self.object_x
            self.target_y = self.object_y
            self.target_z = 0.13  # 抬高10cm
            self.gripper_closed = True
            
            if self.is_position_reached(tolerance=0.02) and self.state_timer > 50:  # 1秒
                self.pick_state = "PLACE_APPROACH"
                self.state_timer = 0
                self.get_logger().info("开始移动到放置位置")
                
        elif self.pick_state == "PLACE_APPROACH":
            # 5. 移动到放置位置上方
            self.target_x = self.drop_x
            self.target_y = self.drop_y
            self.target_z = self.drop_z 
            self.target_gpos[4] = -0.2
            self.gripper_closed = True
            
            if self.is_position_reached(tolerance=0.02) and self.state_timer > 100:  # 2秒
                self.pick_state = "PLACE_DOWN"
                self.state_timer = 0
                self.get_logger().info("开始下降到放置位置")
                
        elif self.pick_state == "PLACE_DOWN":
            # 6. 下降到放置位置
            self.target_x = self.drop_x
            self.target_y = self.drop_y
            self.target_z = self.drop_z
            self.target_gpos[4] = -0.2
            self.gripper_closed = True
            
            if self.is_position_reached(tolerance=0.01) and self.state_timer > 50:  # 1秒
                self.pick_state = "PLACE_LIFT"
                self.state_timer = 0
                self.get_logger().info("开始松开物体")
                
        elif self.pick_state == "PLACE_LIFT":
            # 7. 松开物体并抬起
            self.gripper_closed = False
            
            if self.state_timer > 100:  # 2秒松开时间
                self.target_z = self.drop_z  
                if self.state_timer > 150:  # 再等1秒
                    self.pick_state = "RETURN"
                    self.state_timer = 0
                    self.get_logger().info("物体放置完成，返回home位置")
                    
        elif self.pick_state == "RETURN":
            # 8. 返回home位置
            self.target_x = self.home_x
            self.target_y = self.home_y
            self.target_z = self.home_z
            self.gripper_closed = False
            
            if self.is_position_reached(tolerance=0.02) and self.state_timer > 100:  # 2秒
                self.pick_state = "IDLE"
                self.state_timer = 0
                self.get_logger().info("返回home位置完成，准备接收新任务")

    def is_position_reached(self, tolerance=0.02):
        """检查是否到达目标位置"""
        try:
            current_gpos = lerobot_FK(self.follower_arm.feedback()[1:5], robot=self.robot)
            distance = math.sqrt(
                (current_gpos[0] - self.target_gpos[0])**2 +  
                (current_gpos[2] - self.target_gpos[2])**2
            )
            return distance < tolerance
        except:
            return False

    def destroy_node(self):
        """节点销毁时断开机械臂连接"""
        try:
            if self.follower_arm is not None:
                self.follower_arm.disconnect()
                self.get_logger().info("机械臂连接已断开")
        except Exception as e:
            self.get_logger().error(f"断开机械臂连接时发生错误: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断程序")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
