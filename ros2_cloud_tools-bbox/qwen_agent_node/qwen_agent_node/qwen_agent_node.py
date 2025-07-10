import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
# 导入 QwenAgent 和 MyTools
from .QwenLangchain import QwenAgent
from .MyTools import set_bbox_publisher, set_arm_action_publisher, set_camera_image # 添加 set_bbox_publisher
from ai_msgs.msg import PerceptionTargets     # <─ ai_msgs 接口包
from std_msgs.msg import Int32
class QwenAgentNode(Node):
    """
    ROS2 节点：订阅 /asr_text （语音识别文本），调用 QwenAgent.chat 获取回复，
    并发布到 /tts_text；同时订阅 /publish_image_source （图像数据），
    将其转换为 OpenCV 图像传给 MyTools。
    新增功能：发布物体检测结果到 /detect_bbox
    """
    def __init__(self):
        super().__init__('qwen_agent_node')

        # 初始化 QwenAgent（开启对话记忆）
        self.agent = QwenAgent(use_memory=True)
        self.agent.chat(rf"接下来与我对话的过程中,你可能会使用到与camera有关的工具。你的回答应该尽量控制在50字内,情感丰富,表现自然!同时你的回答中不应该包括'你好'这两个字")
        
        # CvBridge 用于 Image<->CV2 转换
        self.bridge = CvBridge()
        
        # Declare parameters with default values
        self.declare_parameter('asr_topic', '/asr_text')  # Default ASR topic name
        self.declare_parameter('tts_topic', '/tts_text')  # Default TTS topic name
        self.declare_parameter('image_topic', '/publish_image_source')  # Default image topic name
        # 👉 新：检测话题
        self.declare_parameter('perception_topic',   '/hobot_dnn_detection')
        # 👉 新：机械臂动作话题
        self.declare_parameter('arm_action_topic',   '/arm_action_command')
        self.declare_parameter('use_compressed', True) 
        
        # Retrieve parameter values
        asr_topic = self.get_parameter('asr_topic').get_parameter_value().string_value
        tts_topic = self.get_parameter('tts_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        perception_topic   = self.get_parameter('perception_topic').get_parameter_value().string_value
        arm_action_topic   = self.get_parameter('arm_action_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').value 
        
        # 4️⃣ 日志打印
        self.get_logger().info(
            f'配置:\n'
            f'  ASR   -> {asr_topic}\n'
            f'  TTS   -> {tts_topic}\n'
            f'  Image -> {image_topic}   (Compressed: {self.use_compressed})\n'
            f'  Detect-> {perception_topic}\n'
            f'  Arm   -> {arm_action_topic}'
        )
        
        # 订阅 ASR 文本话题
        self.asr_sub = self.create_subscription(
            String,
            asr_topic,
            self.asr_callback,
            10
        )
        
        # 订阅 相机图像话题    
        if self.use_compressed:
            self.get_logger().info('使用 CompressedImage 话题订阅 JPEG 数据')
            self.create_subscription(
                CompressedImage,
                image_topic,
                self.compressed_image_callback,
                10
            )
        else:
            self.get_logger().info('使用 Image 话题订阅原始像素数据')
            self.create_subscription(
                Image,
                image_topic,
                self.image_callback,
                10
            )
        
        # 发布 TTS 文本话题
        self.tts_pub = self.create_publisher(String, tts_topic, 10)
        
         ## 5.3 发布检测结果（ai_msgs/PerceptionTargets）
        self.perception_pub = self.create_publisher(
            PerceptionTargets, perception_topic, 10)
        set_bbox_publisher(self.perception_pub)          # 给 MyTools 用

        ## 5.4 发布机械臂动作（std_msgs/Int32）
        self.arm_action_pub = self.create_publisher(
            Int32, arm_action_topic, 10)
        set_arm_action_publisher(self.arm_action_pub)    # 给 MyTools 用

        self.get_logger().info('QwenAgentNode 已启动，等待输入...')

    def asr_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"[ASR] 收到: {text}")
        
        # 调用 QwenAgent 进行对话
        try:
            reply = self.agent.chat(text)
        except Exception as e:
            self.get_logger().error(f"调用 QwenAgent 出错: {e}")
            reply = "对不起，内部出错。"
        
        # 发布回复
        out_msg = String()
        out_msg.data = reply
        self.tts_pub.publish(out_msg)
        self.get_logger().info(f"[TTS] 发布: {reply}")

    def image_callback(self, msg: Image):
        """
        收到 Image 消息后，转换为 OpenCV 图像并存储到 MyTools.last_image。
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            set_camera_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
    
    def compressed_image_callback(self, msg: CompressedImage):
        """处理 sensor_msgs/CompressedImage（JPEG 格式）消息"""
        try:
            # ROS2 CompressedImage.data 在 Python 中是 List[int]
            np_arr = np.array(msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                raise RuntimeError("JPEG 解码失败")
            set_camera_image(cv_img)
        except Exception as e:
            self.get_logger().error(f"CompressedImage 回调失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QwenAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('QwenAgentNode 正在关闭...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()