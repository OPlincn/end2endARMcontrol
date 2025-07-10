import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PointStamped

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None
        self.has_printed_depth = False
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)
            
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)

        self.sam_sub = self.create_subscription(
            PerceptionTargets,
            '/hobot_sam',
            self.sam_callback,
            10)

        self.target_point_pub = self.create_publisher(PointStamped, 'target_point_in_camera', 10)

        self.camera_to_robot_matrix = np.array([
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.get_logger().info('节点已启动，等待相机信息和SAM目标...')

    def camera_info_callback(self, msg):
        if not self.camera_info:
            self.camera_info = msg
            self.get_logger().info('已接收到相机内参.')
            self.destroy_subscription(self.camera_info_sub)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
         # --- 添加深度图打印逻辑 ---
        
    def process_mask_and_get_depth(self, mask_bool, roi_rect):
        """
        根据布尔掩码，可视化并计算平均深度。
        :param mask_bool: 一个布尔类型的numpy数组，True代表目标区域。
        :param roi_rect: ROI的矩形框，用于确定打印范围。
        :return: 掩码区域的平均深度（米）。
        """
        if self.depth_image is None:
            self.get_logger().warn("无法处理掩码，深度图不可用。")
            return 0.0

        # 1. 可视化掩码区域
        self.get_logger().info("--- 打印SAM分割掩码的可视化 ---")
        x, y, w, h = roi_rect.x_offset, roi_rect.y_offset, roi_rect.width, roi_rect.height

        # 2. 提取并计算深度
        # 从深度图中提取掩码对应的深度值
        masked_depth_values = self.depth_image[mask_bool]
        
        # 过滤掉无效的深度值（通常为0）
        valid_depth_values = masked_depth_values[masked_depth_values > 0]
        self.get_logger().info(f"有效深度值数量：{valid_depth_values.size}")
        if valid_depth_values.size == 0:
            self.get_logger().warn("掩码区域内没有有效的深度值。")
            return 0.0
        
        # 计算平均深度（从毫米转换为米）
        average_depth_mm = np.mean(valid_depth_values)
        average_depth_m = average_depth_mm / 1000.0
        
        self.get_logger().info(f"掩码区域内有效像素点数量: {valid_depth_values.size}")
        self.get_logger().info(f"计算出的平均深度为: {average_depth_m:.3f} 米")
        
        return average_depth_m

    def sam_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            self.get_logger().warn('尚未接收到相机内参或深度图像，跳过处理.')
            return

        # 初始化变量，用于存储从消息中解析出的数据
        roi_rect = None
        mask_capture = None

        # 1. 遍历所有目标，寻找ROI和掩码
        for target in msg.targets:
            # 查找包含有效ROI（矩形框）的目标
            if not roi_rect and target.rois:
                roi_rect = target.rois[0].rect
                self.get_logger().info(f"找到ROI矩形框: x={roi_rect.x_offset}, y={roi_rect.y_offset}, w={roi_rect.width}, h={roi_rect.height}")

            # 查找包含掩码（captures）的目标
            if not mask_capture and target.captures:
                mask_capture = target.captures[0]
                self.get_logger().info("找到掩码数据 (captures)。")

        # 2. 检查是否找到了必要的信息
        if not roi_rect:
            self.get_logger().warn("在消息中未找到有效的ROI矩形框，无法处理。")
            return
        
        if not mask_capture or not mask_capture.features:
            self.get_logger().warn("在消息中未找到有效的掩码特征数据，无法处理。")
            return

        try:
            # 3. 从capture中提取掩码并重塑
            mask_h = mask_capture.img.height
            mask_w = mask_capture.img.width
            
            if mask_h * mask_w != len(mask_capture.features):
                self.get_logger().error(f"掩码尺寸 ({mask_h}x{mask_w}={mask_h*mask_w}) 与特征数量 ({len(mask_capture.features)}) 不匹配!")
                return

            # 将一维列表转换为二维numpy数组
            mask_1d = np.array(mask_capture.features, dtype=np.float32)
            # 将其重塑为二维掩码图像
            reshaped_mask = mask_1d.reshape((mask_h, mask_w))
            # 将浮点掩码转换为布尔掩码（通常大于0的即为掩码部分）
            bool_mask = reshaped_mask > 0

            # 4. 创建一个与深度图同样大小的完整掩码
            full_mask = np.zeros(self.depth_image.shape[:2], dtype=bool)
            
            # 将重塑后的掩码粘贴到由ROI定义的位置
            x, y = roi_rect.x_offset, roi_rect.y_offset
            # 确保粘贴时不会超出边界
            end_y, end_x = min(y + mask_h, full_mask.shape[0]), min(x + mask_w, full_mask.shape[1])
            paste_h, paste_w = end_y - y, end_x - x
            
            full_mask[y:end_y, x:end_x] = bool_mask[:paste_h, :paste_w]

            # 5. 调用处理函数进行可视化和深度计算
            depth = self.process_mask_and_get_depth(full_mask, roi_rect)

            if depth <= 0:
                self.get_logger().warn(f'无法计算目标的3D坐标，因为深度无效。')
                return

            # 6. 使用ROI中心点和计算出的平均深度来获取3D坐标
            u = int(roi_rect.x_offset + roi_rect.width / 2)
            v = int(roi_rect.y_offset + roi_rect.height / 2)

            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            cam_x = (u - cx) * depth / fx
            cam_y = (v - cy) * depth / fy
            cam_z = depth
            
            self.get_logger().info(f'目标在相机坐标系下的坐标: X={cam_x:.3f}, Y={cam_y:.3f}, Z={cam_z:.3f}')
            if cam_z>=0.7:
                self.get_logger().info(f'目标在相机坐标系下的坐标no use')
                return
            # 7. 发布目标点的坐标
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = msg.header.frame_id
            point_msg.point.x = cam_x
            point_msg.point.y = cam_y
            point_msg.point.z = cam_z
            
            self.target_point_pub.publish(point_msg)
            self.get_logger().info(f"已发布目标坐标到 /target_point_in_camera 话题。")

        except Exception as e:
            self.get_logger().error(f"处理SAM回调时发生严重错误: {e}", exc_info=True)
    

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()