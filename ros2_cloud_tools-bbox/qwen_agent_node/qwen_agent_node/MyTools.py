# MyTools.py - 带详细调试信息版本

import os
import json
from datetime import datetime
from dashscope import MultiModalConversation
from pydantic import BaseModel, Field
from langchain.tools import tool
import cv2
import numpy as np
import logging
import traceback

# === 配置日志 ===
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# === ROS2 相关导入 ===
from std_msgs.msg import String   # 新增：使用简单字符串消息
import re
import textwrap
from ai_msgs.msg import PerceptionTargets, Target, Roi        # <== 新增 Target
from sensor_msgs.msg import RegionOfInterest                 # <== 用 RegionOfInterest 代替 Rect
from std_msgs.msg import Int32
# 全局存储从 ROS 话题回调设置的最新图像（OpenCV BGR 格式）
last_image = None

# 全局存储 ROS2 发布器
bbox_publisher = None

def set_camera_image(image):
    """
    ROS 节点在收到 /publish_image_source 的 Image 消息后，
    会调用此函数更新全局 last_image。
    """
    global last_image
    last_image = image
    logger.debug(f"已更新相机图像，尺寸: {image.shape if image is not None else 'None'}")

def set_bbox_publisher(publisher):
    """
    设置全局的 bbox 发布器，由主节点初始化时调用
    """
    global bbox_publisher
    bbox_publisher = publisher
    logger.info(f"已设置 bbox 发布器: {publisher}")

# === 定义 detect_objects_with_camera 工具的输入模型 ===
@tool(
    description=(
        "使用多模态大模型检测图像中指定物体的位置，"
        "返回边界框坐标并发布到 ROS2 话题 /detect_bbox，"
        "输入参数 object_name 为待检测物体的英文名称"
    ),
    return_direct=False,
)
def detect_objects_with_camera(object_name: str) -> str:
    """
    调用多模态大模型检测图像中的指定物体，返回结果并发布到 ROS2 话题。

    发布格式（std_msgs/String.data）示例:
        '[{"bbox_2d": [0, 51, 186, 249], "label": "box"}]'
    """
    global last_image, bbox_publisher

    logger.info(f"== detect_objects_with_camera | target: {object_name} ==")

    # ----------- 1. 输入合法性检查 -----------
    if last_image is None:
        return "错误：尚未收到图像，请确认相机话题是否正常发布。"
    if bbox_publisher is None:
        return "错误：bbox 发布器未初始化，请在主节点中正确初始化。"

    try:
        # ----------- 2. 保存临时图像 -----------
        img_path = f"detect_{object_name}.jpg"
        cv2.imwrite(img_path, last_image)  # 简单保存
        img_uri = f"file://{os.path.abspath(img_path)}"

        # ----------- 3. 组织多模态 prompt -----------
        prompt = (
            f"Detect all {object_name} in the image and return their "
            "locations in JSON. Each item should look like "
            '{"bbox_2d":[x1,y1,x2,y2],"label":"%s"}' % object_name
        )
        messages = [
            {"role": "system",
             "content": [{"text": "You are an assistant skilled in object detection and localization."}]},
            {"role": "user",
             "content": [{"image": img_uri}, {"text": prompt}]}
        ]

        # ----------- 4. 调用模型 -----------
        resp = MultiModalConversation.call(
            api_key=os.getenv("DASHSCOPE_API_KEY", ""),
            model="qwen-vl-plus-latest",
            messages=messages
        )

        # ----------- 5. 提取模型返回字符串 -----------
        try:
            raw_text = resp["output"]["choices"][0]["message"]["content"][0]["text"]
        except Exception:
            logger.error("模型响应格式异常:\n%s", json.dumps(resp, indent=2, ensure_ascii=False))
            return "模型响应格式错误，请检查日志。"

        logger.debug("模型原始返回:\n%s", raw_text)

        # ----------- 6. 清理 + 解析 JSON -----------
        def _clean(txt: str) -> str:
            """去掉 Markdown fence、去除缩进等"""
            txt = txt.strip()
            txt = re.sub(r"```(json)?", "", txt)  # 去掉 ```json 或 ```
            return textwrap.dedent(txt).strip()

        json_text = _clean(raw_text)

        detections = None
        parse_errors = []

        # (1) 直接解析
        try:
            detections = json.loads(json_text)
        except Exception as e:
            parse_errors.append(str(e))

        # (2) 正则提取第一个 JSON 片段
        if detections is None:
            match = re.search(r"(\{.*\}|\[.*\])", json_text, re.S)
            if match:
                try:
                    detections = json.loads(match.group(0))
                except Exception as e:
                    parse_errors.append(str(e))

        # 解析失败
        if detections is None:
            logger.error("JSON 解析失败：%s", " | ".join(parse_errors))
            return "⚠️ 模型返回无法解析为合法 JSON，请重试。"

        # ----------- 7. 统一为列表形式 -----------
        if isinstance(detections, dict):
            detections = [detections]

        # ----------- 8. 发布到 ROS2 -----------
        # ---------- 5. 构造 PerceptionTargets ----------
        pt_msg = PerceptionTargets()

        # 如果你希望所有 ROI 装在同一个 Target → 按官方示例写法
        tgt = Target()
        for det in detections:
            x1, y1, x2, y2 = det["bbox_2d"]

            roi = Roi()
            # 1) 传入 label
            roi.type = det.get("label", object_name)
            roi.confidence = 1.0

            # 2) 填充 RegionOfInterest
            roi.rect = RegionOfInterest()
            roi.rect.x_offset = int(x1)
            roi.rect.y_offset = int(y1)
            roi.rect.width   = int(x2 - x1)
            roi.rect.height  = int(y2 - y1)
            roi.rect.do_rectify = False

            tgt.rois.append(roi)

        pt_msg.targets.append(tgt)   # 整个列表只放 1 个 Target；里边可能含多个 ROI

        # ---------- 6. 发布 ----------
        bbox_publisher.publish(pt_msg)
        logger.info("已发布 %d 个 ROI 到 /hobot_dnn_detection", len(tgt.rois))

        # logger.info("已发布 %d 个 ROI 到 /hobot_dnn_detection", len(target.rois))

        # ---------- 7. 返回给上层 ----------
        lines = [f"成功检测到 {len(tgt.rois)} 个 {object_name}:"]
        for i, det in enumerate(detections, 1):
            x1, y1, x2, y2 = det["bbox_2d"]
            lines.append(f"  {i}. ({x1},{y1}) → ({x2},{y2})")
        return "\n".join(lines)

    except Exception as e:
        logger.exception("检测异常：%s", e)
        return f"检测过程中发生异常：{e}"
# -------------------------------------------------------------------------

# === 保留原有的工具定义（也添加调试信息） ===
class ObserveArgs(BaseModel):
    prompt: str = Field(..., description="触发字符串，如 'start' ，开始取用上游话题中的图像并描述。")

@tool(
    description="通过上游话题 /publish_image 获取一张图像，并且可以得到描述画面的内容。调用前请发送 'start'。",
    return_direct=False,
    args_schema=ObserveArgs
)
def observe_surroundings_with_camera(prompt: str) -> str:
    from cv2 import resize
    global last_image
    
    print(f"\n=== 执行 observe_surroundings_with_camera, prompt: {prompt} ===")
    
    if prompt.lower() != "start":
        return "要开始观察，请输入 'start'。"
    if last_image is None:
        print("[ERROR] last_image 为 None")
        return "尚未收到图像，请检查 /publish_image 话题是否有发布。"
    
    # 拷贝并调整大小为 (448, 448)
    frame = last_image.copy()
    frame = resize(frame, (448, 448))
    
    # 保存临时文件
    image_path = "captured4vllm.jpg"
    from cv2 import imwrite
    success = imwrite(image_path, frame)
    print(f"保存图像: {success}")
    
    # 调用多模态模型进行描述
    image_uri = f"file://{os.path.abspath(image_path)}"
    messages = [
        {"role": "system", "content": [{"text": "You are an assistant skilled in interpreting images."}]},
        {"role": "user", "content": [{"image": image_uri}, {"text": "请简要描述图像内容，控制在60字内。"}]}
    ]
    
    try:
        resp = MultiModalConversation.call(
            api_key=os.environ.get("DASHSCOPE_API_KEY", ""),
            model="qwen-vl-plus-latest",
            messages=messages
        )
        # 提取文本
        description = resp["output"]["choices"][0]["message"]["content"][0]["text"]
        print(f"描述结果: {description}")
        return description
    except Exception as e:
        print(f"[ERROR] 调用模型失败: {e}")
        traceback.print_exc()
        return f"图像描述失败: {str(e)}"

# 追加 import --------------------------------------------------------------
# 全局 arm 动作发布器
arm_action_publisher = None
def set_arm_action_publisher(pub):
    global arm_action_publisher
    arm_action_publisher = pub
    logger.info("已设置 arm_action_publisher: %s", pub)

class ArmActionArgs(BaseModel):
    action_id: str = Field(..., description="动作 ID 字符串，0~5")

@tool(
    description=(
        "控制机械臂执行动作。输入 action_id 为字符串：\n"
        "0: 抓取模式  1: 跳舞  2: 左看  3: 右看  4: 握手  5: 向前看"
    ),
    return_direct=False,
    args_schema=ArmActionArgs
)
def control_arm_action(action_id: str) -> str:
    """
    发布 std_msgs/Int32 到 /arm_action_command 以执行机械臂动作
    """
    global arm_action_publisher
    if arm_action_publisher is None:
        return "错误：arm_action_publisher 未初始化。"

    try:
        val = int(action_id.strip())
        if val not in range(6):
            return "错误：动作 ID 必须在 0~5 之间。"
    except ValueError:
        return "错误：动作 ID 需为整数字符串。"

    msg = Int32(data=val)
    arm_action_publisher.publish(msg)
    return f"已发送动作指令 {val} 到 /arm_action_command"



class GetTimeArgs(BaseModel):
    format: str = Field(..., description="时间格式: 'iso' | 'rfc' | 'local'")

@tool(
    description="获取当前时间，输入格式之一: iso, rfc, local，返回对应格式的时间字符串。",
    return_direct=False,
    args_schema=GetTimeArgs
)
def get_current_time(format: str) -> str:
    now = datetime.now()
    if format == "iso":
        return now.isoformat()
    elif format == "rfc":
        return now.strftime("%a, %d %b %Y %H:%M:%S %z")
    else:  # local
        return now.strftime("%Y-%m-%d %H:%M:%S")