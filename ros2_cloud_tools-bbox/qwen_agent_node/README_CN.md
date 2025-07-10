<h1 style="color:#99FF66; text-shadow:2px 2px #000; text-align: center;">
  👀💻 qwen_agent_node (ROS 2) 🧰🤖
</h1>
[![许可证](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)  
一个 ROS 2 节点，创建了一个名为“地瓜”的代理，通过阿里云 DashScope API 调用 Qwen 大语言模型（LLM），并结合 LangChain 进行编排。该节点监听文本输入（如来自 ASR），订阅摄像头图像话题获取视觉上下文，结合代理的个性和记忆生成对话回复，并将回复以文本形式发布（可供 TTS 使用）。

## 功能特点

* 通过 DashScope API 和 LangChain 将 Qwen LLM 集成到 ROS 2 中。  
* 监听文本输入话题（`std_msgs/msg/String`）。  
* 发布生成的文本回复（`std_msgs/msg/String`）。  
* 订阅摄像头图像话题，支持原始格式（`sensor_msgs/msg/Image`）和压缩格式（`sensor_msgs/msg/CompressedImage`）。  
* 使用 OpenCV（`cv2`）和 `cv_bridge` 处理图像输入，通过 `MyTools.set_camera_image` 调用多模态大模型，以获得视觉反给代理工具链，实现多模态理解。  

> [!WARNING]  
> `cv_bridge` 仅支持 **numpy<2.0.0**，建议使用 `numpy==1.26.4`。

* 保持对话历史/记忆，以实现上下文感知的回复。  
* 初始化时设定代理角色为“地瓜”（回答简洁、有情感，避免使用“你好”）。  
* 可通过 ROS 2 参数配置话题名称和图像格式。  
* 需要 DashScope API Key 进行 LLM 鉴权。

## 安装

1. **构建节点包**  
 ```bash
 cd ~/your_ros2_ws
 colcon build --packages-select qwen_agent_node
 ```

1. **加载环境**
 ``` bash
 cd ~/your_ros2_ws
 source install/setup.bash
 ```

## 配置说明

### 1. DashScope API Key（**关键**）

在运行节点前，**必须**将 DashScope API Key 设置为环境变量，`QwenAgent` 组件会读取该变量进行鉴权：

```bash
复制编辑
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'  # 替换为你的实际 Key
```

### 2. ROS 2 参数

| 参数名           | 说明                                                         | 类型    | 默认值                  |
| ---------------- | ------------------------------------------------------------ | ------- | ----------------------- |
| `asr_topic`      | 订阅的用户文本输入话题（例如来自 ASR）（`std_msgs/msg/String`）。 | string  | `/asr_text`             |
| `tts_topic`      | 发布代理生成的文本回复话题（例如用于 TTS）（`std_msgs/msg/String`）。 | string  | `/tts_text`             |
| `image_topic`    | 订阅的摄像头图像输入话题。                                   | string  | `/publish_image_source` |
| `use_compressed` | 若为 `true`，以 `sensor_msgs/msg/CompressedImage`（如 JPEG）格式订阅 `image_topic`；若为 `false`，使用 `sensor_msgs/msg/Image`。例如RealSense的RGB图像 | boolean | `True`                  |

## 使用示例

```bash
ros2 run qwen_agent_node qwen_agent_node \
  --ros-args \
  -p asr_topic:=/asr_text \
  -p tts_topic:=/tts_text \
  -p image_topic:=/image_jpeg \
  -p use_compressed:=True
```
ros2 run qwen_agent_node qwen_agent_node \
  --ros-args \
  -p asr_topic:=/asr_text \
  -p tts_topic:=/tts_text \
  -p image_topic:=/camera/camera/color/image_raw \
  -p use_compressed:=False

## 测试

```bash
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: \"你是谁？\"}"
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: \"请你看看周围有什么东西？\"}"
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: \"请你检测盒子的坐标！\"}"
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: \"请你跳舞\"}"
```
