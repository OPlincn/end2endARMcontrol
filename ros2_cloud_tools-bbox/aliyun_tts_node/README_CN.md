<h1 style="color:#E91E63; text-shadow:2px 2px #000; text-align: center;">
  🚀 aliyun_tts_node (ROS 2) 🚀
</h1>

[![许可证](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

一个基于 ROS 2 的文字转语音（TTS）节点，使用阿里云 DashScope API。它订阅一个文本话题，调用指定的 TTS 引擎（**Sambert** 或 **CosyVoice**）合成语音，并将生成的音频数据发布到另一个话题。

## 功能特点

- 将阿里云 DashScope TTS（Sambert 和 CosyVoice）集成到 ROS 2 中。  
- 订阅文本输入话题（`std_msgs/msg/String`）。  
- 支持在 `sambert` 与 `cosyvoice` 引擎间切换。  
- 可指定不同的声音角色。  
- 通过 ROS 2 参数配置话题名称、TTS 方式、声音角色等。  
- 需要 DashScope API Key 进行鉴权。

## 安装

**Python 依赖：** 本节点依赖 DashScope SDK：
```bash
source /opt/tro/humble/setup.bash
python -m pip install dashscope langchain-community
```
*(如有其它依赖，请根据节点需要另外安装)*

1. **构建节点包**

  ```bash
   cd ~/your_ros2_ws
   colcon build --packages-select aliyun_tts_node
  ```

2. **加载环境**

  ```bash
   source install/setup.bash
  ```

## 配置说明

### 1. DashScope API Key（**关键**）

在运行节点前，**必须**将 DashScope API Key 设置为环境变量，节点会读取此变量进行鉴权：

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'  # 替换为实际 Key
```

### 2. ROS 2 参数

| 参数名          | 说明                                                      | 类型   | 默认值                   |
| --------------- | --------------------------------------------------------- | ------ | ------------------------ |
| `tts_method`    | 使用的 TTS 引擎 (`'cosyvoice'` 或 `'sambert'`)            | string | `sambert`                |
| `text_topic`    | 订阅的文本输入话题（`std_msgs/msg/String`）               | string | `/tts_input`             |
| `result_topic`  | 发布已保存 WAV 文件路径的话题（`std_msgs/msg/String`）    | string | `/tts_output`            |
| `cosy_model`    | CosyVoice 引擎所用的模型版本（如 `cosyvoice-v1`）         | string | `cosyvoice-v1`           |
| `cosy_voice`    | CosyVoice 引擎使用的声音角色标识符                        | string | `loongstella`            |
| `sambert_model` | Sambert 引擎所用的模型名称（如 `sambert-zhimiao-emo-v1`） | string | `sambert-zhimiao-emo-v1` |
| `audio_device`  | `aplay` 播放时使用的 ALSA 设备名（如 `'plughw:0,0'`）     | string | `plughw:0,0`             |

## 使用示例

> **注意**：`cosy_model` 和 `cosy_voice` 仅在 `tts_method` 设为 `cosyvoice` 时生效

```bash
# 使用 CosyVoice 引擎
ros2 run aliyun_tts_node aliyun_tts_node \
  --ros-args \
  -p tts_method:=cosyvoice \
  -p text_topic:="/tts_text" \
  -p cosy_voice:="loongstella" \
  -p audio_device:="plughw:1,0"
# 使用 Sambert 引擎
ros2 run aliyun_tts_node aliyun_tts_node \
  --ros-args \
  -p tts_method:=sambert \
  -p text_topic:="/tts_text" \
  -p sambert_model:=sambert-zhimiao-emo-v1 \
  -p audio_device:="plughw:1,0"
```

### 更改语音风格

- 若选 `cosyvoice`，修改 `cosy_voice` 参数；
- 若选 `sambert`，修改 `sambert_model` 参数。

> [!TIP]
> 
>  `cosy_model` 仅用于指定 CosyVoice v1/v2 版本，不影响音色。

**语调／音色选取**请参考阿里云文档链接：

```
https://bailian.console.aliyun.com/?tab=doc
```

选择 *语音合成* → *语言合成-CosyVoice/Sambert*

## 测试

可通过以下命令验证节点是否正常运行：

~~~bash
ros2 topic pub --once /tts_input std_msgs/msg/String "{data: \"你是谁？\"}"
ros2 topic pub --once /tts_input std_msgs/msg/String "{data: \"你叫什么名字？可以给我讲个故事吗？非常感谢！哈哈哈你是谁呀！？你也太搞笑了\"}"
~~~