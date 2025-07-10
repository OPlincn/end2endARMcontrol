<h1 style="color:#00CCFF; text-shadow:2px 2px #000; text-align: center;">
  🤖 aliyun_asr_node (ROS 2) 👂
</h1>

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

一个基于 ROS 2 的实时语音识别 (ASR) 节点，使用阿里云 DashScope 服务（`paraformer-realtime-v1`）。该节点持续监听音频输入设备（通过 `pyalsaaudio`），检测可配置的唤醒词，在超时前截取唤醒词之后的语音，将其发送给 DashScope 进行识别，并将识别结果以文本形式发布到 ROS 话题。

## 功能特点

- 集成阿里云 DashScope 实时 ASR（`paraformer-realtime-v1`）。  
- 使用 `pyalsaaudio` 直接捕获 ALSA 音频（需要安装 ALSA 库）。  
- **唤醒词检测**：仅在检测到指定关键词后才开始处理后续语音。  
- 将识别到的文本（唤醒词之后的语音）发布到可配置的 `std_msgs/msg/String` 话题。  
- 可选地在检测到唤醒词时立即发布唤醒词本身，供系统反馈。  
- 通过 ROS 2 参数可配置唤醒词、音频设备、识别超时、话题名称等。  
- 支持运行时动态更新参数（例如修改唤醒词）。  
- 可选尝试禁用 PulseAudio 干扰，以获得更纯净的 ALSA 访问。  
- 使用 `colorama` 美化终端日志输出。

## 前置条件

```bash
source /opt/tros/humble/setup.bash
python -m pip install dashscope pyalsaaudio colorama
```
**注意**：如果缺少 `libasound2-dev`，`pyalsaaudio` 安装可能会失败，请先安装该系统依赖。

## 安装步骤

1. 切换到你的 ROS 2 工作区：

```bash
cd ~/your_ros2_ws
```

2. 使用 `colcon` 构建本包：

```bash
colcon build --packages-select aliyun_asr_node
```

3. **加载环境**
   

构建完成后，运行：

```bash
source install/setup.bash
```

## 配置说明

### 1. DashScope API Key（**关键**）

由于 DashScope SDK 的初始化机制，**必须**在启动节点前将 API Key 设置为环境变量：

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'  # 替换为你的实际 Key
```

### 2. ROS 2 参数

| 参数名               | 说明                                                         | 类型    | 默认值     |
| -------------------- | ------------------------------------------------------------ | ------- | ---------- |
| `awake_keyword`      | 唤醒词，检测到后开始捕捉语音进行识别（例如 `"你好"`）。      | string  | `你好`     |
| `disable_pulseaudio` | 若为 `True`，则尝试通过清空 `PULSE_SERVER` 环境变量来禁用 PulseAudio。 | boolean | `True`     |
| `audio_device`       | `pyalsaaudio` 使用的 ALSA 设备名（如 `'default'`、`'plughw:1,0'`）。 | string  | `default`  |
| `waiting_timeout`    | 检测到唤醒词后，等待语音输入的超时时间（秒）。               | double  | `3.0`      |
| `pub_topic_name`     | 发布识别结果文本的 ROS 话题名（`std_msgs/msg/String`）。     | string  | `asr_text` |
| `pub_awake_keyword`  | 若为 `True`，检测到唤醒词时立即将该唤醒词发布到 `pub_topic_name`。 | boolean | `False`    |

## 使用示例

```bash
ros2 run aliyun_asr_node asr_node \
  --ros-args \
  -p pub_topic_name:="/asr_text" \
  -p audio_device:="plughw:1,0" \
  -p awake_keyword:="你好"
```