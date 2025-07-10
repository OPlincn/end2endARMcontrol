<h1 style="color:#E91E63; text-shadow:2px 2px #000; text-align: center;">
  🚀 aliyun_tts_node (ROS 2) 🚀
</h1>

[查看中文版本](README_CN.md)

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) A ROS 2 node that provides Text-To-Speech (TTS) functionality using Alibaba Cloud's DashScope API. It subscribes to a text topic, synthesizes speech using the specified engine (***Sambert*** or ***CosyVoice***), and publishes the resulting audio data to another topic.

## Features

* Integrates Alibaba Cloud DashScope TTS (Sambert and CosyVoice) into ROS 2.
* Subscribes to a text input topic (`std_msgs/msg/String`).
* Supports selection between `sambert` and `cosyvoice` TTS engines.
* Allows specifying different voices .
* Configurable topic names, TTS method, and voice via ROS 2 parameters.
* Requires a DashScope API key for authentication.

## Installation

* **Python Dependencies:** The node requires the DashScope SDK.
    ```bash
    source /opt/tro/humble/setup.bash
    python -m pip install dashscope langchain-community
    ```
    *(Install other Python dependencies if required by the node)*

1. **Build the Package:**
   Build the package using `colcon`:
   ```bash
   cd ~/your_ros2_ws
   colcon build --packages-select aliyun_tts_node
   ```

2. **Source the Workspace:**
   Source your workspace's setup file:
   ```bash
   source install/setup.bash
   ```

## Configuration

### 1. DashScope API Key (CRITICAL)

Before running the node, you **MUST** set your DashScope API key as an environment variable. The node reads this variable for authentication.

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx' # Replace with your actual key
```

### 2.  ROS 2 Parameters

| Parameter Name  | Description                                                  | Type   | Default Value            |
| :-------------- | :----------------------------------------------------------- | :----- | :----------------------- |
| `tts_method`    | The TTS engine to use ('cosyvoice' or 'sambert').            | string | `sambert`                |
| `text_topic`    | Topic to subscribe to for incoming text (`std_msgs/msg/String`). | string | `/tts_input`             |
| `result_topic`  | Topic to publish the *file path* of the saved WAV audio (`std_msgs/msg/String`). | string | `/tts_output`            |
| `cosy_model`    | The specific model name for the CosyVoice engine.            | string | `cosyvoice-v1`           |
| `cosy_voice`    | The voice identifier to use with the CosyVoice engine.       | string | `loongstella`            |
| `sambert_model` | The specific model name for the Sambert engine.              | string | `sambert-zhimiao-emo-v1` |
| `audio_device`  | The ALSA audio device name used by `aplay` for playback (e.g., 'plughw:0,0'). | string | `plughw:0,0`             |



## Usage

`cosy_mode` and `cosy_voice` ***only work*** if you select `sambert` as `tts_method`

```bash
# Use Cosyvoice
ros2 run aliyun_tts_node aliyun_tts_node \
  --ros-args \
  -p tts_method:=cosyvoice \
  -p text_topic:="/tts_input" \
  -p cosy_voice:="loongstella" \
  -p audio_device:="plughw:0,0"
```



```bash
# Use Samberts
ros2 run aliyun_tts_node aliyun_tts_node \
  --ros-args \
  -p tts_method:=sambert \
  -p sambert_model:=sambert-zhimiao-emo-v1 \
  -p audio_device:="plughw:0,0"
```



**To change the tone of the TTS.**
If you choose the `cosyvoice` method, please change the `cosy_voice` parameter.
If you choose `sambert`, change `sambert_model` parameter.

> [!TIP]
>
> The `cosy_model` is only used to specify whether the cosyvoice is the v1 or v2 version, not to replace the timbre. It's safe to say that this parameter doesn't help.

**For tone selection**, please refer to the document at the link, and select 

语音识别/合成 **->** 语言合成-Cosyvoice/Sambert

```
https://bailian.console.aliyun.com/?tab=doc
```

**Test**

You can verify that aliyun_tts_node is running correctly by sending a message with the following command!

```shell
ros2 topic pub --once /tts_input std_msgs/msg/String "{data: ""你是谁？""}"
ros2 topic pub --once /tts_input std_msgs/msg/String "{data: ""你叫什么名字可以和我讲个故事吗非常感谢你！哈哈哈哈哈哈你是谁呀！？你也太搞笑了""}"
```