<h1 style="color:#99FF66; text-shadow:2px 2px #000; text-align: center;">
  👀💻 qwen_agent_node (ROS 2) 🧰🤖
</h1>

[查看中文版本](README_CN.md)

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) A ROS 2 node that implements a conversational agent named "地瓜" powered by Alibaba Cloud's Qwen large language model (LLM) via the DashScope API and orchestrated using LangChain. This node listens for text input (e.g., from Automatic Speech Recognition - ASR), incorporates visual context from a camera image topic, generates conversational responses considering its persona and memory, and publishes the responses as text (e.g., for Text-To-Speech - TTS).

## Features

* Integrates Qwen LLM into ROS 2 using DashScope API and LangChain.
* Listens to a text input topic (`std_msgs/msg/String`).
* Publishes generated text responses (`std_msgs/msg/String`).
* Subscribes to a camera image topic, supporting both raw (`sensor_msgs/msg/Image`) and compressed (`sensor_msgs/msg/CompressedImage`) formats.
* Processes image input using OpenCV (`cv2`) and `cv_bridge`, making visual information available to the agent's tools (via `MyTools.set_camera_image`) for multimodal understanding.

> [!WARNING]
>
> cv_bridge only support **numpy<2.0.0**, I recommend use numpy==1.26.4

* Maintains conversation history/memory for contextual responses.
* Initializes with a specific agent persona ("地瓜", brief, emotional responses, avoids "你好").
* Configurable topic names and image format via ROS 2 parameters.
* Requires a DashScope API key for LLM access.

## Installation

1.  **Build the Package:**
    Build the package using `colcon`:
    
    ```bash
    cd ~/your_ros2_ws
    colcon build --packages-select qwen_agent_node
    ```
    
4.  **Source the Workspace:**
    Source your workspace's setup file:
    ```bash
    cd ~/your_ros2_ws
    source install/setup.bash
    ```

## Configuration

### 1. DashScope API Key (CRITICAL)

Before running the node, you **MUST** set your DashScope API key as an environment variable. The `QwenAgent` component relies on this for authentication.

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx' # Replace with your actual key
```



### 2. ROS 2 Parameters

| Parameter Name   | Description                                                  | Type    | Default Value           |
| :--------------- | :----------------------------------------------------------- | :------ | :---------------------- |
| `asr_topic`      | Topic to subscribe to for incoming user text (e.g., from ASR) (`std_msgs/msg/String`). | string  | `/asr_text`             |
| `tts_topic`      | Topic to publish the agent's generated text response (e.g., for TTS) (`std_msgs/msg/String`). | string  | `/tts_text`             |
| `image_topic`    | Topic to subscribe to for camera image input.                | string  | `/publish_image_source` |
| `use_compressed` | If `true`, subscribes to `image_topic` as `sensor_msgs/msg/CompressedImage` like JPEG; if `false`, uses `sensor_msgs/msg/Image`. | boolean | `True`                  |



## Usage

```shell
ros2 run qwen_agent_node qwen_agent_node \
--ros-args \
-p asr_topic:=/asr_text \
-p tts_topic:=/tts_text \
-p image_topic:=/image_jpeg \
-p use_compressed:=True \
```

**Test**

```
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: ""你是谁？""}"
ros2 topic pub --once /asr_text std_msgs/msg/String "{data: ""请你看看周围有什么东西？""}"
```

