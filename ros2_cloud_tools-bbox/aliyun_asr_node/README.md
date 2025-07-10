<h1 style="color:#00CCFF; text-shadow:2px 2px #000; text-align: center;">
  🤖 aliyun_asr_node (ROS 2) 👂
</h1>

[查看中文版本](README_CN.md)

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) A ROS 2 node for real-time Automatic Speech Recognition (ASR) using Alibaba Cloud's DashScope service (`paraformer-realtime-v1`). It listens continuously to an audio input device using `pyalsaaudio`, detects a configurable wake word, captures subsequent speech within a timeout, sends it to DashScope for recognition, and publishes the resulting text to a ROS topic.

## Features

* Integrates Alibaba Cloud DashScope real-time ASR (`paraformer-realtime-v1`).
* Uses `pyalsaaudio` for direct ALSA audio capture (requires ALSA libraries).
* Implements **wake word detection**: Only processes speech after a specified keyword.
* Publishes recognized text (spoken after the wake word) to a configurable `std_msgs/msg/String` topic.
* Optionally publishes the wake word itself upon detection for system feedback.
* Configurable wake word, audio device, recognition timeout, and topic names via ROS 2 parameters.
* Supports dynamic parameter updates during runtime (e.g., changing wake word).
* Includes an option to attempt disabling PulseAudio interaction for potentially cleaner ALSA access.
* Uses `colorama` for enhanced terminal logging output.

## Prerequisites

```bash
source /opt/tros/humble/setup.bash
python -m pip install dashscope pyalsaaudio colorama
```
*(Note: `pyalsaaudio` installation might fail if `libasound2-dev` is missing.)*

## Installation

1.  
    Build the package using `colcon`:
    
    ```bash
    cd ~/your_ros2_ws
    colcon build --packages-select aliyun_asr_node
    ```
    
4.  **Source the Workspace:**
    Source your workspace's setup file:
    ```bash
    source install/setup.bash
    ```

## Configuration

### 1. DashScope API Key (CRITICAL)

**IMPORTANT:** Due to how DashScope SDK initializes, you **MUST** set your API key as an environment variable **BEFORE** launching the node. 

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx' # Replace with your actual key
```

### 2. ROS 2 Parameters

| Parameter Name       | Description                                                  | Type    | Default Value |
| :------------------- | :----------------------------------------------------------- | :------ | :------------ |
| `awake_keyword`      | The wake word required before capturing speech for recognition (e.g., "你好"). | string  | `你好`        |
| `disable_pulseaudio` | If `True`, attempts to prevent PulseAudio interference by setting `PULSE_SERVER` environment variable to "". | boolean | `True`        |
| `audio_device`       | The ALSA audio capture device name used by `pyalsaaudio` (e.g., 'default', 'plughw:1,0'). | string  | `default`     |
| `waiting_timeout`    | Seconds to listen for speech after the `awake_keyword` is detected before resetting. | double  | `3.0`         |
| `pub_topic_name`     | Topic name to publish the recognized text (`std_msgs/msg/String`). | string  | `asr_text`    |
| `pub_awake_keyword`  | If `True`, publish the `awake_keyword` itself to `pub_topic_name` immediately upon detection. | boolean | `False`       |

## Usage

```
ros2 run aliyun_asr_node asr_node \
--ros-args \
-p pub_topic_name:="/asr_text" \
-p audio_device:="plughw:0,0" \
-p awake_keyword:="你好"
```

