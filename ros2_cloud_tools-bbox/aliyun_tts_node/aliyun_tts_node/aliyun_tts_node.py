#!/usr/bin/env python3
import os, time, threading, queue, subprocess
import dashscope
from dashscope.audio.tts_v2 import SpeechSynthesizer as CosySynth, AudioFormat
from dashscope.audio.tts import SpeechSynthesizer as SambertSynth
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AliyunTTSNode(Node):
    def __init__(self):
        super().__init__('aliyun_tts_node')
        # 声明参数及默认值
        self.declare_parameter('tts_method', 'sambert')       # cosyvoice or sambert
        self.declare_parameter('text_topic', '/tts_input')
        self.declare_parameter('result_topic', '/tts_output')
        self.declare_parameter('cosy_model', 'cosyvoice-v1')
        self.declare_parameter('cosy_voice', 'loongstella')
        self.declare_parameter('sambert_model', 'sambert-zhimiao-emo-v1')
        self.declare_parameter('audio_device', 'plughw:0,0')

        # 获取参数
        p = self.get_parameters([
            'tts_method','text_topic','result_topic',
            'cosy_model','cosy_voice','sambert_model','audio_device'
        ])
        self.tts_method  = p[0].get_parameter_value().string_value
        self.text_topic  = p[1].get_parameter_value().string_value
        self.result_topic= p[2].get_parameter_value().string_value
        self.cosy_model  = p[3].get_parameter_value().string_value
        self.cosy_voice  = p[4].get_parameter_value().string_value
        self.sambert_model= p[5].get_parameter_value().string_value
        self.audio_device= p[6].get_parameter_value().string_value

        print(f"当前的tts_method={self.tts_method}, 当前的text_topic={self.text_topic}, 当前的audio_device={self.audio_device}")
        
        self.count = 0
        os.system("rm -rf *.wav")
        # 订阅输入文本，发布音频文件路径
        self.sub = self.create_subscription(
            String, self.text_topic, self.on_text, 10)
        self.pub = self.create_publisher(
            String, self.result_topic, 10)

        self.get_logger().info(
            f"[TTS] 方法={self.tts_method}，订阅={self.text_topic}，发布={self.result_topic}"
        )

        # 播放队列 & 播放线程
        self._play_queue = queue.Queue()
        self._play_thread = threading.Thread(
            target=self._playback_worker, daemon=True
        )
        self._play_thread.start()

    def _playback_worker(self):
        """
        后台线程：持续读取播放队列，按顺序调用 aplay。
        """
        while rclpy.ok():
            try:
                filepath = self._play_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            self.get_logger().info(f"开始播放: {filepath}")
            try:
                subprocess.run(['aplay', '-D', self.audio_device, filepath], check=True)
                self.get_logger().info("播放完毕")
            except Exception as e:
                self.get_logger().error(f"播放失败: {e}")
            finally:
                self._play_queue.task_done()
    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().warn(f"收到文本: “{text}”")
        try:
            if self.tts_method == 'cosyvoice':
                audio = self._cosy(text)
                prefix = 'cosy'
            else:
                audio = self._sambert(text)
                prefix = 'sambert'
        except Exception as e:
            self.get_logger().error(f"TTS 生成失败: {e}")
            return

        # 统一写到同一个文件
        filename = "latest_tts" + str(self.count) + ".wav"
        self.count += 1
        filepath = os.path.join(os.getcwd(), filename)
        with open(filepath, 'wb') as f:
            f.write(audio)
        self.get_logger().info(f"音频已保存并覆盖: {filepath}")

        # 推入播放队列
        self._play_queue.put(filepath)
        self.get_logger().info(f"音频已保存: {filepath}")

        out = String()
        out.data = filepath
        self.pub.publish(out)

    def _cosy(self, text: str) -> bytes:
        synth = CosySynth(
            model=self.cosy_model,
            voice=self.cosy_voice,
            format=AudioFormat.WAV_16000HZ_MONO_16BIT
        )
        result = synth.call(text)
        if result is None:
            raise RuntimeError("CosyVoice 未返回音频")
        return result

    def _sambert(self, text: str) -> bytes:
        resp = SambertSynth.call(
            model=self.sambert_model,
            text=text,
            sample_rate=16000,
            rate=0.75,
            format='wav'
        )
        audio = resp.get_audio_data()
        if audio is None:
            raise RuntimeError("Sambert 未返回音频")
        return audio

def main(args=None):
    rclpy.init(args=args)
    node = AliyunTTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
