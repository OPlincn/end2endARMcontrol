import os
import sys
import time
import alsaaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import colorama
from rclpy.parameter import Parameter

# 初始化 colorama，用于彩色终端输出
colorama.init(autoreset=True)
#TODO 重要!!! 必须在导入dashscope前设置API Key环境变量，否则会报错，所以要记得export环境变量
from dashscope.audio.asr import Recognition, RecognitionCallback, RecognitionResult # 导入dashscope的ASR相关模块

class ASRCallbackClass(RecognitionCallback):
    """
    语音识别回调类，处理语音识别事件和状态
    管理音频设备，以及处理识别结果和唤醒机制
    """
    def __init__(self) -> None:
        """初始化ASR回调类"""
        super().__init__()
        self.stream = None  # 音频输入流对象
        self.asr_text: str = ''  # 当前识别的文本
        self.max_chars: int = 50  # 识别文本的最大字符数
        self.clear_flag: bool = False  # 清除标志，用于判断是否需要清空识别的文本
        self.user_input: str = ''  # 处理后的用户输入
        self.awake_keyword = "你好"  # 唤醒关键词
        self.awoken = False  # 是否已被唤醒
        self.device_name = "default"  # 音频设备名称
        self.publisher = None
        
        # 新增音频缓冲区，用于积累数据到指定大小后再发送
        self.audio_buffer = bytearray()
        self.buffer_size = 3200  # 与原PyAudio设置相同，确保数据块大小一致
        
        # 新增的状态变量，用于处理唤醒词后的停顿
        self.waiting_for_more = False  # 是否正在等待更多内容
        self.waiting_start_time = 0  # 开始等待的时间戳
        self.waiting_timeout = 3.0  # 最长等待时间(秒)
        
        # 新增识别控制变量
        self.has_new_content = False  # 标记是否有新内容需要发布
        self.is_processing = False  # 标记是否正在处理识别

    def on_open(self):
        """
        开启语音识别器，初始化音频设备
        在识别开始时调用
        """
        print(colorama.Fore.GREEN + '语音识别器已打开。')
        # 避免重复打开设备
        if self.stream is not None:
            try:
                self.stream.close()
            except:
                pass
            
        try:
            # 使用pyalsaaudio替代PyAudio初始化音频设备
            self.stream = alsaaudio.PCM(
                type=alsaaudio.PCM_CAPTURE,  # 音频捕获模式
                mode=alsaaudio.PCM_NONBLOCK,  # 非阻塞模式，提高响应速度
                device=self.device_name,  # 使用指定的音频设备
                channels=1,  # 单声道录音
                rate=16000,  # 采样率16kHz，适合语音识别
                format=alsaaudio.PCM_FORMAT_S16_LE,  # 16位小端格式
                periodsize=1600  # 每次读取的样本数量，减小以提高读取频率
            )
            # 重置音频缓冲区和状态变量
            self.audio_buffer = bytearray()
            self.waiting_for_more = False
            self.awoken = False
            self.user_input = ''
            self.asr_text = ''
            self.has_new_content = False
            self.is_processing = False
            print(colorama.Fore.GREEN + '成功打开音频设备')
        except Exception as e:
            print(colorama.Fore.RED + f'打开音频设备失败: {e}')
            # 尝试列出可用设备以便诊断
            try:
                print("可用音频设备:", alsaaudio.pcms(alsaaudio.PCM_CAPTURE))
            except:
                pass

    def on_close(self):
        """
        关闭语音识别器，释放音频设备资源
        在识别停止时调用
        """
        print(colorama.Fore.RED + '语音识别器已关闭。')
        if self.stream:
            try:
                # 关闭pyalsaaudio的音频流
                self.stream.close()
                self.stream = None
                print(colorama.Fore.YELLOW + '音频设备已关闭')
            except Exception as e:
                print(colorama.Fore.RED + f'关闭音频设备时出错: {e}')

    def on_event(self, result: RecognitionResult):
        """
        处理ASR识别事件回调，更新识别结果并处理唤醒逻辑
        
        Args:
            result: ASR识别结果对象
        """
        # 获取识别句子和是否句子结束的标志
        sentence = result.get_sentence()
        is_end = result.is_sentence_end(sentence)
        
        # 如果需要清空识别文本，则重置
        if self.clear_flag:
            self.clear_flag = False
            self.asr_text = ''

        # 更新当前识别的文本
        self.asr_text = sentence['text']
        
        # 唤醒词检测逻辑
        if not self.awoken and self.awake_keyword in self.asr_text:
            # 检测到唤醒词，设置唤醒状态
            self.awoken = True
            print(colorama.Fore.BLUE + f"检测到唤醒词: {self.awake_keyword}")
            # 发布唤醒词
            if self.publisher is not None:
                msg = String()
                msg.data = self.awake_keyword
                self.publisher.publish(msg)
            # 启动等待更多内容的计时器
            self.waiting_for_more = True
            self.waiting_start_time = time.time()
        
        # 如果已唤醒且句子结束，处理用户输入
        if self.awoken and is_end:
            # 从识别文本中提取唤醒词后的内容
            idx = self.asr_text.find(self.awake_keyword)
            if idx != -1:
                raw_text = self.asr_text[idx + len(self.awake_keyword):]
                # 去除开头所有中文标点符号和空白字符
                self.user_input = raw_text.lstrip(' ，。,.')
            else:
                self.user_input = self.asr_text.strip()

            # 检查是否只是唤醒词，或者用户输入内容为空
            if not self.user_input or self.user_input.isspace():
                # 如果只有唤醒词，但还没超时，继续等待更多内容
                if self.waiting_for_more and (time.time() - self.waiting_start_time < self.waiting_timeout):
                    print(colorama.Fore.YELLOW + f"检测到唤醒词后无内容，等待更多输入... ({int(self.waiting_timeout - (time.time() - self.waiting_start_time))}秒)")
                    return  # 继续等待，不重置唤醒状态
                else:
                    # 如果等待超时，重置等待状态
                    self.waiting_for_more = False
            else:
                # 有实际内容，停止等待
                self.waiting_for_more = False
                print(colorama.Fore.GREEN + f"接收到用户指令: {self.user_input}")
                # 设置标志，表示有新内容需要发布
                self.has_new_content = True

            # 如果识别文本长度超过最大字符数，标记需要清空
            if len(self.asr_text) > self.max_chars:
                self.clear_flag = True
                
            # 完成处理后重置唤醒状态，除非仍在等待更多内容
            if not self.waiting_for_more:
                self.awoken = False


class ASRNode(Node):
    """
    ROS2节点类，负责创建和管理语音识别服务
    将识别结果发布到ROS话题
    """
    def __init__(self):
        """初始化ASR节点，设置发布者和定时器"""
        super().__init__('asr_node')
        
        # 声明ROS参数
        self.declare_parameter('awake_keyword', '你好')
        # self.declare_parameter('api_key', 'something')
        self.declare_parameter('disable_pulseaudio', True)
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('waiting_timeout', 3.0)
        self.declare_parameter('pub_topic_name', 'asr_text')
        self.declare_parameter('pub_awake_keyword', False)

        # 获取参数值
        awake_keyword = self.get_parameter('awake_keyword').value
        # api_key = self.get_parameter('api_key').value
        disable_pulseaudio = self.get_parameter('disable_pulseaudio').value
        self.audio_device = self.get_parameter('audio_device').value
        waiting_timeout = self.get_parameter('waiting_timeout').value
        result_publisher = self.get_parameter('pub_topic_name').value
        self.pub_awake_keyword = self.get_parameter('pub_awake_keyword').value
        print(rf"当前使用的音频设备: {self.audio_device}, 当前使用的唤醒词: {awake_keyword}, 当前使用的发布话题: {result_publisher}")
        # 设置环境变量, 屏蔽掉PulseMusic
        if disable_pulseaudio:
            os.environ["PULSE_SERVER"] = ""
        # os.environ["DASHSCOPE_API_KEY"] = api_key
        
        # 创建参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # 创建ROS发布者，用于发布识别到的文本
        self.publisher = self.create_publisher(String, result_publisher, 10)
        self.get_logger().info('ASR 节点启动成功。')

        # 创建ASR回调对象
        self.asr_callback = ASRCallbackClass()
        # 设置回调对象的参数
        self.asr_callback.awake_keyword = awake_keyword
        self.asr_callback.waiting_timeout = waiting_timeout
        self.asr_callback.device_name = self.audio_device
        if self.pub_awake_keyword:
            self.asr_callback.publisher = self.publisher
        
        # 标记识别器状态
        self.recognition_running = False
        # 初始化识别器
        self.recognition = Recognition(
            model='paraformer-realtime-v1',  # 使用实时语音识别模型
            format='pcm',  # PCM音频格式
            sample_rate=16000,  # 16kHz采样率
            callback=self.asr_callback  # 设置回调对象
        )
        # 启动识别服务
        self.start_recognition()
        
        # 创建定时器，定期调用timer_callback函数处理音频数据
        self.timer = self.create_timer(0.1, self.timer_callback)

    def start_recognition(self):
        """启动语音识别"""
        if not self.recognition_running:
            self.recognition.start()
            self.recognition_running = True
            self.get_logger().debug("语音识别已启动")

    def stop_recognition(self):
        """停止语音识别"""
        if self.recognition_running:
            self.recognition.stop()
            self.recognition_running = False
            self.get_logger().debug("语音识别已停止")

    def parameters_callback(self, params):
        """
        参数回调函数，当ROS参数被修改时调用
        Args:
            params: 被修改的参数列表
        
        Returns:
            参数设置结果
        """
        for param in params:
            if param.name == 'awake_keyword':
                self.asr_callback.awake_keyword = param.value
                self.get_logger().info(f'更新唤醒词为: {param.value}')
            elif param.name == 'api_key':
                os.environ["DASHSCOPE_API_KEY"] = param.value
                self.get_logger().info('已更新API Key')
            elif param.name == 'disable_pulseaudio':
                if param.value:
                    os.environ["PULSE_SERVER"] = ""
                    self.get_logger().info('已禁用PulseAudio')
                else:
                    if "PULSE_SERVER" in os.environ:
                        del os.environ["PULSE_SERVER"]
                    self.get_logger().info('已启用PulseAudio')
            elif param.name == 'audio_device':
                self.audio_device = param.value
                self.get_logger().info(f'更新音频设备为: {param.value}')
                # 需要重新初始化音频设备
                self.asr_callback.on_close()
                time.sleep(0.5)
                # 更新设备名称
                self.asr_callback.device_name = param.value
                self.asr_callback.on_open()
            elif param.name == 'waiting_timeout':
                self.asr_callback.waiting_timeout = param.value
                self.get_logger().info(f'更新等待超时时间为: {param.value}秒')
        
        return True  # 参数设置成功

    def timer_callback(self):
        """
        定时器回调函数，定期执行以读取音频数据并发送给ASR引擎
        还负责检查识别结果并发布到ROS话题
        """
        if self.asr_callback.stream is not None:
            try:
                # 读取多次音频数据，积累足够的数据量
                for _ in range(5):  # 多次读取以获取足够的数据
                    try:
                        # 使用pyalsaaudio的read方法，返回(长度,数据)元组
                        length, data = self.asr_callback.stream.read()
                        if length > 0:  # 只在实际读取到数据时处理
                            # 将读取的数据追加到缓冲区
                            self.asr_callback.audio_buffer.extend(data)
                    except alsaaudio.ALSAAudioError as e:
                        # 非阻塞模式下没有数据可读是正常的，不视为错误
                        if "Resource temporarily unavailable" not in str(e):
                            self.get_logger().error(f"ALSA音频读取错误: {e}")
                        break
                
                # 如果缓冲区积累了足够的数据，发送到ASR引擎
                if len(self.asr_callback.audio_buffer) >= self.asr_callback.buffer_size:
                    # 提取一个完整的音频块
                    audio_chunk = bytes(self.asr_callback.audio_buffer[:self.asr_callback.buffer_size])
                    # 发送到ASR引擎进行识别
                    self.recognition.send_audio_frame(audio_chunk)
                    # 保留缓冲区中剩余的数据供下次使用
                    self.asr_callback.audio_buffer = self.asr_callback.audio_buffer[self.asr_callback.buffer_size:]
                
                # 检查是否还在等待更多内容
                if self.asr_callback.waiting_for_more:
                    # 检查等待是否超时
                    elapsed_time = time.time() - self.asr_callback.waiting_start_time
                    if elapsed_time >= self.asr_callback.waiting_timeout:
                        self.get_logger().info("等待用户输入超时，重置状态")
                        self.asr_callback.waiting_for_more = False
                        self.asr_callback.awoken = False
                
                # 检查是否有新的识别结果需要发布
                if self.asr_callback.has_new_content and not self.asr_callback.is_processing:
                    # 标记为正在处理，防止重复处理
                    self.asr_callback.is_processing = True
                    
                    # 发布识别结果
                    msg = String()
                    msg.data = self.asr_callback.user_input
                    self.publisher.publish(msg)
                    self.get_logger().info(f"发布 ASR 文本: {self.asr_callback.user_input}")
                    
                    # 重置识别器和状态
                    self.stop_recognition()
                    # 清空缓冲区和状态
                    self.asr_callback.audio_buffer = bytearray()
                    self.asr_callback.has_new_content = False
                    time.sleep(0.1)
                    self.start_recognition()
                    # 完成处理
                    self.asr_callback.is_processing = False
                    
            except Exception as e:
                self.get_logger().error(f"处理音频数据时出错: {e}")
                # 如果遇到严重错误，尝试重新初始化
                try:
                    self.asr_callback.on_close()
                    time.sleep(0.5)
                    self.asr_callback.on_open()
                    self.stop_recognition()
                    time.sleep(0.1)
                    self.start_recognition()
                except Exception as re_err:
                    self.get_logger().error(f"重新初始化音频设备失败: {re_err}")

    def destroy_node(self):
        """
        销毁节点时的清理操作
        停止识别服务并关闭音频设备
        """
        # 停止语音识别
        self.stop_recognition()
        # 关闭音频设备
        self.asr_callback.on_close()
        # 调用父类的destroy_node方法
        super().destroy_node()


def main(args=None):
    """
    主函数，启动ROS节点并进入事件循环
    Args:
        args: 命令行参数
    """
    # 初始化ROS系统
    rclpy.init(args=args)
    # 创建ASR节点
    node = ASRNode()
    try:
        # 进入ROS事件循环
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获Ctrl+C退出信号
        node.get_logger().info("ASR 节点即将关闭。")
    finally:
        # 确保在任何情况下都能正确清理资源
        node.destroy_node()
        # 关闭ROS系统
        rclpy.shutdown()


if __name__ == '__main__':
    # 作为脚本直接运行时执行main函数
    main()
