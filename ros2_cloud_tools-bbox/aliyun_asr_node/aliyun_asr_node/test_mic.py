import pyaudio

p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    # 输出设备名称、最大输入通道数量等信息
    print(f"Device {i}: {info.get('name')}, maxInputChannels: {info.get('maxInputChannels')}")
p.terminate()

