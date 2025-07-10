from setuptools import find_packages, setup

package_name = 'aliyun_tts_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OPlin',
    maintainer_email='oplin@oplin.cn',
    description='Aliyun TTS ROS2 node using DashScope CosyVoice and Sambert',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aliyun_tts_node = aliyun_tts_node.aliyun_tts_node:main',
        ],
    },
)
