from setuptools import find_packages, setup

package_name = 'qwen_agent_node'

setup(
    name=package_name,
    version='1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python',
        'dashscope',
        'langchain',
        'langchain-community',
    ],
    zip_safe=True,
    maintainer='OPlin',
    maintainer_email='oplin@oplin.cn',
    description='LangChain+LLM package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qwen_agent_node = qwen_agent_node.qwen_agent_node:main'
        ],
    },
)
