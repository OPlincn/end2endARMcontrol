from setuptools import find_packages, setup

package_name = 'aliyun_asr_node'

setup(
    name=package_name,
    version='1.0',
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
    description='ASR node package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr_node = aliyun_asr_node.asr_node:main'
        ],
    },
)
