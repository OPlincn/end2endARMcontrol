## run the RealSense camera with RGBD enabled
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
## publish the segmentation roi
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
source /opt/tros/humble/setup.bash