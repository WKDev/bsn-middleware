rm -r agv cmd_center cobot
cd ~/ros2_ws && colcon build --symlink-install && source ~/ros2_ws/install/setup.bash