# Ros_Design
A course design for robotics

task:
`25、酒店智能送餐机器人 `
- 首先由个人语音下单，然后机器人根据语音指令取餐，接着通过导航到达指定送餐位置，接着根据门牌号识别实现餐品确认送达。
- 要求：环境需要有三到五个房间作为酒店的模拟，送餐类型可选的至少有三种，最后根据门牌号确定送餐房间。


export TURTLEBOT3_MODEL=waffle
## 建图
```
roslaunch ros_design build_world.launch
roslaunch ros_design build_map.launch
roslaunch teleop_twist_joy teleop.launch # 手柄遥控
rosrun map_server map_saver -f $(rospack find ros_design)/maps/map
```

### 导航2
```
roslaunch ros_design build_world.launch
roslaunch ros_design turtlebot_navigation.launch
```

### 手柄遥控
```
sudo apt-get install ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy
ls /dev/input/js* # 插上手柄检测系统已识别 
roslaunch teleop_twist_joy teleop.launch # 手柄遥控
(need little fix for its launch and config file)
```

### 摄像头
```
机器人模型配置目录
roscd turtlebot3_description

话题
/camera/rgb/image_raw   /camera/depth_image_raw

处理
~/miniconda3/bin/python scripts/process_image.py
/usr/bin/python scripts/get_image.py

```

## Note
vosk权重链接[vosk](https://alphacephei.com/vosk/models)(放在scripts/checkpoints)
