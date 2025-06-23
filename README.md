# Ros_Design
A course design for robotics

task:
`25、酒店智能送餐机器人 `
- 首先由个人语音下单，然后机器人根据语音指令取餐，接着通过导航到达指定送餐位置，接着根据门牌号识别实现餐品确认送达。
- 要求：环境需要有三到五个房间作为酒店的模拟，送餐类型可选的至少有三种，最后根据门牌号确定送餐房间。
  
## 环境配置
```
# 建议将解释器设为自己的conda环境
pip install -r requirements.txt
```
修改机器人模型配置
```
roscd turtlebot3_description
```
修改 urdf/turtlebot3_waffle.urdf.xacro 195行内容
```
<origin xyz="0.064 -0.065 0.094" rpy="0 -0.7854 0"/>
```
添加世界资源文件
```
cp -r models/* ~/.gazebo/models
```
涉及机器人的终端使用前
```
export TURTLEBOT3_MODEL=waffle
```
## 建图
```
roslaunch ros_design build_world.launch
roslaunch ros_design build_map.launch
roslaunch teleop_twist_joy teleop.launch # 手柄遥控
rosrun map_server map_saver -f $(rospack find ros_design)/maps/map
```

### 导航
```
roslaunch ros_design build_world.launch
roslaunch ros_design turtlebot_navigation.launch
# 手机下单
python scripts/order_tcp.py
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
# 话题
/camera/rgb/image_raw   /camera/depth_image_raw

# 处理(可使用进行遥控查看)
~/miniconda3/bin/python scripts/process_image.py
/usr/bin/python scripts/get_image.py

```

## 模型权重
vosk权重链接[vosk](https://alphacephei.com/vosk/models)(放在scripts/checkpoints)

```
# 安装tts需要
sudo apt-get install espeak
```
tts权重连接[tts](https://coqui.gateway.scarf.sh/v0.6.1_models/tts_models--zh-CN--baker--tacotron2-DDC-GST.zip)(放在~/.local/share/tts)

yolo权重链接[yolo](https://drive.google.com/file/d/1TsKEgMmDxBiGm7AFvVJ0QGOUlJRjUzog/view?usp=sharing)(放在scripts/checkpoints)

