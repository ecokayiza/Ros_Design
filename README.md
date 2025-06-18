# Ros_Design
A course design for robotics

task:
`25、酒店智能送餐机器人 `
- 首先由个人语音下单，然后机器人根据语音指令取餐，接着通过导航到达指定送餐位置，接着根据门牌号识别实现餐品确认送达。
- 要求：环境需要有三到五个房间作为酒店的模拟，送餐类型可选的至少有三种，最后根据门牌号确定送餐房间。
## 建图
### 启动 Gazebo 并加载世界
```
roslaunch ros_design build_world.launch
export TURTLEBOT3_MODEL=waffle
roslaunch ros_design build_map.launch
```
### 控制机器人建图
```
rosrun rqt_robot_steering rqt_robot_steering
rosrun map_server map_saver -f $(rospack find ros_design)/maps/map
```