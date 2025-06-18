## 建图
### 启动 Gazebo 并加载世界
```
roslaunch ros_design build_world.launch
export TURTLEBOT3_MODEL=burger
roslaunch ros_design build_map.launch
```
### 控制机器人建图
```
rosrun rqt_robot_steering rqt_robot_steering
rosrun map_server map_saver -f $(rospack find ros_design)/maps/map
```