#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import rospy
import json
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf
import math
import cv2
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from playsound import playsound
from utils.yolo_detect import process_image 
from ultralytics import YOLO
from utils.voice_generate import VoiceGenerator
import logging
logging.getLogger().setLevel(logging.WARNING)

py_folder = os.path.dirname(os.path.abspath(__file__))
class GoalManager:
    def __init__(self, restaurant_goal):
        self.yolo = YOLO(os.path.join(py_folder, 'checkpoints', 'svhn_best.pt'))
        self.voice_generator = VoiceGenerator()
        self.bridge = CvBridge()
        self.restaurant_goal = restaurant_goal
        self.cur_goal = restaurant_goal  # 初始目标为厨房
        self.current_goal_list = [restaurant_goal]  # 目的地队列, (room， (x, y, yaw_deg), 点餐内容) / / 餐厅目标为 (room, (x, y, yaw_deg))
        self.next_goal_list = [] # 下一次从厨房出发，目的地缓存队列
        self.has_reached = False  # 检测是否到达目标点，防止状态重复，用门牌号检测
        self.flag = False  # 标记是否已进行过一次发布
        self.kitchen_get_pub = rospy.Publisher('/kitchen_get', std_msgs.msg.String, queue_size=10)
        rospy.sleep(1.0) 
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('/goals_instruction', std_msgs.msg.String, self.get_goals_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_image_callback)
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1.0) 

    def send_next_goal(self):
        if len(self.current_goal_list) == 0:
            rospy.loginfo("所有目标点已完成, 返回厨房")
            self.current_goal_list.append(self.restaurant_goal)  # 如果队列为空，添加餐厅目标
        room, goal = self.cur_goal = self.current_goal_list[0]  # 取队首
        rospy.loginfo("当前目标: %s, 坐标: %s", room, goal)
        self.send_goal(*goal)

    def send_goal(self, x, y, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        rospy.loginfo("send_goal 发送目标点: x=%.2f, y=%.2f, yaw=%s°", x, y, quaternion)
        self.move_pub.publish(goal)
        # 重置到达状态
        self.has_reached = False 
        self.flag = False 
        
    def status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        text = msg.status_list[-1].text
        if status == 3:
            if self.cur_goal[0] == "000":  # 如果到达的是厨房
                if self.has_reached and not self.flag:
                    rospy.loginfo("机器人成功到达目的地：%s, 目标点: %s, 状态: %s", 
                                    self.cur_goal[0], self.cur_goal[1], text)
                    self.current_goal_list.pop(0)  # 移除已完成的目标
                    self.flag = True  # 标记已进行过一次发布
                    self.kitchen_get_pub.publish(f'机器人到达厨房')
                    rospy.sleep(5.0)
                    self.send_next_goal()
            else:
                if self.has_reached and not self.flag:
                    rospy.loginfo("机器人成功到达目的地：%s, 目标点: %s, 状态: %s", 
                                    self.cur_goal[0], self.cur_goal[1], text)
                    self.current_goal_list.pop(0)  # 移除已完成的目标
                    self.flag = True  # 标记已进行过一次发布
                    rospy.sleep(3.0)
                    playsound(os.path.join(py_folder, "voice", f"{self.cur_goal[0]}.wav"))
                    self.send_next_goal()
        elif status == 4 or status == 5:
            rospy.logwarn("机器人未能成功达目的地：%s, 目标点: %s, 状态: %s",
                          self.cur_goal[0], self.cur_goal[1], text)

    def get_goals_callback(self, msg):
        goal_list_and_text = json.loads(msg.data)
        goals = goal_list_and_text.get("goals", [])
        text = goal_list_and_text.get("text", "")
        self.current_goal_list = goals
        rospy.loginfo("接收到新的目标点列表: %s", self.current_goal_list)
        if text != "":
            text = "已取得" + text
            self.voice_generator.generate_voice(text, os.path.join(py_folder, "voice", "get.wav"))
            playsound(os.path.join(py_folder, "voice", "get.wav"))
        
    def get_image_callback(self, msg):
        img_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = 1080, 1920 
        h_half = height // 2
        w_quarter = width // 4
        crop = img_np[0:h_half, w_quarter:width-w_quarter]
        number_str = process_image(crop, self.yolo)
        # 在左上角标注识别到的数字
        cv2.putText(crop, f"No: {number_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("Room No", crop)
        cv2.waitKey(1)
        # rospy.loginfo("识别到的数字: %s, cur_goal: %s, has_reached: %s", number_str, self.cur_goal[0], self.has_reached)
        if number_str == self.cur_goal[0]:
            self.has_reached = True

if __name__ == '__main__':
    restaurant_goal = ("000", (4.728, -0.423, 90)) # 000 代表厨房
    rospy.init_node('send_goal_node')
    goalmanager = GoalManager(restaurant_goal)
    goalmanager.send_next_goal()  # 发送初始目标
    rospy.spin()