
#!usr/bin/python
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
from utils.yolo_detect import process_image 
from ultralytics import YOLO
import logging
logging.getLogger().setLevel(logging.WARNING)

py_folder = os.path.dirname(os.path.abspath(__file__))
class GoalManager:
    def __init__(self, consumer_goals, restaurant_goal):
        self.yolo = YOLO(os.path.join(py_folder, 'checkpoints', 'svhn_best.pt'))
        self.bridge = CvBridge()
        self.consumer_goals = consumer_goals
        self.restaurant_goal = restaurant_goal
        self.cur_goal = restaurant_goal  # 初始目标为餐厅
        self.in_kitchen_dishes = {'咖啡': 0, '牛奶': 0, '蛋糕': 0}  # 餐厅内菜品数量
        self.carried = {'咖啡': 0, '牛奶': 0, '蛋糕': 0} # 记录当前携带的菜品数量
        self.goal_list = [restaurant_goal]  # 目的地队列, (room， (x, y, yaw_deg), 点餐内容) / / 餐厅目标为 (room, (x, y, yaw_deg))
        self.bill_list = []  # 订单队列 (room, 点餐内容)
        self.kitchen_goal_in_list = True  # 是否有厨房目标在队列中
        self.has_reached = False  # 检测是否到达目标点，防止状态重复
        self.kitchen_get_pub = rospy.Publisher('/kitchen_get', std_msgs.msg.String, queue_size=10)
        rospy.sleep(1.0) 
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('/dishes_kitchen', std_msgs.msg.String, self.kitchen_callback)
        rospy.Subscriber('/order_consumer', std_msgs.msg.String, self.order_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.send_image_callback)
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1.0) 

    def send_next_goal(self):
        if self.goal_list:
            if len(self.goal_list[0]) == 2:  # 如果是餐厅目标
                room, goal = self.cur_goal = self.restaurant_goal
                order = None  # 餐厅目标没有点餐内容
            else:
                room, goal, order = self.cur_goal = self.goal_list[0]  # 取队首
            rospy.loginfo("当前目标: %s, 坐标: %s, 点餐内容: %s", room, goal, order)
            self.send_goal(*goal)
        else:
            rospy.loginfo("所有目标点已完成")
            # self.cur_goal = self.restaurant_goal
            # self.goal_list.append(self.restaurant_goal)  
            # self.send_goal(*self.restaurant_goal[1])

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
        self.has_reached = False  # 重置到达状态
        
    def cal_get_dishes(self, in_kitchen):
        available = self.in_kitchen_dishes.copy()
        result = {dish: 0 for dish in available}
        for bill in self.bill_list:
            order = bill[1]
            # 检查本订单能否全部满足
            can_fulfill = True
            for dish, count in order.items():
                if available.get(dish, 0) < count:
                    can_fulfill = False
                    break
            if not can_fulfill:
                # 只要有一个菜品无法满足，整个订单都不计入
                return result
            # 全部满足才一次性扣减
            for dish, count in order.items():
                result[dish] += count
                available[dish] -= count
            if in_kitchen:
                # 将该订单放到目的地队列中
                self.goal_list.append((bill[0], self.consumer_goals[bill[0]], bill[1]))
        return result
                    
    def kitchen_callback(self, msg):
        if not msg.data:
            return
        dishes = json.loads(msg.data)
        self.in_kitchen_dishes.update(dishes)
        if self.kitchen_goal_in_list:
            return
        else:
            # 检测是否有可满足的订单
            get_dishes = self.cal_get_dishes(in_kitchen=False)
            if any(count > 0 for count in get_dishes.values()):
                self.kitchen_goal_in_list = True
                self.add_goal(("餐厅", self.restaurant_goal[1]))
                if len(self.goal_list) == 1:
                    # 如果队列原本为空，立即发送新目标
                    self.send_next_goal()
        
    def status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        text = msg.status_list[-1].text
        if status == 3:
            rospy.loginfo("机器人成功到达目的地：%s, 目标点: %s, 状态: %s", 
                          self.cur_goal[0], self.cur_goal[1], text)
            if self.goal_list:
                self.goal_list.pop(0)  # 完成后出队列
            if self.cur_goal[0] == "餐厅":
                get_dishes = self.cal_get_dishes(in_kitchen=True)  # 计算需要取的菜品
                self.kitchen_get_pub.publish(json.dumps(get_dishes, ensure_ascii=False))
                for dish, count in get_dishes.items():
                    if count > 0:
                        self.carried[dish] += count
                self.kitchen_goal_in_list = False
                self.has_reached = True  # 到达餐厅后重置到达状态
                rospy.loginfo("已从餐厅取菜: %s", get_dishes)
                rospy.sleep(5.0)
                self.send_next_goal()
            else:
                # 如果是消费者目标，检查是否有订单需要处理
                bill = self.cur_goal[2]
                if self.has_reached:
                    for dish, count in bill.items():
                        if count > 0:
                            if dish in self.carried:
                                self.carried[dish] -= count
                    rospy.loginfo("已将菜品送到消费者: %s, 当前携带的菜品: %s", self.cur_goal[0], self.carried)
                    rospy.sleep(5.0)
                    self.send_next_goal()
            
        elif status == 4 or status == 5:
            rospy.logwarn("机器人未能成功达目的地：%s, 目标点: %s, 状态: %s",
                          self.cur_goal[0], self.cur_goal[1], text)
            
    def add_carried_dishes(self, dishes):
        for dish, count in dishes.items():
            self.carried[dish] += count

    def add_goal(self, goal):
        self.goal_list.append(goal)
        rospy.loginfo("新目标已加入队列: x=%.2f, y=%.2f, yaw=%.2f°", *(goal[1]))
        # 如果队列原本为空，立即发送新目标
        if len(self.goal_list) == 1:
            self.send_next_goal()
            
    def order_callback(self, msg):
        recv = json.loads(msg.data)
        room =  recv.get("consumer")
        order = recv.get("order")
        self.bill_list.append((room, order)) # 添加新订单到订单列表
        rospy.loginfo("收到新顾客点单: %s, 点单内容为 %s", room, order)
        
    def send_image_callback(self, msg):
        if not self.has_reached:
            img_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = 1080, 1920 
            h_half = height // 2
            w_quarter = width // 4
            crop = img_np[0:h_half, w_quarter:width-w_quarter]
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            number_str = process_image(crop, self.yolo)
            # rospy.loginfo("识别到的数字: %s, cur_goal: %s, has_reached: %s", number_str, self.cur_goal[0], self.has_reached)
            if self.cur_goal[0] != '餐厅' and number_str == self.cur_goal[0]:
                self.has_reached = True

if __name__ == '__main__':
    consumer_goals ={
        "101": (4.761, -0.114, -90),
        "102": (2.167, -0.053, -90),
        "103": (-0.286, -0.027, -90),
        "104": (-0.318, -0.023, 90),
        "105": (1.762, 0.093, 90),       
    }
    restaurant_goal = ("餐厅", (4.941, 0.068, 90))
    rospy.init_node('send_goal_node')
    goalmanager = GoalManager(consumer_goals, restaurant_goal)
    goalmanager.send_next_goal()  # 发送初始目标
    rospy.spin()