#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf
import math
import random
from voice_detect import VoiceDetector  
from voice_generate import VoiceGenerator

py_folder = os.path.dirname(os.path.abspath(__file__))
class GoalManager:
    def __init__(self, consumer_goals, restaurant_goal):
        # self.voice_detector = VoiceDetector(os.path.join(py_folder, "checkpoints", "vosk-model-small-cn-0.22"))
        # self.voice_generator = VoiceGenerator()
        self.consumer_goals = consumer_goals
        self.restaurant_goal = restaurant_goal
        self.cur_goal = ("原点", (0,0,0))
        self.goal_list = []  # 作为队列
        self.dishes = ['A', 'B', 'C', None, None]  # 菜品列表
        self.bills = []  # 记录顾客每一单点的菜品
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1.0)  # 等待发布器准备就绪
        self.consumer_order(None)  # 初始化时立即一个顾客订单
        rospy.Timer(rospy.Duration(30), self.consumer_order)  # 每30秒会有一个随机顾客下单

    def send_next_goal(self):
        if self.goal_list:
            des, goal = self.goal_list[0]  # 取队首
            self.cur_goal = (des, goal)
            rospy.loginfo("当前目标: %s, 坐标: %s", des, goal)
            self.send_goal(*goal)
        else:
            rospy.loginfo("所有目标点已完成。")

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
        self.pub.publish(goal)

    def order_dish(self):
        oreder_dishes = {'A': 0, 'B': 0, 'C': 0}  # 记录点的菜品数量
        # 假设随机选择三个菜品
        for _ in range(3):
            selected_dish = random.choice(self.dishes)
            if selected_dish is not None:
                oreder_dishes[selected_dish] += 1
        return oreder_dishes  #  返回一个字典，记录每种菜品的数量

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
            self.bills.append(self.order_dish())  # 记录订单
            rospy.sleep(5.0)
            self.send_next_goal()
        elif status == 4 or status == 5:
            rospy.logwarn("机器人未能成功达目的地：%s, 目标点: %s, 状态: %s",
                          self.cur_goal[0], self.cur_goal[1], text)

    def consumer_order(self, event):
        goal = random.choice(list(self.consumer_goals.items()))
        self.add_goal((goal[0], goal[1]))
        rospy.loginfo("新顾客要点单: %s", goal[0])
    
    def add_goal(self, goal):
        self.goal_list.append(goal)
        rospy.loginfo("新目标已加入队列: x=%.2f, y=%.2f, yaw=%.2f°", *(goal[1]))
        # 如果队列原本为空，立即发送新目标
        rospy.loginfo("当前目标队列: %s", self.goal_list)
        if len(self.goal_list) == 1:
            self.send_next_goal()

if __name__ == '__main__':
    consumer_goals ={
        "101": (4.742, -0.752, -90),
        "102": (2.135, -0.714, -90),
        "103": (-0.305, -0.737, -90),
        "104": (-0.291, 0.543, 90),
        "105": (1.865, 0.540, 90),       
    }
    restaurant_goal = ("餐厅", (4.941, 0.068, 90))
        
    rospy.init_node('send_goal_node')
    goalmanager = GoalManager(consumer_goals, restaurant_goal)
    rospy.Subscriber('/move_base/status', GoalStatusArray, goalmanager.status_callback)
    goalmanager.send_next_goal()  # 发送第一个目标
    rospy.spin()