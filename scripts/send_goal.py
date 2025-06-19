#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf
import math
import random

class GoalManager:
    def __init__(self, goal_list, possible_goals):
        self.goal_list = goal_list  # 作为队列
        self.possible_goals = possible_goals
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1.0)
        rospy.Timer(rospy.Duration(10), self.add_random_goal)  # 每10秒添加一个目标

    def send_next_goal(self):
        if self.goal_list:
            x, y, yaw = self.goal_list[0]  # 取队首
            self.send_goal(x, y, yaw)
        # else:
            rospy.loginfo("所有目标点已完成。")
        #     rospy.signal_shutdown("任务完成")

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

        rospy.loginfo("发送目标: x=%.2f, y=%.2f, yaw=%.2f°", x, y, yaw_deg)
        self.pub.publish(goal)

    def status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        text = msg.status_list[-1].text
        if status == 3:
            rospy.loginfo("机器人已成功到达目标位置。")
            if self.goal_list:
                self.goal_list.pop(0)  # 完成后出队列
            rospy.sleep(1.0)
            self.send_next_goal()
        elif status == 4 or status == 5:
            rospy.logwarn("导航失败：%s", text)

    def add_random_goal(self, event):
        goal = random.choice(self.possible_goals)
        self.goal_list.append(goal)
        rospy.loginfo("新目标已加入队列: x=%.2f, y=%.2f, yaw=%.2f°", *goal)
        # 如果队列原本为空，立即发送新目标
        if len(self.goal_list) == 1:
            self.send_next_goal()

if __name__ == '__main__':
    possible_goals = [
        (1.845, 0.450, 90),
        (-0.185, 0.390, 90),       
        (-0.305, -0.500, -90),
        (2.135, -0.570, -90),
        (4.742, -0.752, -90),
    ]
    goal_list = [(1.845, 0.450, 90)]
    rospy.init_node('send_goal_node')
    goalmanager = GoalManager(goal_list, possible_goals)
    rospy.Subscriber('/move_base/status', GoalStatusArray, goalmanager.status_callback)
    goalmanager.send_next_goal()  # 发送第一个目标
    rospy.spin()