#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import std_msgs.msg
import rospy
import json
from std_msgs.msg import String

py_folder = os.path.dirname(os.path.abspath(__file__))  
voice_file = os.path.join(py_folder, "voice", "order.wav")
class Kitchen:
    def __init__(self, consumer_goals):
        self.submit_dishes = {'咖啡': 0, '牛奶': 0, '蛋糕': 0} 
        self.consumer_goals = consumer_goals
        self.incompleted_bills = []  # 记录未完成的订单
        self.goals_list = []  # 记录带发布的目标点 (room，(x, y, yaw_deg)) / / 餐厅目标为 (room, (x, y, yaw_deg))
        self.flag = True  # 标记当前订单是否正在处理
        self.current_order = None  # 当前正在处理的订单
        rospy.Subscriber('/order_consumer', std_msgs.msg.String, self.order_callback)
        rospy.Subscriber('/kitchen_get', std_msgs.msg.String, self.robot_get_callback)  # 接收菜品数据
        self.pub = rospy.Publisher('/goals_instruction', String, queue_size=10) # 发布目标点指令列表
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(2), self.process_order)  # 每2秒会有一个菜品被完成
        
    def process_order(self, event):
        if len(self.incompleted_bills) == 0:
            # rospy.loginfo("没有待处理的订单。")
            return
        if self.flag:
            room, order = self.incompleted_bills[0]
            self.current_order = (room, order.copy())
            self.flag = False 
        for dish, count in self.incompleted_bills[0][1].items():
            if count > 0:
                rospy.loginfo(f"正在处理{self.current_order[0]}的订单")
                self.incompleted_bills[0][1][dish] -= 1  # 减少菜品数量
                break
        if all(count == 0 for count in self.incompleted_bills[0][1].values()):
            self.incompleted_bills.pop(0)
            self.goals_list.append((self.current_order[0], self.consumer_goals[self.current_order[0]]))
            self.flag = True  # 标记当前订单已处理完毕
            for dish, count in self.current_order[1].items():
                if count > 0:
                    self.submit_dishes[dish] += count
            self.flag = True  # 标记当前订单已处理完毕
            
    def robot_get_callback(self, msg):
        text = ""
        for dish, count in self.submit_dishes.items():
            if count > 0:
                text += f"{count}份{dish}，"
        if text != "":
            text += "。"
        self.goals_list = list(set(self.goals_list))  # 去重
        goals_list_and_text = {
            "goals": self.goals_list,
            "text": text
        }
        self.pub.publish(json.dumps(goals_list_and_text, ensure_ascii=False)) 
        self.goals_list = []
        self.submit_dishes = {'咖啡': 0, '牛奶': 0, '蛋糕': 0}  # 重置待提交菜品

    def order_callback(self, msg):
        order_data = json.loads(msg.data)
        order = order_data.get("order", {})
        room = order_data.get("room", "")
        rospy.loginfo(f"厨房接收到{room}房间的订单: {order}")
        self.incompleted_bills.append((room, order))
            
if __name__ == '__main__':
    consumer_goals ={
        "101": (4.761, -0.114, -90),
        "102": (2.167, -0.053, -90),
        "103": (-0.286, -0.027, -90),
        "104": (-0.318, -0.023, 90),
        "105": (1.762, 0.093, 90),       
    }
    rospy.init_node('kitchen_node')
    kitchen = Kitchen(consumer_goals)
    rospy.spin()  # 保持节点运行，等待事件触发