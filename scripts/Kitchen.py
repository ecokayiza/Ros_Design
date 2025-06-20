#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import std_msgs.msg
import rospy
import json
from std_msgs.msg import String

py_folder = os.path.dirname(os.path.abspath(__file__))  
voice_file = os.path.join(py_folder, "voice", "order.wav")
class Kitchen:
    def __init__(self):
        self.stored_dishes = {'咖啡': 0, '牛奶': 0, '蛋糕': 0}  # 存储菜品数量
        self.bills = []  # 记录顾客每一单点的菜品
        rospy.Subscriber('/order_consumer', std_msgs.msg.String, self.order_callback)
        rospy.Subscriber('/kitchen_get', std_msgs.msg.String, self.get_dishes_callback)  # 机器人取走菜品
        self.pub = rospy.Publisher('/dishes_kitchen', String, queue_size=10)
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(2), self.process_order)  # 每2秒会有一个菜品被完成
        
    def process_order(self, event):
        if not self.bills:
            rospy.loginfo("没有待处理的订单。")
            return
        current_order = self.bills[0]
        for dish, count in current_order.items():
            if count > 0:
                self.stored_dishes[dish] += 1 # 增加存储的菜品数量
                self.bills[0][dish] -= 1 # 减少订单中的菜品数量
                break
        if all(count == 0 for count in self.bills[0].values()):
            self.bills.pop(0)
        # 发布更新后的菜品数量
        self.pub.publish(json.dumps(self.stored_dishes, ensure_ascii=False))
        
    def get_dishes_callback(self, msg):
        dish_data = json.loads(msg.data)
        for dish, count in dish_data.items():
            if dish in self.stored_dishes:
                self.stored_dishes[dish] -= count

    def order_callback(self, msg):
        order_data = json.loads(msg.data)
        order = order_data.get("order", {})
        self.bills.append(order)
        
        
            
if __name__ == '__main__':
    rospy.init_node('kitchen_node')
    kitchen = Kitchen()
    rospy.spin()  # 保持节点运行，等待事件触发