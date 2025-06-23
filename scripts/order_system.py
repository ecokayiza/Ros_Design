#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import random
import rospy
import json
from std_msgs.msg import String
from playsound import playsound
from utils.voice_detect import VoiceDetector
from utils.voice_generate import VoiceGenerator
from utils.parse_text import parse_order_text

py_folder = os.path.dirname(os.path.abspath(__file__))  
voice_file = os.path.join(py_folder, "voice", "order.wav")
class Order_System:
    def __init__(self):
        self.consumers = [
            "101", "102", "103", "104", "105"
        ]
        self.call_pub = rospy.Publisher('/order_consumer', String, queue_size=10)
        self.voice_detector = VoiceDetector(os.path.join(py_folder, "checkpoints", "vosk-model-small-cn-0.22"))
        self.voice_generator = VoiceGenerator()
        self.dishes = ['咖啡', '牛奶', '蛋糕', None]
        rospy.sleep(1)  # 等待发布者准备就绪
        rospy.loginfo("订单系统已启动")
        rospy.Timer(rospy.Duration(30), self.consumer_order)  # 每30秒会有一个随机顾客下单

    def consumer_order(self, event):
        room=self.order_dish()
        order_text = self.voice_detector.recognize(voice_file)  # 检测语音
        rospy.loginfo("音频识别结果: %s", order_text)
        order = parse_order_text(order_text, self.dishes)  # 解析订单文本
        rospy.loginfo("%s 顾客要点单，解析内容为 %s", room, order)
        msg = {
            "room": room,
            "order": order
        }
        self.call_pub.publish(json.dumps(msg, ensure_ascii=False))
    


    def order_dish(self):
        room = random.choice(self.consumers)  # 随机选择一个顾客
        oreder_dishes = {'咖啡': 0, '牛奶': 0, '蛋糕': 0}  # 记录点的菜品数量
        # 假设随机选择三个菜品
        for _ in range(3):
            selected_dish = random.choice(self.dishes)
            if selected_dish is not None:
                oreder_dishes[selected_dish] += 1
        # 如果没有点菜品，则不生成语音
        if all(count == 0 for count in oreder_dishes.values()):
            rospy.loginfo("当前顾客没有点菜品")
            return
        # 生成语音播报
        order_text = f"{room}房间想要" + " ".join([f"{count} 份 {dish}," for dish, count in oreder_dishes.items() if count > 0]) + "。"
        self.voice_generator.generate_voice(order_text, voice_file)
        playsound(voice_file)
        return room
            
        
if __name__ == '__main__':
    rospy.init_node('order_system_node')
    order_system = Order_System()
    rospy.spin()  # 保持节点运行，等待事件触发