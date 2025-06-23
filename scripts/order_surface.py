# Flask 端
from flask import Flask, request
import rospy
from std_msgs.msg import String
import requests
import json
import websocket

app = Flask(__name__)
rospy.init_node('web_order', anonymous=True)
pub = rospy.Publisher('/order_consumer', String, queue_size=10)
pub_test = rospy.Publisher('/from_flask', String, queue_size=10)

def send_to_web(data):
    requests.post('http://localhost:5000/order', json=data)

# @app.route('/')
# def order():
#     data = json.dumps({
#             "message": 'hello ros',
#         }, ensure_ascii=False)
#     pub_test.publish(data)
#     return 'Order received'

def on_message(ws, message):
    """
    WebSocket 消息处理函数
    :param ws: WebSocket 对象
    :param message: 接收到的消息
    """
    rospy.loginfo("接收到 WebSocket 消息: %s", message)
    pub.publish(message)  # 发布到 ROS 话题

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
    # ws = websocket.WebSocketApp("ws://对方公网IP:端口/ws", on_message=on_message)
    # ws.run_forever()