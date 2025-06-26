import os
import socket
import threading
import time
import json
import rospy
from std_msgs.msg import String
from playsound import playsound
from utils.voice_generate import VoiceGenerator

py_folder = os.path.dirname(os.path.abspath(__file__))
voice_file = os.path.join(py_folder, "voice", "web_order.wav")

class TCPSocketClient:
    def __init__(self, host, port):
        self.voice_generator = VoiceGenerator()
        self.host = host
        self.port = port
        self.sock = None
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
        self.pub = rospy.Publisher('/order_consumer', String, queue_size=10)
        rospy.sleep(1)  # 等待发布者准备就绪

    def run(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                print("Connected to server at {}:{}".format(self.host, self.port))
                # Send "ros" and "hello" after connecting
                self.sock.sendall(b"ros")
                while True:
                    data = self.sock.recv(1024)
                    if not data:
                        break
                    try:
                        msg = data.decode('utf-8')
                        # 解析JSON
                        obj = json.loads(msg)
                        room = obj.get("room")
                        orders = obj.get("order", {})
                        print("Room:", room)
                        print("order:", orders)
                        order = {
                            'room': room,
                            'order': orders
                        }
                        text = f"{room}房间想要" + " ".join([f"{count}份{dish}，" for dish, count in orders.items() if count > 0]) + "。"
                        self.voice_generator.generate_voice(text, voice_file)
                        playsound(voice_file) 
                        self.pub.publish(json.dumps(order, ensure_ascii=False))
                    except Exception as e:
                        print("Parse error:", e)
            except Exception as e:
                print("Socket error:", e)
                time.sleep(2)
            finally:
                if self.sock:
                    self.sock.close()
                    self.sock = None
                print("Socket closed, retrying connection...")

def main():
    rospy.init_node('web_order')
    host = "121.40.165.119"  # 服务器IP
    port = 9090              # 服务器端口
    client = TCPSocketClient(host, port)
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()