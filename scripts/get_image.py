#!/home/para/anaconda3/envs/anygrasp/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image 
import socket

import struct
import time

class ImageSender:
    def __init__(self, host, port,fps=10):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.fps = fps
        self.last_send = 0
        start_time = time.time()
        while True:
            try:
                self.s.connect((host, port))
                break
            except Exception as e:
                if time.time() - start_time > 10:
                    raise RuntimeError(f"10秒内无法连接到 {host}:{port}") from e
                time.sleep(1)
                
    def send_image(self, data):
        now = time.time()
        if now - self.last_send < 1.0 / self.fps:
            return  # 跳过本帧
        self.last_send = now
        payload = data.data
        length = struct.pack('!I', len(payload))
        try:
            print("Sending image of length:", len(payload))
            self.s.sendall(length)
            self.s.sendall(payload)
        except Exception as e:
            rospy.logerr("Socket error: %s", e)
            self.s.close()
            rospy.signal_shutdown("Socket closed")

def main():
    rospy.init_node('ros_tcp_sender')
    sender = ImageSender('localhost', 8888)        
    rospy.Subscriber('/camera/rgb/image_raw', Image, sender.send_image)
    rospy.spin()
    sender.s.close()

if __name__ == '__main__':
    main()