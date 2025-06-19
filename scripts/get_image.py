#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image 
import socket

import struct
import time

class ImageSender:
    def __init__(self, host, port,fps=30):
        self.host = host
        self.port = port
        self.fps = fps
        self.last_send = 0
        self.connect()
    
    def connect(self):
        while True:
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((self.host, self.port))
                print("Connected to server.")
                break
            except Exception as e:
                print("Connect failed, retrying in 1s:", e)
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
            # 尝试重新连接
            self.connect()

def main():
    rospy.init_node('ros_tcp_sender')
    sender = ImageSender('localhost', 8888)        
    rospy.Subscriber('/camera/rgb/image_raw', Image, sender.send_image)
    rospy.spin()
    sender.s.close()

if __name__ == '__main__':
    main()