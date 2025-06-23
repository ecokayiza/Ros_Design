#!/home/para/anaconda3/envs/anygrasp/bin/python
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
        self.connected = False
        self.connect()
    
    def connect(self):
        while not self.connected:
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((self.host, self.port))
                self.connected = True
                rospy.logger("Connected to server.")
            except Exception as e:
                self.connected = False
                time.sleep(1)
                
    def send_image(self, data):
        now = time.time()
        if now - self.last_send < 1.0 / self.fps:
            return  # 跳过本帧
        self.last_send = now
        payload = data.data
        length = struct.pack('!I', len(payload))
        try:
            self.s.sendall(length)
            self.s.sendall(payload)
        except Exception as e:
            # rospy.logerr("Socket error: %s", e)
            self.s.close()
            self.connected = False
            # 尝试重新连接
            self.connect()

def main():
    rospy.init_node('get_image')
    sender = ImageSender('localhost', 8888)        
    rospy.Subscriber('/camera/rgb/image_raw', Image, sender.send_image)
    rospy.spin()
    sender.s.shutdown(socket.SHUT_RDWR)

if __name__ == '__main__':
    main()