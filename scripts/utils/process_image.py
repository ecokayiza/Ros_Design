#!/home/para/anaconda3/envs/anygrasp/bin/python
import os
import socket
import struct
import numpy as np
import cv2
from ultralytics import YOLO
from utils.yolo_detect import process_image

# height: 1080
# width: 1920
# encoding: "rgb8"

py_folder = os.path.dirname(os.path.abspath(__file__))

def recv_all(conn, length):
    data = b''
    while len(data) < length:
        more = conn.recv(length - len(data))
        if not more:
            raise EOFError('Socket closed before receiving all data')
        data += more
    return data

def main():
    width, height, channels = 1920, 1080, 3  
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', 8888))
    s.listen(1)
    print("Python3 server listening on port 8888")
    model = YOLO(os.path.join(py_folder, "checkpoints", "svhn_best.pt"))
    while True:
        conn, addr = s.accept()
        print("Connected by", addr)
        try:
            while True:
                length_data = recv_all(conn, 4)
                if not length_data:
                    break
                frame_length = struct.unpack('!I', length_data)[0]
                image_data = recv_all(conn, frame_length)
                print("Received image frame of length:", frame_length)
                
                # 还原为numpy数组
                img_np = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, channels)).copy()
                # 裁剪
                h_half = height // 2
                w_quarter = width // 4
                crop = img_np[0:h_half, w_quarter:width-w_quarter]

                number_str = process_image(crop,model)
                if number_str.strip() != "":
                    cv2.putText(crop, number_str, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Image with OCR', crop)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break  # 按下 q 键退出循环
        except Exception as e:
            print("Connection closed or error:", e)
        finally:
            conn.close()

if __name__ == '__main__':
    main()