#!/home/eco/miniconda3/bin/python
import socket
import struct
import numpy as np
import cv2
import pytesseract
# height: 1080
# width: 1920
# encoding: "rgb8"

# number recog example
# need to apt-get install tesseract-ocr
def recognize_number(img):
    # 灰度化
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 二值化
    _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # OCR识别
    config = '--psm 6 -c tessedit_char_whitelist=0123456789'
    text = pytesseract.image_to_string(thresh, config=config)
    # 只保留数字
    number = ''.join(filter(str.isdigit, text))
    return number


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
                # 还原为numpy数组并显示
                img_np = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, channels)).copy()
                cv2.imshow('Image', img_np)
                # OCR识别
                number = recognize_number(img_np)
                if number:
                    # add to cv image
                    cv2.putText(img_np, f'Number: {number}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break  # 按下 q 键退出循环
        except Exception as e:
            print("Connection closed or error:", e)
        finally:
            conn.close()

if __name__ == '__main__':
    main()