import socket
import threading
import time
import json
class TCPSocketClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

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
                        dishes = obj.get("dishes", [])
                        print("Room:", room)
                        print("Dishes:", [d for d in dishes])
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
    host = "192.168.10.103"  # 服务器IP
    port = 9090              # 服务器端口
    client = TCPSocketClient(host, port)
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()