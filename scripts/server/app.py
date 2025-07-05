from flask import Flask, request, jsonify, render_template
import socket, json
import threading
import time
app = Flask(__name__)
ros_socket = None

@app.route('/', methods=['GET'])
def index():
    return render_template('index.html')

@app.route('/', methods=['POST'])
def ros_order():
    global ros_socket
    if ros_socket is None:
        return 'ROS服务未连接。', 503
    room = request.form.get('room')
    dishes = request.form.getlist('dish') 
    if not room or not dishes:
        return '房间和菜品不能为空', 400

    order = {
        'room': room,
        'dishes': dishes
    }
    try:
        ros_socket.sendall(json.dumps(order).encode())
        return '下单成功'
    except Exception as e:
        print(f"发送订单失败: {e}")
        ros_socket = None
        return '发送订单失败，请重试', 500


def connect_ros():
    global ros_socket
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', 9090))
            sock.listen(1)
            print("等待ROS连接...", flush=True)
            conn, addr = sock.accept()
            print(f"连接到: {addr}", flush=True)
            data = conn.recv(1024).decode()
            if data.strip() == 'ros':
                print("已确认对方为ros", flush=True)
                ros_socket = conn
                # 进入保持连接的循环，直到断开
                while True:
                    try:
                        # 尝试接收心跳或保持连接
                        test = conn.recv(1)
                        if not test:
                            raise Exception("ROS断开连接")
                    except Exception as e:
                        print("ROS连接断开，等待重连...", flush=True)
                        ros_socket = None
                        conn.close()
                        break
            else:
                print("对方不是ros", flush=True)
                conn.close()
            sock.close()
        except Exception as e:
            print(f"连接ROS失败: {e}", flush=True)
            ros_socket = None
            time.sleep(2)  # 等待2秒后重试
    
@app.route('/status')
def ros_status():
    # 这里检测ROS是否在线，比如尝试socket连接或调用服务
    try:
        global ros_socket
        if ros_socket is None:
            return jsonify({'online': False})
        # 尝试发送心跳包
        try:
            ros_socket.send(b'')  # 发送空数据测试连接
            return jsonify({'online': True})
        except:
            ros_socket = None
            return jsonify({'online': False})
    except Exception:
        return jsonify({'online': False})

if __name__ == '__main__':
    threading.Thread(target=connect_ros, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)