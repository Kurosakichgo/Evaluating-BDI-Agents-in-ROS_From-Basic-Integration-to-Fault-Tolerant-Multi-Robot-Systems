import socket
import json
import threading
import rclpy
from nav2_client import Nav2Client  # 确保这个导入是正确的，根据你的项目结构

def handle_client(conn, starting_position, nav_client, turtlebot_name):
    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            data = json.loads(data.decode())
            if 'win' in data and data['win']:
                print(f"{turtlebot_name} 代理赢得了目标 {data['target']} 的竞标")
                # 发送导航目标到 TurtleBot
                nav_client.send_goal(turtlebot_name, data['target'])
                response = {'status': 'ok'}
                conn.sendall(json.dumps(response).encode()) 
                break
            else:
                # 计算从起始位置到目标的曼哈顿距离
                distance = abs(data['x'] - starting_position[0]) + abs(data['y'] - starting_position[1])
                response = {'cost': distance}
                conn.sendall(json.dumps(response).encode())

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    nav_client = Nav2Client()

    host = 'localhost'
    port = 8002  # 每个代理使用不同的端口
    starting_position = (-1.5, 2.5)  # 对每个代理不同
    turtlebot_name = 'tb2'  # 对每个代理使用不同的 TurtleBot 名称

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen()

    # 启动ROS2事件处理线程
    ros_thread = threading.Thread(target=ros_spin_thread, args=(nav_client,))
    ros_thread.start()

    print(f"代理在 {port} 上监听")
    try:
        while True:
            conn, addr = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, starting_position, nav_client, turtlebot_name))
            client_thread.start()
    except KeyboardInterrupt:
        pass
    finally:
        nav_client.destroy_node()
        rclpy.shutdown()
        server_socket.close()
        ros_thread.join()

if __name__ == '__main__':
    main()
