# agent1_dynmic.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from nav2_client import Nav2Client

class Agent(Node):
    def __init__(self):
        super().__init__('agent6')
        self.name = 'agent6'
        # 初始位姿，用于拍卖距离计算
        self.initial_pos = (7, 6)
        self.current_target = None
        self._pending_resume = None

        # Nav2 客户端
        self.nav = Nav2Client(self)

        # 订阅协调者命令
        self.cmd_sub = self.create_subscription(
            String, '/coordinator_cmd', self.coordinator_callback, 10)
        # 发布出价
        self.bid_pub = self.create_publisher(String, '/agent_bid', 10)
        # （可选）故障报告发布器，用于模拟 agent 故障
        self.fail_pub = self.create_publisher(String, '/agent_failure', 10)

    def coordinator_callback(self, msg: String):
        data = json.loads(msg.data)
        t = data.get('type')

        # 1. 拍卖请求
        if t == 'auction':
            tgt = tuple(data['target'])
            dist = abs(self.initial_pos[0] - tgt[0]) + abs(self.initial_pos[1] - tgt[1])
            bid = {'agent': self.name, 'distance': dist, 'target': list(tgt)}
            self.get_logger().info(f'出价 {dist} → {tgt}')
            self.bid_pub.publish(String(data=json.dumps(bid)))

        # 2. 正式分配
        if t == 'assign' and data.get('agent') == self.name:
            tgt = tuple(data['target'])
            self.current_target = tgt
            self.get_logger().info(f'收到 assign：{tgt}')
            self.nav.send_goal(self.name, tgt)

        # 3. 收到 rescue
        elif t == 'rescue' and data.get('agent') == self.name:
            pos = tuple(data['position'])
            self.get_logger().info(f'收到 RESCUE：{pos}，取消 {self.current_target}')
            self.nav.cancel_goal()
            self.nav.send_goal(self.name, pos)
            self._pending_resume = self.current_target

        # 4. 收到 resume
        elif t == 'resume' and data.get('agent') == self.name:
            orig = self._pending_resume or tuple(data.get('target', []))
            if orig:
                self.get_logger().info(f'收到 RESUME：恢复导航 {orig}')
                self.nav.send_goal(self.name, orig)
            self._pending_resume = None

    # 可在任意时机调用此方法，模拟当前 agent 故障
    def trigger_failure(self):
        from nav2_client import GetCurrentPose  # 假设 nav2_client 提供获取当前坐标方法
        pos = self.nav.get_current_position()
        msg = {'agent': self.name, 'position': [pos.x, pos.y]}
        self.fail_pub.publish(String(data=json.dumps(msg)))
        self.get_logger().warn(f'** 模拟故障：发布 {msg} 到 /agent_failure **')

def main():
    rclpy.init()
    agent = Agent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
