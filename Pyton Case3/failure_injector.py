#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time

class FailureInjector(Node):
    def __init__(self, agent_name, x, y, delay_sec=0.0):
        super().__init__('failure_injector')
        self.pub = self.create_publisher(String, '/agent_failure', 10)
        self.agent_name = agent_name
        self.x = x
        self.y = y
        self.delay_sec = delay_sec
        # 启动定时器，延迟发布
        self.create_timer(0.1, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if not self.published and time.time() >= self.start_time + self.delay_sec:
            payload = {
                'agent':   self.agent_name,
                'position': [self.x, self.y]
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.pub.publish(msg)
            self.get_logger().info(f'Published agent_failure: {msg.data}')
            self.published = True
            # 发布完成后直接退出
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    # 参数： agent 名字, x, y, [延迟秒数]
    if len(sys.argv) < 4:
        print('用法：failure_injector.py <agent_name> <x> <y> [delay_sec]')
        sys.exit(1)
    agent = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    delay = float(sys.argv[4]) if len(sys.argv)>=5 else 0.0

    node = FailureInjector(agent, x, y, delay)
    # 记录起始时间
    node.start_time = time.time()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


