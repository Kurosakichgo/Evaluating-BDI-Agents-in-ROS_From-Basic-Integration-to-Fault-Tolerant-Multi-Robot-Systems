import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_client import Nav2Client
import json

class DynamicCoordinator(Node):
    def __init__(self):
        super().__init__('dynamic_coordinator')
        # 初始目标和代理
        self.targets = [(2.5,1.0), (-6.0, -0.5), (5.0, 1.0), (5.0, -2.0), (-5.5, 2.0), (0.5, -2.0)]
        self.agents = ['agent1','agent2','agent3','agent4','agent5','agent6']
        self.active = list(self.agents)
        self.assigned = {}

        # 内部状态
        self.current_target = None
        self._bid_list = []
        self._rescue_list = []
        self._rescue_pos = None

        # 发布与订阅
        self.pub = self.create_publisher(String, '/coordinator_cmd', 10)
        self.create_subscription(String, '/agent_bid', self.bid_callback, 10)
        self.create_subscription(String, '/agent_failure', self.failure_callback, 10)
        self.create_subscription(String, '/rescue_bid', self.bid_callback_rescue, 10)

        # Nav2 帮助
        self.nav = Nav2Client(self)

        # 创建拍卖定时器，延迟1秒触发首次拍卖
        self._auction_timer = self.create_timer(1.0, self.start_auction)

    def start_auction(self):
        # 如果没有剩余目标，打印一次并停用定时器
        if not self.targets:
            self.get_logger().info('所有目标已分配完毕')
            self.destroy_timer(self._auction_timer)
            return
        # 选取下一个目标
        self.current_target = self.targets.pop(0)
        self._bid_list.clear()
        # 广播拍卖请求
        payload = json.dumps({'type':'auction', 'target': list(self.current_target)})
        self.pub.publish(String(data=payload))
        self.get_logger().info(f'开始拍卖目标 {self.current_target}')

    def bid_callback(self, msg: String):
        data = json.loads(msg.data)
        if data.get('type') != 'auction':
            return
        agent = data.get('agent')
        dist = data.get('distance')
        tgt = tuple(data.get('target', []))
        # 只记录当前拍卖有效代理的报价
        if self.current_target and tgt == self.current_target and agent in self.active:
            self._bid_list.append((agent, dist))
        # 收集完所有active代理报价
        if len(self._bid_list) == len(self.active):
            # 选出最近的赢家
            winner, _ = min(self._bid_list, key=lambda x: x[1])
            self.get_logger().info(f'Winner: {winner} -> {self.current_target}')
            # 发布指令
            cmd = json.dumps({'type':'assign', 'agent': winner, 'target': list(self.current_target)})
            self.pub.publish(String(data=cmd))
            # 更新状态
            self.active.remove(winner)
            self.assigned[winner] = self.current_target
            self.current_target = None
            # 如果还有目标，延迟1秒后再次拍卖；否则结束并销毁定时器
            if self.targets:
                self._auction_timer = self.create_timer(1.0, self.start_auction)
            else:
                self.get_logger().info('所有目标已分配完毕')
                self.destroy_timer(self._auction_timer)

    def failure_callback(self, msg: String):
        self.get_logger().info(f'DEBUG got raw msg: {msg.data}')
        data = json.loads(msg.data)
        failed = data.get('agent')
        pos = tuple(data.get('position', []))
        self.get_logger().warn(f'Received failure from {failed} at {pos}')
        # 清空残留数据并保存故障位置
        self._rescue_list.clear()
        self._rescue_pos = pos
        # 广播救援拍卖请求
        for ag in self.agents:
            if ag != failed:
                req = json.dumps({'type':'rescue_auction','agent':ag,'position': list(pos)})
                self.pub.publish(String(data=req))

    def bid_callback_rescue(self, msg: String):
        data = json.loads(msg.data)
        if data.get('type') != 'rescue_bid':
            return
        agent = data.get('agent')
        dist = data.get('distance')
        # 只记录剩余代理的报价
        if agent in self.agents and self._rescue_pos:
            self._rescue_list.append((agent, dist))
        # 收集完所有救援报价后
        if len(self._rescue_list) == len(self.agents) - 1:
            rescuer, _ = min(self._rescue_list, key=lambda x: x[1])
            self.get_logger().info(f'Rescuer: {rescuer}')
            # 发布救援命令
            cmd1 = json.dumps({'type':'rescue', 'agent': rescuer, 'position': list(self._rescue_pos)})
            self.pub.publish(String(data=cmd1))
            # 发布恢复命令
            original = self.assigned.get(rescuer)
            if original:
                cmd2 = json.dumps({'type':'resume', 'agent': rescuer, 'target': list(original)})
                self.pub.publish(String(data=cmd2))
            # 清理定时器或其他状态无需继续循环
            self._rescue_pos = None


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
