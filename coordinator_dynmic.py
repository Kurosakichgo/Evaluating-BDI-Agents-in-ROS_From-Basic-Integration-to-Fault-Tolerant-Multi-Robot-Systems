import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from nav2_client import Nav2Client
import json

class DynamicCoordinator(Node):
    def __init__(self):
        super().__init__('dynamic_coordinator')
        # Load initial targets and agents
        self.targets = [(x, y) for x, y in [(1,2), (3,4), (5,6), (7,8), (9,1), (2,7)]]
        self.agents = ['agent1', 'agent2', 'agent3', 'agent4', 'agent5', 'agent6']
        self.active = set(self.agents)
        self.assigned = {}

        # ROS subscriptions
        self.create_subscription(String, '/agent_bid', self.bid_callback, 10)
        self.create_subscription(String, '/agent_failure', self.failure_callback, 10)
        self.publisher = self.create_publisher(String, '/coordinator_cmd', 10)

        # Nav2 helper
        self.nav = Nav2Client(self)

        # Start initial auction
        self.get_logger().info('Starting initial auction')
        self.run_auction(self.targets)

    def run_auction(self, pending_targets):
        bids = []
        # Broadcast auction request
        for t in pending_targets:
            msg = json.dumps({'type':'auction', 'target': t})
            self.publisher.publish(String(data=msg))

        # Collect bids
        self.get_logger().info('Waiting for bids...')
        # (Assumes bid_callback populates bids)
        self._bid_list = []
        self._auct_targets = pending_targets

    def bid_callback(self, msg):
        data = json.loads(msg.data)
        agent = data['agent']
        dist = data['distance']
        tgt = tuple(data['target'])
        self._bid_list.append((agent, tgt, dist))
        if len(self._bid_list) == len(self.active):
            # All active agents have bid
            # Select lowest for each target
            winner = min(self._bid_list, key=lambda x: x[2])
            agent, tgt, _ = winner
            self.get_logger().info(f'Winner: {agent} -> {tgt}')
            cmd = json.dumps({'type':'assign', 'agent': agent, 'target': tgt})
            self.publisher.publish(String(data=cmd))
            self.active.remove(agent)
            self.assigned[agent] = tgt
            remaining = [t for t in self._auct_targets if t != tgt]
            if remaining:
                self.run_auction(remaining)

    def failure_callback(self, msg):
        data = json.loads(msg.data)
        failed_agent = data['agent']
        fail_pos = tuple(data['position'])
        self.get_logger().warn(f'Received failure from {failed_agent} at {fail_pos}')

        # Re-auction rescue task among remaining agents
        rescue_bids = []
        # Ask for rescue bids
        for agent in self.agents:
            if agent != failed_agent:
                req = json.dumps({'type':'rescue_auction', 'agent': agent, 'position': fail_pos})
                self.publisher.publish(String(data=req))

        # Collect rescue bids
        self._rescue_list = []
        self._rescue_pos = fail_pos

    def bid_callback_rescue(self, msg):
        # Similar to bid_callback but for rescue
        data = json.loads(msg.data)
        agent = data['agent']
        dist = data['distance']
        self._rescue_list.append((agent, dist))
        if len(self._rescue_list) == len(self.agents) - 1:
            rescuer = min(self._rescue_list, key=lambda x: x[1])[0]
            self.get_logger().info(f'Rescuer: {rescuer}')
            # Instruct rescuer to pause and go rescue
            cmd = json.dumps({'type':'rescue', 'agent': rescuer, 'position': list(self._rescue_pos)})
            self.publisher.publish(String(data=cmd))

            # After rescue, instruct to resume
            resume_cmd = json.dumps({'type':'resume', 'agent': rescuer, 'target': list(self.assigned[rescuer])})
            self.publisher.publish(String(data=resume_cmd))


def main(args=None):
    rclpy.init(args=args)
    coord = DynamicCoordinator()
    rclpy.spin(coord)
    coord.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
