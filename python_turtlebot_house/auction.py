import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import asyncio
import websockets
import json

# Example turtlebot positions and target positions
turtlebot_positions = {
    'tb1': {'x_pose': -5.5, 'y_pose': -2.5},
    'tb2': {'x_pose': -1.5, 'y_pose': 2.5},
    'tb3': {'x_pose': 7.0, 'y_pose': -4.0}
}

target_positions = [
    {'x': 2.5, 'y': 1.0},
    {'x': -6.0, 'y': -0.5},
    {'x': 5.0, 'y': 1.0}
]

# Calculate Manhattan distance
def manhattan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)

# Auction-based allocation
def auction_allocation(turtlebot_positions, target_positions):
    allocation = {}

    while target_positions:
        distances = {}
        
        for target in target_positions:
            for tb, pos in turtlebot_positions.items():
                if tb not in allocation:
                    distance = manhattan_distance(pos['x_pose'], pos['y_pose'], target['x'], target['y'])
                    distances[(tb, target['x'], target['y'])] = distance
        
        winning_bid = min(distances, key=distances.get)
        winning_bot = winning_bid[0]
        target = {'x': winning_bid[1], 'y': winning_bid[2]}
        
        allocation[winning_bot] = target
        
        turtlebot_positions[winning_bot]['x_pose'] = target['x']
        turtlebot_positions[winning_bot]['y_pose'] = target['y']
        
        target_positions = [t for t in target_positions if not (t['x'] == target['x'] and t['y'] == target['y'])]

    return allocation

allocation = auction_allocation(turtlebot_positions, target_positions)

class Nav2Client(Node):

    def __init__(self):
        super().__init__('nav2_client')
        self.action_clients = {}

    def create_action_client(self, turtlebot_name):
        action_client = ActionClient(self, NavigateToPose, f'/{turtlebot_name}/navigate_to_pose')
        self.action_clients[turtlebot_name] = action_client
        return action_client

    def send_goal(self, turtlebot_name, target):
        if turtlebot_name not in self.action_clients:
            action_client = self.create_action_client(turtlebot_name)
        else:
            action_client = self.action_clients[turtlebot_name]

        if not action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server not available for {turtlebot_name}')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to {turtlebot_name}: {target}')
        send_goal_future = action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')

async def main():
    rclpy.init()
    node = Nav2Client()

    for tb, target in allocation.items():
        node.send_goal(tb, target)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
