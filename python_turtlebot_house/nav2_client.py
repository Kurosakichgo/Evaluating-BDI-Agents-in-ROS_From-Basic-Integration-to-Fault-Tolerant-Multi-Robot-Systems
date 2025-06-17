import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

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
