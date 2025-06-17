import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2Client:
    def __init__(self, node):
        """
        接受一个 rclpy.node.Node 实例，不再继承 Node。
        """
        self.node = node
        self.action_clients = {}

    def create_action_client(self, turtlebot_name):
        """
        使用传入的 node 创建 ActionClient 而非 self。
        """
        action_client = ActionClient(
            self.node,
            NavigateToPose,
            f'/{turtlebot_name}/navigate_to_pose'
        )
        self.action_clients[turtlebot_name] = action_client
        return action_client

    def send_goal(self, turtlebot_name, target):
        """
        发送导航目标，所有 ROS2 接口都通过 self.node 调用。
        """
        if turtlebot_name not in self.action_clients:
            action_client = self.create_action_client(turtlebot_name)
        else:
            action_client = self.action_clients[turtlebot_name]

        # 等待服务器
        if not action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f'Action server not available for {turtlebot_name}')
            return

        # 构造 Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'[Nav2Client] Sending goal to {turtlebot_name}: {target}')
        send_goal_future = action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        处理服务器接受/拒绝响应。
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Goal rejected')
            return

        self.node.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        处理导航结果回调。
        """
        result = future.result().result
        self.node.get_logger().info(f'Goal result: {result}')
