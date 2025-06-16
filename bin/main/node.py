#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/java_to_ros', String, self.callback_command)
        self.rate = rospy.Rate(10)  # 10 Hz

    def callback_command(self, data):
        command = data.data
        if command == "move_forward":
            self.move_forward()
        elif command == "stop":
            self.stop()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2  # Example linear velocity (m/s)
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tb_controller = TurtleBotController()
        tb_controller.run()
    except rospy.ROSInterruptException:
        pass
