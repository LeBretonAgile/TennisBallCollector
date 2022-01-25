import sys
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_command.msg import Tuple

class Commande(Node):
	def __init__(self):
		super().__init__("command")
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
		self.pos_subscriber = self.create_subscription(Tuple, "/pos", self.callback, 10)
		

		self.ball = np.array([0],[0])

	def callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.ball[0] = msg.x
		self.ball[1] = msg.y

	def commande(self):
		pass

	def shutdown(self, signum, frame):
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_vel_publisher.publish(twist)
		sys.exit(0)

def main(args=None):
	rclpy.init(args=args)
	node = Commande()
	rate = rclpy.create_rate(30)
	while not rclpy.ok():
		node.commande()
		signal.signal(signal.SIGINT, node.shutdown)
		rclpy.spin(node)
		rate.sleep()