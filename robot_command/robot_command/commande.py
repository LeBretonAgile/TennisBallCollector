import sys
import rclpy
import signal
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_command.msg import Balls_pos

class Commande(Node):
	def __init__(self):
		super().__init__("command")
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
		self.pos_subscriber = self.create_subscription(Balls_pos, "/pos", self.callback, 10)

	def callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")

	def shutdown(self, signum, frame):
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_vel_publisher.publish(twist)
		sys.exit(0)

def main(args=None):
	rclpy.init(args=args)
	node = Commande()
	signal.signal(signal.SIGINT, node.shutdown)
	rclpy.spin(node)