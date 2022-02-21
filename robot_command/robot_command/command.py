import sys
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
from robot_command.roblib import *

class Command(Node):
	def __init__(self):
		super().__init__("command")
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
		self.posball_subscriber = self.create_subscription(Pose, "/waypoint", self.ball_callback, 10)
		self.posrob_subscriber = self.create_subscription(Pose, "/robot_state", self.rob_callback, 10)
		
		self.ball = np.array([0, 0, 0])
		self.rob = np.array([[0, 0, 0],[0, 0, 0]])

	def ball_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.ball[0] = msg.position.x
		self.ball[1] = msg.position.y

	def rob_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.rob[0,0] = msg.position.x
		self.rob[0,1] = msg.position.y
		
		self.command()

	def command(self):
		yr = self.rob[0,1]
		yb = self.ball[1]

		if yr*yb >= 0 or abs(yr) < 0.2 : #same side
			thetp, v = self.rob_to_ball()
		else : #has to avoid the net
			thetp, v = self.rob_to_middle()
		self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=thetp))) 

	def rob_to_ball(self):
		xd = self.ball[0]
		yd = self.ball[1]

		xr = self.rob[0,0]
		yr = self.rob[0,1]
		thetr = self.rob[1,2]

		thetd = arctan2(yd-yr, xd-xr)
		thetp = sawtooth(thetd - thetr)
		v = sqrt((yd-yr)**2 + (xd-xr)**2) * 0.01

		return thetp, v

	def rob_to_middle(self):
		
		xd = 6.6 #middle waypoint
		yd = 0

		xr = self.rob[0,0]
		yr = self.rob[0,1]
		thetr = self.rob[1,2]

		thetd = arctan2(yd-yr, xd-xr)
		thetp = sawtooth(thetd - thetr)
		v = sqrt((yd-yr)**2 + (xd-xr)**2) * 0.01

		return thetp, v

	def shutdown(self, signum, frame):
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_vel_publisher.publish(twist)
		print("Ctrl + C detected")
		sys.exit(0)

def main(args=None):
	rclpy.init(args=args)
	node = Command()
	signal.signal(signal.SIGINT, node.shutdown)
	rclpy.spin(node)
	print("je passe ici")
