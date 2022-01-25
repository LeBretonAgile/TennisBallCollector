import sys
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from robot_command.roblib import *

class Commande(Node):
	def __init__(self):
		super().__init__("command")
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
		self.posball_subscriber = self.create_subscription(Pose, "/posball", self.ball_callback, 10)
		self.posrob_subscriber = self.create_subscription(Pose, "/posrob", self.rob_callback, 10)
		
		self.ball = np.array([0, 0, 0])
		self.rob = np.array([[0, 0, 0],[0, 0, 0]])

	def ball_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.ball[:] = msg.position

	def rob_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.rob[0,:] = msg.position
		
		self.command()

	def commande(self):
		yr = self.rob[0,1]
		yb = self.ball[1]

		if yr*yb >= 0:
			thetp, v = self.control1()
		else :
			thetp, v = self.control2()
		self.cmd_vel_pubisher.publish(Twist(Linear=Vector3(x=v), angular=Vector3(z=thetp))) 

	def control1(self):
		xd = self.ball[0]
		yd = self.ball[1]

		xr = self.rob[0,0]
		yr = self.rob[0,1]
		thetr = self.rob[1,2]

		thetd = arctan2(yd-yr, xd-xr)
		thetp = sawtooth(thetd - thetr)
		v = sqrt((yd-yr)**2 + (xd-xr)**2) * 0.01

		return thetp, v

	def control2(self):
		
	    xd = 6.6 #proche du mur
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
	node = Commande()
	signal.signal(signal.SIGINT, node.shutdown)
	rclpy.spin(node)
	print("je passe ici")
