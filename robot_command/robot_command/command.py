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
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 30)
		self.posball_subscriber = self.create_subscription(Pose, "/waypoint", self.ball_callback, 30)
		self.posrob_subscriber = self.create_subscription(Pose, "/robot_state", self.rob_callback, 30)
		
		self.ball = np.array([0, 0, 0])
		self.rob = np.array([[0, 0, 0],[0, 0, 0]])
		self.go_to_ball = False
		self.go_to_corner = False
		self.wait_in_corner = False
		self.t0 = 0

	def ball_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.ball[0] = msg.position.x
		self.ball[1] = msg.position.y
		if (not self.wait_in_corner) and (not self.go_to_corner):
		    self.get_logger().info("STATE: GO TO BALL")
		    self.go_to_ball = True

	def rob_callback(self, msg):
		self.get_logger().info(f"Received message : {msg}")
		self.rob[0,0] = msg.position.x
		self.rob[0,1] = msg.position.y
		self.rob[1,2] = msg.position.z
		
		if self.go_to_ball:
		    self.command_ball()
		    
		elif self.go_to_corner:
		    self.command_corner()
		    
		elif self.wait_in_corner:
		    self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.), angular=Vector3(z=0.)))
		    if time.time()-self.t0>10: #si on a attendu assez longtemps
		        self.get_logger().info("STATE: FINISHED")
		        self.wait_in_corner = False
		        
		    
		else: #robot waits
		    self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.), angular=Vector3(z=0.))) 

	def command_ball(self):
		xr = self.rob[0,0]
		yr = self.rob[0,1]
		xb = self.ball[0]
		yb = self.ball[1]
		
		if yr*yb >= 0 or abs(yr) < 0.2 : #same side
			thetp, v = self.rob_to_target(xb, yb)
		else : #has to avoid the net
			thetp, v = self.rob_to_middle()
		self.get_logger().info("v---"+str(v)+"---thetp"+str(thetp))
		self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=thetp))) 
		
		if sqrt((yb-yr)**2 + (xb-xr)**2) < 0.3: #if it is close to the ball, next state
		    self.get_logger().info("STATE: CLOSE TO BALL, NOW LET'S GO TO THE CORNER")
		    self.go_to_ball = False
		    self.go_to_corner = True #next state: go to corner

	def rob_to_target(self, xd, yd):
		xr = self.rob[0,0]
		yr = self.rob[0,1]
		thetr = self.rob[1,2]

		thetd = arctan2(yd-yr, xd-xr)
		thetp = sawtooth(thetd - thetr)
		v = sqrt((yd-yr)**2 + (xd-xr)**2) * 0.06

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
		
	def command_corner(self):
		xr = self.rob[0,0]
		yr = self.rob[0,1]
		xc1 = 6.9 #au dessus à gauche
		yc1 = 13.6
		xc2 = -6.9 #en bas à droite
		yc2 = -13.6
		
		if yr*yc1 >=0:
		    target_x = xc1
		    target_y = yc1
		else:
		    target_x = xc2
		    target_y = yc2
		self.rob_to_target(target_x, target_y)
		if sqrt((target_y-yr)**2 + (target_x-xr)**2) < 0.1: #if it is close to the corner, next state
		    self.get_logger().info("STATE: CLOSE TO CORNER, NOW WAIT")
		    self.go_to_corner = False
		    self.wait_in_corner = True #next state: waits that the ball disappears
		    self.t0 = time.time()
		

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
