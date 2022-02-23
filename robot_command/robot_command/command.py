import sys
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
from robot_command.roblib import *

#self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=thetp))) 
def sawtooth(x):
	return (x+pi)%(2*pi)-pi

class Command(Node):
	def __init__(self):
		super().__init__("command")
		self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 30)
		self.posball_subscriber = self.create_subscription(Pose, "/waypoint", self.ball_callback, 30)
		self.posrob_subscriber = self.create_subscription(Pose, "/robot_state", self.rob_callback, 30)
		
		self.ball = np.array([-100., -100., 0])
		self.verouille = np.array([-100., -100., 0])
		self.rob = np.array([0., 0., 0])
		self.go_to_ball = False
		self.go_to_corner = False
		self.wait_in_corner = False
		self.t0 = 0
		self.etat = "WAIT"
		self.side = 1.
	
	def move_to(self,target,frein=True):
		vect = target-self.rob
		vect[2]=0.
		dist = np.linalg.norm(vect)
		vect /= dist
		orient = np.arccos(vect[0])
		if vect[1] < 0:
			orient *= -1
		delta_angle = sawtooth(orient-self.rob[2])
		v=1.
		thetp = 1.5* delta_angle
		if dist<2. and frein:
			v=0.05+0.95*dist/2.
		#v *= (1.-delta_angle/np.pi)
		if abs(delta_angle)>np.pi*0.25:
			v=0.
			thetp /= abs(thetp)*1.5
		self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=thetp)))
		return dist

	def ball_callback(self, msg):
		self.ball[0] = msg.position.x
		self.ball[1] = msg.position.y

	def rob_callback(self, msg):
		self.rob[0] = msg.position.x
		self.rob[1] = msg.position.y
		self.rob[2] = msg.position.z
		print(self.etat)
		if self.etat == "WAIT":
			self.etat_WAIT()
		elif self.etat == "CHANGE_SIDE_UP_1":
			self.etat_CHANGE_SIDE_UP_1()
		elif self.etat == "CHANGE_SIDE_UP_2":
			self.etat_CHANGE_SIDE_UP_2()
		elif self.etat == "GO_BALL":
			self.etat_GO_BALL()
		elif self.etat == "GO_TO_CORNER_1":
			self.etat_GO_TO_CORNER_1()
		elif self.etat == "GO_TO_CORNER_2":
			self.etat_GO_TO_CORNER_2()
		elif self.etat == "GO_TO_CORNER_3":
			self.etat_GO_TO_CORNER_3()
	
	def etat_WAIT(self):
		self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.), angular=Vector3(z=0.))) 
		if self.ball[0]>-50:
			if self.ball[1]*self.rob[1]<0:
				if self.rob[1]>0:
					self.side = 1
				else:
					self.side = -1
				self.etat = "CHANGE_SIDE_UP_1"
			else:
				self.etat = "GO_BALL"
	
	def etat_CHANGE_SIDE_UP_1(self):
		dist = self.move_to(np.array([7.,self.side*1.1,0]),False)
		if dist<0.2:
			self.etat = "CHANGE_SIDE_UP_2"
	
	def etat_CHANGE_SIDE_UP_2(self):
		dist = self.move_to(np.array([7.,-self.side*1.1,0]),False)
		if dist<0.2:
			self.etat = "GO_BALL"
	
	def etat_GO_BALL(self):
		dist = self.move_to(self.ball)
		if dist<=0.5:
			self.etat = "GO_TO_CORNER_1"
			if self.rob[1]>0:
				self.side = 1
			else:
				self.side = -1
		
	
	def etat_GO_TO_CORNER_1(self):
		dist = self.move_to(np.array([self.side*3.5,self.side*10.1,0]),False)
		if dist<=1.:
			self.etat="GO_TO_CORNER_2"
	
	def etat_GO_TO_CORNER_2(self):
		dist = self.move_to(np.array([self.side*6.5,self.side*13.1,0]),False)
		if dist<=0.3:
			self.etat="GO_TO_CORNER_3"
	
	def etat_GO_TO_CORNER_3(self):
		dist = self.move_to(np.array([self.side*3.5,self.side*10.1,0]),False)
		if dist<=1.:
			self.etat="WAIT"
	

	
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
