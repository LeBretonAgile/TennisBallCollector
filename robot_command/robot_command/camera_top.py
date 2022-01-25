import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Image
import cv2
import imutils

import numpy as np


def detect_pos(frame,hsvLower=(23, 230,128), hsvUpper=(38,255, 142)): #hsvLower=(248, 130, 27), hsvUpper=(255, 140, 33)):
    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    cv2.imshow("image", frame)
    cv2.waitKey(1)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    fact = 8.0*np.tan( 2.2 *0.5)/(0.5*frame.shape[1])
    
    centre = []
    for c in cnts:
    	((x, y), radius) = cv2.minEnclosingCircle(c)
    	centre.append([-(y-frame.shape[0]*0.5)*fact,-(x-frame.shape[1]*0.5)*fact])
    return np.array(centre)

def dist(a,b):
	return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5
	

class MyNode(Node):
	def __init__(self):
		super().__init__("camera_top")
		self.ball_publisher = self.create_publisher(Float64MultiArray,"/ball_list",30)
		self.str_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.callback,30)
		self.targets = np.zeros((10,2))#Dist/Id/x,y
		for i in range(10):
			self.targets[i,0] = 100 + 100*i
		self.balls=Float64MultiArray()
		self.nb = 0
	
	def linkage(self,pos):
		etat = np.zeros((len(pos),2))
		for i in range(len(pos)):
			etat[i,0]=i
			etat[i,1]=dist(pos[i],self.targets[i,:])
		change = True
		while change:
			change = False
			for i in range(len(etat)):
				for k in range(10):
					distance = dist(pos[i],self.targets[k])
					if distance<etat[i,1] and best(etat,i,distance):
						change = True
						etat[i,1]=distance
						etat[i,0]=k
		for i in range(len(pos)):
			self.targets[int(etat[i,0]),0] = pos[i,0]
			self.targets[int(etat[i,0]),1] = pos[i,1]
		
			
	
	def callback(self,msg):
		img = np.reshape(np.asarray(msg.data),(msg.height,msg.width,3))
		Red = np.array(img[:,:,0])
		img[:,:,0] = img[:,:,2]
		img[:,:,2] = Red
		pos = detect_pos(img)
		if len(pos)<=10:
			self.linkage(pos)
		balls = []
		for i in range(10):
			balls.append(self.targets[i,0])
			balls.append(self.targets[i,1])
		self.balls.data = balls
		self.ball_publisher.publish(self.balls)
		self.get_logger().info("Received message !!! "+str(pos[0]))
		
	

def main():
	rclpy.init()
	node = MyNode()
	rclpy.spin(node)
	
