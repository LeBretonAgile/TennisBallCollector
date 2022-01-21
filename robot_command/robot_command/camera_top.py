import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
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
    cv2.imshow("image", mask)
    cv2.waitKey(1)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    centre = []
    for c in cnts:
    	((x, y), radius) = cv2.minEnclosingCircle(c)
    	centre.append([x,y])
    return np.array(centre)
	

class MyNode(Node):
	def __init__(self):
		super().__init__("camera_top")
		self.cmd_vel_publisher = self.create_publisher(Twist,"/cmd_vel",30)
		self.str_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.callback,30)
		self.twist = Twist()
	
	def callback(self,msg):
		self.cmd_vel_publisher.publish(self.twist)
		img = np.reshape(np.asarray(msg.data),(msg.height,msg.width,3))
		Red = np.array(img[:,:,0])
		img[:,:,0] = img[:,:,2]
		img[:,:,2] = Red
		pos = detect_pos(img)
		self.get_logger().info("Received message !!! "+str(len(pos)))
		
	

def main():
	rclpy.init()
	node = MyNode()
	rclpy.spin(node)
	
