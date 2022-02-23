import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
import cv2
import imutils

import numpy as np

# modified from https://github.com/neka-nat/ros_np_multiarray/blob/master/src/ros_np_multiarray/ros_np_multiarray.py
def numpy_to_multiarray(arr, arr_type):
    multiarray = arr_type()

    multiarray.layout.dim = [MultiArrayDimension() for i in range(arr.ndim)]

    for i in range(arr.ndim):
        multiarray.layout.dim[i].size = arr.shape[i]
        multiarray.layout.dim[i].stride = arr.shape[i] * arr.dtype.itemsize
        multiarray.layout.dim[i].label = 'dim%d' % i

    multiarray.data = arr.reshape([1, -1])[0].tolist()

    return multiarray

def detect_pos(hsv,h=0,hsvLower=(23, 230,128), hsvUpper=(38,255, 142)): #hsvLower=(248, 130, 27), hsvUpper=(255, 140, 33)):
    """
    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    """
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    fact = (8.0-h)*np.tan( 2.2 *0.5)/(0.5*hsv.shape[1])
    centre = []
    for c in cnts:
    	((x, y), radius) = cv2.minEnclosingCircle(c)
    	centre.append([-(y-hsv.shape[0]*0.5)*fact,-(x-hsv.shape[1]*0.5)*fact])
    return np.array(centre)

def dist(a,b):
	return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5
	

class MyNode(Node):
	def __init__(self):
		super().__init__("camera_top")
		self.ball_publisher = self.create_publisher(Float64MultiArray,"/ball_list",30)
		self.robot_pos_publisher = self.create_publisher(Pose,"/robot_state",30)
		self.str_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.callback,30)
		self.str_subscriber = self.create_subscription(Pose,"/waypoint",self.callback_cible,30)
		self.targets = np.zeros((10,2))#Dist/Id/x,y
		self.times = np.zeros(10)
		self.orientation = 0.
		self.cible = [-100.,-100.]
		for i in range(10):
			self.targets[i,0] = 100 + 100*i
		self.balls=Float64MultiArray()
		self.nb = 0
		self.pos_rob = [100.,0.]
	
	def linkage(self,pos):
		Tab = np.zeros((10,len(pos)))
		for i in range(10):
			for j in range(len(pos)):
				Tab[i,j] = dist(pos[j],self.targets[i])
		identifier = -np.ones(len(pos))
		compte = 0
		while compte!=len(pos):
			mini = np.min( Tab )
			Id = np.where(Tab==mini)
			if identifier[ Id[1][0] ]<-0.5 and len(np.where(identifier==Id[0][0])[0])==0:
				compte+=1
				identifier[ Id[1][0] ] = Id[0][0]
			Tab[Id[0][0],Id[1][0]] = 10**6
		cache = np.zeros(10)
		for i in range(len(pos)):
			self.targets[int(identifier[i]),0] = pos[i,0]
			self.targets[int(identifier[i]),1] = pos[i,1]
			cache[int(identifier[i])] = 1
		for i in range(len(cache)):
			if cache[i]>0.5:
				self.times[i] += 1
			else:
				self.times[i] =  0
		
	def callback_cible(self,msg):
		self.cible[0] = msg.position.x
		self.cible[1] = msg.position.y
	
	def callback(self,msg):
		img = np.reshape(np.asarray(msg.data),(msg.height,msg.width,3))
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		
		blurred = cv2.GaussianBlur(img, (3, 3), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		
		pos = detect_pos(hsv)
		front_robot = detect_pos(hsv,0.4,hsvLower=(145, 240,125), hsvUpper=(160,255, 145))
		back_robot = detect_pos(hsv,0.4,hsvLower=(50, 240,120), hsvUpper=(140,255, 145))
		if (len(front_robot)==1 and len(back_robot)==1):
			self.pos_rob = [ (front_robot[0][0]+back_robot[0][0])*0.5 , (front_robot[0][1]+back_robot[0][1])*0.5 ]
			direction = np.array([front_robot[0][0]-back_robot[0][0],front_robot[0][1]-back_robot[0][1]])
			direction /= np.linalg.norm(direction)
			self.orientation = np.arccos(direction[0])
			if direction[1] < 0:
				self.orientation *= -1
		if len(pos)<=10:
			self.linkage(pos)
		
		if True:
			fact = 8.0*np.tan( 2.2 *0.5)/(0.5*img.shape[1])
			
			for i in range(10):
				x = int( -self.targets[i,1]/fact + img.shape[1]*0.5 )
				y = int( -self.targets[i,0]/fact + img.shape[0]*0.5 )
				cv2.putText(img,str(i),(x,y),cv2.FONT_HERSHEY_SIMPLEX,1,(50,125,255),2,cv2.LINE_4)
			x = int( -self.cible[1]/fact + img.shape[1]*0.5 )
			y = int( -self.cible[0]/fact + img.shape[0]*0.5 )
			cv2.rectangle(img,(x-10,y-10),(x+10,y+10),(255,0,0),2)
			
			fact = (8.0-0.4)*np.tan( 2.2 *0.5)/(0.5*img.shape[1])
			x = int( -self.pos_rob[1]/fact + img.shape[1]*0.5 )
			y = int( -self.pos_rob[0]/fact + img.shape[0]*0.5 )
			img = cv2.circle(img, (x,y), 5, (255,255,255), 2)
			cv2.putText(img,str(int(self.orientation*180/np.pi))+" deg",(x+5,y+20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_4)
		cv2.imshow("image", img)
		cv2.waitKey(1)
		
		balls = []
		for i in range(10):
			balls.append(self.targets[i,0])
			balls.append(self.targets[i,1])
		for i in range(10):
			balls.append(self.times[i])
		self.balls.data = balls
		self.robot_pos_publisher.publish(Pose(position=Point(x=self.pos_rob[0],y=self.pos_rob[1],z=self.orientation)))
		self.ball_publisher.publish(self.balls)
		

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
