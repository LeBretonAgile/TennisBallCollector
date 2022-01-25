import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
import cv2
import imutils

import numpy as np

# modified from https://github.com/neka-nat/ros_np_multiarray/blob/master/src/ros_np_multiarray/ros_np_multiarray.py
#def numpy_to_multiarray(arr, arr_type):
#    multiarray = arr_type()
#
#    multiarray.layout.dim = [MultiArrayDimension() 'dim%d' % i,
#                                                 arr.shape[i],
#                                                 arr.shape[i] * arr.dtype.itemsize) for i in range(arr.ndim)]
#
#    multiarray.data = arr.reshape([1, -1])[0].tolist()
#
#    return multiarray

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
        centre.append([-(y-frame.shape[0])*fact,-(x-frame.shape[1])*fact])
    return np.array(centre)


class MyNode(Node):
    def __init__(self):
        super().__init__("camera_top")
        self.ball_publisher = self.create_publisher(Float64MultiArray,"/ball_list",30)
        self.cam_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.callback,30)
        self.balls = Float64MultiArray()

    def callback(self,msg):

        balls =[0.0, 1.0, 2.0, 3.0]
        self.balls.data = balls
#        self.balls.layout =

#        self.balls = numpy_to_multiarray(balls, Float64MultiArray)

        self.ball_publisher.publish(self.balls)
        self.get_logger().info("Sent balls with shape:" + str(self.balls.layout))

        img = np.reshape(np.asarray(msg.data), (msg.height,msg.width,3))
        Red = np.array(img[:,:,0])
        img[:,:,0] = img[:,:,2]
        img[:,:,2] = Red
        pos = detect_pos(img)
        self.get_logger().info("Received message !!! "+str(pos[0]))



def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
