#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import cv2
import imutils


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


def multiarray_to_numpy(multiarr, multiarr_type, arr_type):
    dims = map(lambda x: x.size, multiarr.layout.dim)
    return np.array(multiarr.data, dtype=multiarr_type).reshape(dims).astype(arr_type)


class WaypointNode(Node):
    def __init__(self):
        super().__init__("waypoint_gen")
        self.waypoint_publisher = self.create_publisher(Pose, "/waypoint", 30)
        self.ball_subscriber = self.create_subscription(Float64MultiArray, "/ball_list", self.ball_callback, 30)
        self.pose = Pose()
        self.waypoint = Pose()

    def ball_callback(self, msg):
        balls = msg.data#multiarray_to_numpy(msg, np.float64, np.float64)
        #print("balls shape:", balls.shape)
        #print("balls data:", balls)

        max_id = 0
        for i in range(1,10):
            if balls[20+i]>balls[20+max_id]:
                max_id = i
        if balls[max_id] == 0 :
            self.waypoint.position.x = -100
            self.waypoint.position.y = -100
        else:
            self.waypoint.position.x = balls[max_id*2]
            self.waypoint.position.y = balls[max_id*2+1]
        	

        self.get_logger().info("Received message with layout, size : " + str(len(balls)))

        self.waypoint_publisher.publish(self.waypoint)


def main():
    rclpy.init()
    node = WaypointNode()
    rclpy.spin(node)
