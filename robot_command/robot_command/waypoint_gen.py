#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import cv2
import imutils


class WaypointNode(Node):
    def __init__(self):
        super().__init__("camera_top")
        self.waypoint_publisher = self.create_publisher(Pose, "/waypoint", 30)
        self.ball_subscriber = self.create_subscription(Float64MultiArray, "/ball_list", self.ball_callback, 30)
        self.pose = Pose()
        self.waypoint = Float64MultiArray()

    def ball_callback(self, msg):
        balls_shape = msg.layout
        balls = msg.data
#        print("layout:", balls_shape)
#        print("data:", balls)

        self.get_logger().info("Received message with layout: " + str(balls_shape) + "\and data:\n" + str(balls))


def main():
    rclpy.init()
    node = WaypointNode()
    rclpy.spin(node)
