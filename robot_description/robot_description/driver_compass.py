import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import tf2_ros as tf2
from tf_transformations import euler_from_quaternion
import geometry_msgs

class driver_compass(Node):
	"""docstring for driver_compass"""
	def __init__(self):
		super().__init__("driver_compass")

		# Publisher
		self.msg_compass = Float64()
		self.compass_publisher = self.create_publisher(Float64, '/compass', 50)

		# Subscriber
		self.imu = self.create_subscription(Imu, '/imu', self.imu_callback, 30)

		# Timer
		self.timer_period = 0.1  # seconds
		self.timer = self.create_timer(self.timer_period, self.process)


	def imu_callback(self, msg_imu):
		orientatio_list = [msg_imu.orientation.x, msg_imu.orientation.y, msg_imu.orientation.z, msg_imu.orientation.w]
		euler_angles = euler_from_quaternion(orientatio_list)
		roll = euler_angles[0]
		pitch = euler_angles[1]
		yaw = euler_angles[2]
		self.msg_compass.data = yaw


	def process(self):
		self.compass_publisher.publish(self.msg_compass)


def main(args=None):
    rclpy.init(args=args)
    driver_compass_node = driver_compass()
    rclpy.spin(driver_compass_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver_compass_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()