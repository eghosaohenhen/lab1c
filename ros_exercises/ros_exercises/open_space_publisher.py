
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('subscriber_topic', "fake_scan"),
                ('publisher_topic', "open_space")
            ]
        )
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('subscriber_topic').value,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # self.angle_publisher_ = self.create_publisher(Float32, 'open_space/angle', 10)
        # self.distance_publisher_ = self.create_publisher(Float32, 'open_space/distance', 10)
        self.publisher_ = self.create_publisher(OpenSpace, self.get_parameter('publisher_topic').value, 10)

    def listener_callback(self, scan):
        # self.get_logger().info(f'Recieved laser scan message at time: {scan.header.stamp}')
        max_distance = float('-inf')
        max_angle = float('-inf')
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > max_distance:
                max_distance = scan.ranges[i]
                max_angle = scan.angle_min + i * scan.angle_increment
        
        msg = OpenSpace()
        msg.angle = max_angle
        msg.distance = max_distance
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing OpenSpace: {msg.angle}, {msg.distance}')
        


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
