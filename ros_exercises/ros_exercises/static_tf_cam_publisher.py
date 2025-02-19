'''
Write a ROS node that publishes the relative pose between each camera and the robot as a static transform. 
The static transforms to broadcast are base_link->left_cam and left_cam->right_cam. 
The node should only broadcast each transform once.

Name this node static_tf_cam_publisher.py.

Save a short (~3-5 seconds) gif of RViz just as in part 1, but with your static_tf_cam_publisher.py node running. Name this file static_node.gif and save it in the ros_exercises/rviz directory of your package. This should look much smoother than in part 1.

'''

import rclpy
import numpy as np
from rclpy.node import Node
import random, math, sys
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class StaticTfCamPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_cam_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic', "fake_scan"),
                ('angle_min', (-2/3)*math.pi),
                ('angle_max', (2/3)*math.pi),
                ('range_min', 1.0),
                ('range_max', 10.0),
                ('angle_increment', (1/300)*math.pi)
            ]
        )
        
        
        # ? how am i supposed to use the topic parameter?
        
        self.publisher_ = self.create_publisher(LaserScan, self.get_parameter('topic').value, 10)
        self.range_publisher_ = self.create_publisher(Float32, 'range_test', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        
        self.publish_fake_scan()
    def publish_fake_scan(self):
        # create a LaserScan message
        msg = LaserScan()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = self.get_parameter('angle_min').value
        msg.angle_max = self.get_parameter('angle_max').value
        msg.angle_increment = self.get_parameter('angle_increment').value
        # cuz we are publishing at a rate of 20Hz
        msg.scan_time = 0.05 
        msg.range_min = self.get_parameter('range_min').value
        msg.range_max = self.get_parameter('range_max').value
        # range is angle min to angle max inclusive
        # angle = msg.angle_min
        msg.ranges = []
        # count = 0 
        # while angle <= (msg.angle_max + msg.angle_increment):
        for i in range(401):
            msg.ranges.append(random.uniform(int(msg.range_min), int(msg.range_max)))
            # angle += msg.angle_increment
            # count += 1
            

        
        
        # create the range message
        msg_range = Float32()
        msg_range.data = float(len(msg.ranges))
        self.range_publisher_.publish(msg_range)
        self.publisher_.publish(msg)
        string = f"Publishing Laser Scan: {msg.ranges}"
        self.get_logger().info(string)
        self.get_logger().info('Publishing Range: "%f"' % msg_range.data)
        

def main(args=None):
    rclpy.init(args=args)

    static_tf_cam_publisher = StaticTfCamPublisher()

    rclpy.spin(static_tf_cam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    static_tf_cam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
