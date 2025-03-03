# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import random, math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


'''

fake_scan topic
Publish rate
Angle_min
Angle_max
Range_min
Range_max
Angle_increment

'''

class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
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

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
