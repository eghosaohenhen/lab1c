'''
The left camera is located 0.05m to the left of the base_link position, 

and the right camera is located 0.05m to the right of the base_link position. 

Both cameras have identity rotation relative to the base_link pose. 

the "forward" direction is the positive-x axis
the "left" direction is the positive-y axis.

'''
# xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        
        # Rx = rotation_matrix(alpha, xaxis)
        # Ry = rotation_matrix(beta, yaxis)
        # Rz = rotation_matrix(gamma, zaxis)
        # R = concatenate_matrices(Rz, Ry, Rx)
        
        # t_matrix = np.eye(3)
        
        # # add the translation to the matrix
        
        # t_matrix = np.hstack((t_matrix, translation_vector.reshape(-1, 1)))
        # # add the bottom part 
        # t_matrix = np.vstack((t_matrix, np.array([0, 0, 0, 1])))

import rclpy
import numpy as np
from rclpy.node import Node
import random, math, sys
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, compose_matrix, quaternion_from_euler, euler_from_matrix, rotation_matrix, concatenate_matrices


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

class DynamicTfCamPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('from_frame', 'odom'),
                ('to_frame', 'base_link'),
            ]
        )
        
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the transform buffer and listener
        
        # listen to the to the odom frame transformations?
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        timer_period = 0.05  # in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def camera_transform(self, y_shift):
        t_matrix = np.eye(3)
        
        # add the translation to the matrix
        translation_vector = np.array([0, y_shift, 0])
        t_matrix = np.hstack((t_matrix, translation_vector.reshape(-1, 1)))
        # add the bottom part 
        t_matrix = np.vstack((t_matrix, np.array([0, 0, 0, 1])))
        
        return t_matrix
           

    def timer_callback(self):
        # pre compute the transformation matrix for the left and right cameras in respect to the base_link frame
        T_LC_R = self.camera_transform(0.05)
        T_RC_R = self.camera_transform(-0.05)
        
        self.get_logger().info(f"Transformation matrix for the left camera relative to the base_link frame: {T_LC_R}\n\n")
        self.get_logger().info(f"Transformation matrix for the right camera relative to the base_link frame: {T_RC_R}\n\n")
        from_frame_rel = self.get_parameter('from_frame').value
        to_frame_rel = self.get_parameter('to_frame').value
        # lookup for the transformations between the odom frame
        # 
        try:
            # rclpy.duration.Duration(seconds=1.0)
            t = self.tf_buffer.lookup_transform(from_frame_rel, to_frame_rel, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f"Failed to transform {from_frame_rel} to {to_frame_rel}: {e}")
            return
        
        # to get transformation matrix T^(odom)_(base_link)
        # we need to get the translation and rotation from the transform message
        # first we build the rotation matrix R = Rz * Ry * Rx
        
        alpha, beta, gamma = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        angles = (alpha, beta, gamma)
        
        self.get_logger().info(f"Rotation angles from {from_frame_rel} to {to_frame_rel}: {angles}")
        
        # then we build the transformation matrix T = [R | t]
        # where t is the translation vector
        
        translation_vector = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        # tranformation of the base_link frame relative to the odom frame
        # W for world/odom frame
        T_R_W = compose_matrix(translate=translation_vector, angles=angles)
        
        
        self.get_logger().info(f"Transformation matrix from {from_frame_rel} to {to_frame_rel}: {T_R_W}")
        
        # we want to get the transformation matrix of the left camera relative to the odom frame
        
        T_LC_W = concatenate_matrices(T_R_W, T_LC_R)
        
        # we want to get the transformation matrix of the right camera relative to the left camera frame
        
        T_RC_LC = concatenate_matrices(np.linalg.inv(T_LC_R), T_RC_R)
        
        # broadcast the transformation matrix of the left camera relative to the odom frame
        translation = T_LC_W[:3, 3]
        angle = quaternion_from_euler(*euler_from_matrix(T_LC_W[:3, :3]))
        self.broadcast_tf(*translation, *angle, 'odom', 'left_cam')
        
        # broadcast the transformation matrix of the right camera relative to the left camera frame
        translation = T_RC_LC[:3, 3]
        angle = quaternion_from_euler(*euler_from_matrix(T_RC_LC[:3, :3]))
        self.broadcast_tf(*translation, *angle, 'left_cam', 'right_cam')
        
    
    def broadcast_tf(self, x,y, z, ax, ay, az, w, parent_frame, child_frame):
        # broadcast the transformation matrix of the left camera relative to the odom frame
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        t.transform.rotation.x = ax
        t.transform.rotation.y = ay
        t.transform.rotation.z = az
        t.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f"Broadcasted transformation matrix from {parent_frame} to {child_frame}")
    
    
def main(args=None):
    rclpy.init(args=args)

    dynamic_tf_cam_publisher = DynamicTfCamPublisher()

    rclpy.spin(dynamic_tf_cam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dynamic_tf_cam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
