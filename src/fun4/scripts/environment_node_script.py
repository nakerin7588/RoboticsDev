#!/usr/bin/python3

"""

"""

# ROS
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# Additional library
import roboticstoolbox as rtb
import random
import os
from ament_index_python.packages import get_package_share_directory
from spatialmath import SE3
from math import pi, sqrt
from tf2_ros import Buffer, TransformListener

# Additional msg srv
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import SetModePosition
from sensor_msgs.msg import JointState

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        # Parameters setup
        self.declare_parameter('rate', 100) # Timer interupt frequency (Hz)
        rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        self.create_timer(1/rate, self.timer_callback)
        
        # Topic Publisher variables
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10) # This publiseher is for publish random task space
        self.end_effector_pub = self.create_publisher(PoseStamped, '/end_effector', 10) # This publiseher is for publish end effector pose
        
        # Topic Subscriber variables
        
        # Service Server variables
        self.create_service(SetModePosition, '/random_target', self.random_target_callback)
        
        # Variables
        self.random_target_values = [0.0, 0.0, 0.0]
        self.joints_angle = [0.0001, 0.0001, 0.0001]
        self.t_0_e = SE3()
        self.t_3_e = SE3()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Start up node
        self.get_logger().info("Environment node has been started")
        # Create robot model
        mdh = [[0.0, 0.0, 0.2, 0], [0.0, pi/2.0, 0.12, pi/2.0], [0.25, 0.0, -0.1, -pi/2.0]]
        revjoint = [] # Create revolute joint
        for i, data in enumerate(mdh):
            revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append revolute joint
        self.robot = rtb.DHRobot(
            [
                revjoint[0],
                revjoint[1],
                revjoint[2]
            ]
            ,tool = SE3.Rx(-pi/2) * SE3.Tz(0.28)
            ,name="3R robot"
        )
        # Show robot5
        # print(self.robot)
        
    # Function
    def target_publish_func(self, position):
        try:
            msg = PoseStamped()
            msg.header.frame_id = "link_0"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            self.target_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Target_publish_function has {e}")
            
    def eff_publish_func(self, tf):
        try:
            if tf == None:
                raise ValueError("Wait for tf.")
            msg = PoseStamped()
            msg.header.frame_id = "link_0"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = tf.translation.x
            msg.pose.position.y = tf.translation.y
            msg.pose.position.z = tf.translation.z
            msg.pose.orientation.w = tf.rotation.w
            msg.pose.orientation.x = tf.rotation.x
            msg.pose.orientation.y = tf.rotation.y
            msg.pose.orientation.z = tf.rotation.z
            self.end_effector_pub.publish(msg)
        except ValueError as e:
            self.get_logger().error(f"Error: {e}")
        
    def timer_callback(self):
        # Publish target position
        self.target_publish_func(self.random_target_values)
        self.eff_publish_func(self.endeffector_pose_compute())    
    
    def endeffector_pose_compute(self):
        try:    
            # Lookup transform from 'base_link' to 'end_effector'
            now = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform(
                'link_0',        # Base frame (replace with the actual base frame name)
                'end_effector',     # End-effector frame (replace with the actual end-effector frame name)
                rclpy.time.Time(),  # Use the latest available transform
                timeout=Duration(seconds=1.0)
            )

            # # Extract the translation part of the transform (x, y, z)
            # translation = trans.transform.translation

            # # Log the translation of the end-effector relative to the base frame
            # self.get_logger().info(f"End Effector Translation: x={translation.x}, y={translation.y}, z={translation.z}")

            return trans.transform
            
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {str(e)}")
            
    def random_func(self, l1 = 0.2, l2 = 0.25, l3 = 0.28):
        while(True):
            x = random.uniform(-(l2+l3), (l2+l3))
            y = random.uniform(-(l2+l3), (l2+l3))
            z = random.uniform(-(l2+l3), (l1+l2+l3))
            if sqrt(x**2 + (z-l1)**2) < l2+l3 and sqrt(y**2 + (z-l1)**2) < l2+l3:
                return x, y, z
            
    def random_target_callback(self, request, response):
        try:
            # Random postion
            # range_variables = self.workspace_calculate_func()
            # for i in range(3):
            #     self.random_target_values[i] = self.random_func()
            # self.random_target_values[2] = self.random_func()
            self.random_target_values[0], self.random_target_values[1], self.random_target_values[2] = self.random_func()
            # Response random position
            response.position.x = self.random_target_values[0]
            response.position.y = self.random_target_values[1]
            response.position.z = self.random_target_values[2]
            response.success = True
            # Log the response
            self.get_logger().info(f"Success to random position. The random position is x:{self.random_target_values[0]}, y:{self.random_target_values[1]}, z:{self.random_target_values[2]}")
            return response
        except Exception as e:
            self.get_logger().info(f"Random target has {e}, Fail to random position.")
            response.success = False
            return response
        
    def get_file_path(self, package_name, relative_path):
        try:
            # Get the path to the package
            package_path = get_package_share_directory(package_name)

            # Append the relative file path to the package path
            file_path = os.path.join(package_path, relative_path)

            if os.path.exists(file_path):
                return file_path
            else:
                raise FileNotFoundError(f"File '{relative_path}' not found in package '{package_name}'")
        except Exception as e:
            print(f"Error: {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
