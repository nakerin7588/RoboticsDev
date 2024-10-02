#!/usr/bin/python3

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# Additional library
import roboticstoolbox as rtb
import random
from spatialmath import SE3
from tf2_ros import Buffer, TransformListener
import numpy as np

# Additional msg srv
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import SetModePosition

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        # Declare parameters for node configuration
        self.declare_parameter('rate', 100)
        
        # Initialize parameters from the declared parameters
        rate = self.get_parameter('rate').get_parameter_value().integer_value
        
        # Initialize timer
        self.create_timer(1/rate, self.timer_callback)
        
        # ROS 2 Topic publisher setup
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10) # This publiseher is for publish random task space
        self.end_effector_pub = self.create_publisher(PoseStamped, '/end_effector', 10) # This publiseher is for publish end effector pose
        
        # ROS 2 Service clients setup
        self.create_service(SetModePosition, '/random_target', self.random_target_callback)
        self.create_service(SetModePosition, '/ik_target_display', self.ik_target_display_callback)
        
        # Initialize a variables
        self.target_values_display = [0.0, 0.0, 0.0]
        self.random_target_values = [0.0, 0.0, 0.0]
        self.joints_angle = [0.0001, 0.0001, 0.0001]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define the robot using Modified Denavit-Hartenberg parameters
        mdh = [[0.0, 0.0, 0.2, 0], [0.0, np.pi/2.0, 0.12, np.pi/2.0], [0.25, 0.0, -0.1, -np.pi/2.0]]
        revjoint = []
        
        # Create revolute joints based on MDH parameters
        for data in mdh:
            revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append each revolute joint
        
        # Create a DHRobot object representing a 3-DOF robot arm
        self.robot = rtb.DHRobot(
            [
                revjoint[0],
                revjoint[1],
                revjoint[2]
            ],
            tool = SE3.Rx(-np.pi/2) * SE3.Tz(0.28),
            name="3R robot"
        )
        
        self.get_logger().info("Environment node has been started")
    
    # Define function
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
        self.target_publish_func(self.target_values_display)
        self.eff_publish_func(self.endeffector_pose_compute())    
    
    def endeffector_pose_compute(self):
        try:    
            now = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform(
                'link_0',
                'end_effector',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            return trans.transform
            
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {str(e)}")
            
    def ik_target_display_callback(self, request, response):
        try:
            self.target_values_display[0] = request.position.x
            self.target_values_display[1] = request.position.y
            self.target_values_display[2] = request.position.z
            self.get_logger().info(f"Display target values are {self.target_values_display}")
            response.success = True
            return response
        except Exception as e:
            response.success = False
            self.get_logger().error(f"ik target display callback has {e}")
            return response
            
    def random_func(self, l1 = 0.2, l2 = 0.25, l3 = 0.28):
        while(True):
            x = random.uniform(-(l2+l3), (l2+l3))
            y = random.uniform(-(l2+l3), (l2+l3))
            z = random.uniform(-(l2+l3), (l1+l2+l3))
            if np.sqrt(x**2 + (z-l1)**2) < l2+l3 and np.sqrt(y**2 + (z-l1)**2) < l2+l3:
                return x, y, z
            
    def random_target_callback(self, request, response):
        try:
            self.random_target_values[0], self.random_target_values[1], self.random_target_values[2] = self.random_func()
            response.position.x = self.random_target_values[0]
            response.position.y = self.random_target_values[1]
            response.position.z = self.random_target_values[2]
            response.success = True
            self.get_logger().info(f"Success to random position. The random position is x:{self.random_target_values[0]}, y:{self.random_target_values[1]}, z:{self.random_target_values[2]}")
            return response
        except Exception as e:
            self.get_logger().info(f"Random target has {e}, Fail to random position.")
            response.success = False
            return response
    
def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
