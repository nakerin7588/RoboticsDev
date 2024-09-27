#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library
from math import pi
from spatialmath import *
import numpy as np
from roboticstoolbox import trapezoidal_func, trapezoidal

# Additional msg srv
from sensor_msgs.msg import JointState

class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_node')
        
        # Parameters setup
        self.declare_parameter('rate', 1) # Timer interupt frequency (Hz)
        rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        timer_ = self.create_timer(1/rate, self.timer_callback)

        # Topic Subscriber variables
        config_sub = self.create_subscription(JointState, '/configuration_space', self.config_callback, 10)

        # Variables
        self.joint_name = ["joint_1", "joint_2", "joint_3"]
        self.q_target = []
        
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # [q, q_dot, q_dot_dot] = trapezoidal_func()
        for i in range(len(self.q)):
            msg.position.append(self.q[i])
            msg.name.append(self.name[i])
        self.joint_pub.publish(msg)
    
    def config_callback(self, msg):
        self.q_target = []
        for q in msg:
            self.q_target.append(q.position)    
        
    
def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
