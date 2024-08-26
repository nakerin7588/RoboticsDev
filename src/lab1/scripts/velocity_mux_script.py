#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class VelocityMuxNode(Node):
    def __init__(self):
        super().__init__('velocity_mux_node')
        # Create subscriber for topic /linear/noise
        self.linear_vel_subscriber = self.create_subscription(Float64, '/linear/noise', self.linear_vel_sub_callback, 10)
        # Create subscriber for topic /angular/noise
        self.linear_vel_subscriber = self.create_subscription(Float64, '/angular/noise', self.angular_vel_sub_callback, 10)
        # Create publisher for topic /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('rate', 5.0)
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        
        # if(len(sys.argv) >= 1):
        #     self.rate = float(sys.argv[1])
        # else: 
        #     self.rate = 5.0
        # Additional attributes
        self.cmd_vel = Twist()
        # Start timer for publish /cmd_vel
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        self.get_logger().info(f'Starting {self.get_name()}')
        
    # Callback: Set the x-component of linear velocity
    def linear_vel_sub_callback(self, msg:Float64):
        self.cmd_vel.linear.x = msg.data
    
    # Callback: Set the z-component of angular velocity
    def angular_vel_sub_callback(self, msg:Float64): 
        self.cmd_vel.angular.z = msg.data
    
    # Timer callback for publish cmd_vel
    def timer_callback(self):
        self.cmd_vel_publisher.publish(self.cmd_vel)
        
def main(args=None):
    rclpy.init(args=args)
    node = VelocityMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
