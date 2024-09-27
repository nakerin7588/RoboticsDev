#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library

# Additional msg srv

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parameters setup
        self.declare_parameter('rate', 1) # Timer interupt frequency (Hz)
        rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        timer_ = self.create_timer(1/rate, self.timer_callback)
        
    def timer_callback(self):
        pass
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
