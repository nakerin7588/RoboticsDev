#!/usr/bin/python3

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from lab1_interfaces.srv import SetNoise

class NoiseGeneratorNode(Node):
    def __init__(self):
        # Initialize super class node
        super().__init__('noise_generator_node')
        # Create publisher for topic/noise
        self.noise_publisher = self.create_publisher(Float64, 'noise', 10)
        # Set the publisher rate
        self.declare_parameter('rate', 5.0)
        self.rate = float(self.get_parameter('rate').value)
        # if len(sys.argv) >= 1:
        #     self.rate = float( sys.argv[1] )
        # else:
        #     self.rate = 5.0
        # Additional attributes
        self.mean = 0.0
        self.variance = 1.0
        # Create service server for /set_noise
        self.set_noise_server = self.create_service(SetNoise, 'set_noise', self.set_noise_callback)
        # Create timer for publish topic/noise
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        self.get_logger().info(f'Starting {self.get_namespace()} / {self.get_name()} with the default parameter. mean: {self.mean}, variance: {self.variance}')
        
    def set_noise_callback(self, request:SetNoise.Request, response:SetNoise.Response):
        self.mean = request.mean.data
        self.variance = request.variance.data
        return response # Service calback have to return response every time
    
    def timer_callback(self):
        msg = Float64()
        msg.data = np.random.normal(self.mean, np.sqrt(self.variance))
        self.noise_publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = NoiseGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
