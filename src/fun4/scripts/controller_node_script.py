#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library
import numpy as np

# Additional msg srv
from sensor_msgs.msg import JointState

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parameters setup
        self.declare_parameter('rate', 10) # Timer interupt frequency (Hz)
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        
        # Publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber
        self.create_subscription(JointState, '/taskspace_target', self.target_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        # Variables
        self.target = np.array([0.0, 0.0, 0.0])
        self.current_joint_states = np.array([0.0, 0.0, 0.0])
        self.last_joint_states = np.array([0.0, 0.0, 0.0])
        self.trajectory_duration = 5.0  # Total duration of the trajectory in seconds
        self.num_steps = int(self.trajectory_duration * self.rate)  # Number of steps in the trajectory
        self.time_elapsed = 0.0  # Keep track of the time elapsed
        self.step_idx = 0  # Step counter
        self.joint_positions = np.array([0.0, 0.0, 0.0])
        
        # Initialize variables to track trajectory execution
        self.reset_trajectory()
        
    def timer_callback(self):
        """Publishes one step of the joint trajectory."""
        if self.last_joint_states.any != self.target.any:
            self.reset_trajectory()
        # Ensure we only run the trajectory for the defined duration
        if self.step_idx < self.num_steps:
            self.last_joint_states = self.target
            # Linearly interpolate between current and target joint positions
            alpha = self.step_idx / self.num_steps
            self.joint_positions = (1 - alpha) * self.current_joint_states + alpha * self.target
            
            # Increment step index
            self.step_idx += 1
            
            self.get_logger().info(f'{self.joint_positions}')
        else:
            # Trajectory completed, stop the timer
            self.get_logger().info('Trajectory completed.')
            self.last_joint_states = self.target
            
        # Publish the joint positions
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']  # Replace with your robot's joint names
        msg.position = self.joint_positions.tolist()

        self.joint_state_pub.publish(msg)
        
    def target_callback(self, msg):
        self.target = np.array(msg.position)
    
    def joint_states_callback(self, msg):
        self.current_joint_states = np.array(msg.position)
    
    def reset_trajectory(self):
        """Reset trajectory interpolation and compute number of steps."""
        self.time_elapsed = 0.0  # Track time elapsed
        self.step_idx = 0  # Step index for trajectory
        self.num_steps = int(self.trajectory_duration * self.rate)  # Number of steps
        self.timer.reset()
            
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
