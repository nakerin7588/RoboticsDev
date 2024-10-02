#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
import numpy as np 

# Additional msg srv
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class DifferencetialKinematicNode(Node):
    def __init__(self):
        super().__init__('differencetial_kinematic_node')
        
        # Parameters setup
        self.declare_parameter('rate', 1) # Timer interupt frequency (Hz)
        rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        timer_ = self.create_timer(1/rate, self.timer_callback)
        
        # Topic Publisher variables
    
        # Topic Subscriber variables
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joints_state_callback, 10)
        self.create_subscription(PoseStamped, '/end_effector', self.eff_current_pos_callback, 10)
        
        # Service Server variables
        # mode_select_server = self.create_service()
        
        # Service Client variables
        
        # Variables
        self.joints_angle = [0.0001, 0.0001, 0.0001]
        self.eff_current_position = [[0.0001, 0.0001, 0.0001], [0.0001, 0.0001, 0.0001, 0.0001]] # xyz, wxyz
        
    
        # Start up
        # Create robot model
        mdh = [[0.0, 0.0, 0.2, 0], [0.0, pi/2.0, 0.12, pi/2.0], [0.25, 0.0, -0.1, -pi/2.0]]
        revjoint = [] # Create revolute joint
        for data in mdh:
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
        
    def timer_callback(self):
        pass
    
    def joints_state_callback(self, msg):
        self.joints_angle = msg.position
    
    def cmd_callback(self, msg):
        linear_velo = [msg.linear.x, msg.linear.y, msg.linear.z]
        
        if True:
            return
    
    def eff_current_pos_callback(self, msg):
        self.eff_current_position[0][0] = msg.pose.position.x
        self.eff_current_position[0][1] = msg.pose.position.y
        self.eff_current_position[0][3] = msg.pose.position.z
        self.eff_current_position[1][0] = msg.pose.orientation.w
        self.eff_current_position[1][1] = msg.pose.orientation.x
        self.eff_current_position[1][2] = msg.pose.orientation.y
        self.eff_current_position[1][3] = msg.pose.orientation.z
    
    def compute_q_dot_ref_eff(self, joint_angles, ee_velocity, manipulability_threshold=0.05):
            J = self.robot.jacob0(self.joints_angle) # Calculate the Jacobian matrix at the current joint configuration
            J = J[0:3, :] # Reduce jacobian. Use only translation
            w = self.manipulability(J)
            if w < manipulability_threshold:
                self.get_logger().warn((f"Low manipulability ({w}). Restricting motion."))
            J_pseudo_inv = np.linalg.pinv(J) # Compute the standard Jacobian pseudo-inverse
            joint_velocities = J_pseudo_inv @ ee_velocity
            adjusted_joint_velocities = self.limit_motion_based_on_manipulability(joint_angles, joint_velocities, w, manipulability_threshold) # Limit the motion if near a singularity
            return adjusted_joint_velocities, w
    
    def compute_q_dot_ref_world(self, joint_angles, ee_velocity, manipulability_threshold=0.05):
            J = self.robot.jacob0(self.joints_angle) # Calculate the Jacobian matrix at the current joint configuration
            J = J[0:3, :] # Reduce jacobian. Use only translation
            w = self.manipulability(J)
            if w < manipulability_threshold:
                self.get_logger().warn((f"Low manipulability ({w}). Restricting motion."))
            J_pseudo_inv = np.linalg.pinv(J) # Compute the standard Jacobian pseudo-inverse
            joint_velocities = J_pseudo_inv @ ee_velocity
            adjusted_joint_velocities = self.limit_motion_based_on_manipulability(joint_angles, joint_velocities, w, manipulability_threshold) # Limit the motion if near a singularity
            return adjusted_joint_velocities, w
            
                
    def manipulability(self, J):
        # Manipulability measure based on the determinant of the translational part of J
        JJ_T = J @ J.T
        manipulability_measure = np.sqrt(np.linalg.det(JJ_T))
        return manipulability_measure
    
    def limit_motion_based_on_manipulability(self, joint_angles, joint_velocities, manipulability_value, manipulability_threshold):
        if manipulability_value < manipulability_threshold:
            # If manipulability is too low, adjust the velocities to prevent moving closer to the singularity
            scaling_factor = manipulability_value / manipulability_threshold
            self.get_logger().warn((f"Near singularity! Reducing velocities by factor: {scaling_factor}"))
            adjusted_joint_velocities = scaling_factor * joint_velocities
        else:
            # If manipulability is sufficient, use the original velocities
            adjusted_joint_velocities = joint_velocities

        return adjusted_joint_velocities
    
def main(args=None):
    rclpy.init(args=args)
    node = DifferencetialKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
