#!/usr/bin/python3

# from my_packages.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

import numpy as np
import math


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.create_timer(0.01, self.timer_callback) # timer
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10) #publisher odom
        self.tf_boardcaster = TransformBroadcaster(self) #transform
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) #publisher
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10) #subscriber
        self.create_subscription(Point, '/mouse_position', self.mouse_callback, 10) #subscriber
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10) #subscriber
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza') #service
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat') #service
        
        self.robot_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        self.goal_pose = np.array([0.0, 0.0]) #x, y
        
        self.mouse_pose = np.array([0.0, 0.0, 0.0]) #x, y, z
        self.pizza_pose = np.array([0.0, 0.0]) #x, y
        self.Queue = [[0.0, 0.0]] #Queue x, y  
        
    def goal_callback(self, msg):
        self.goal_pose[0] = msg.pose.position.x
        self.goal_pose[1] = msg.pose.position.y
        # print(self.goal_pose)
        self.spawn_pizza(self.goal_pose[0], self.goal_pose[1]) #spawn pizza on pose when click mouse
        
    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x =  v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
        
    def pose_callback(self, msg):
        # print(msg)
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta
        # print(self.robot_pose)
        
    def mouse_callback(self, msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y
        self.mouse_pose[2] = msg.z
        # print(self.mouse_pose)
        
        self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1]) #spawn pizza on pose when click mouse
    
    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = self.pizza_pose[0] = x
        position_request.y = self.pizza_pose[1] = y
        self.Queue.append([self.pizza_pose[0], self.pizza_pose[1]])
        self.spawn_pizza_client.call_async(position_request)
        print(self.Queue)
        
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)
    
    #------------------------------------------------------timer----------------------------------------------------------------    
    def timer_callback(self):
        # self.cmdvel(0.0, 1.0)

        if len(self.Queue) == 0:
            det_x = 0
            det_y = 0
        if len(self.Queue) >= 1:
            det_x  =  self.Queue[0][0] - self.robot_pose[0]
            det_y  =  self.Queue[0][1] - self.robot_pose[1]
        
        d = math.sqrt((det_x * det_x) + (det_y * det_y))
        
        alfa = math.atan2(det_y, det_x)
            
        e = alfa - self.robot_pose[2]
        
        # kp_d = 30
        # kp_t = 66
        
        kp_d = 25
        kp_t = 55
        
        if d > 0.1:
            # Wrap around
            if e > math.pi:
                e -= 2*math.pi
            elif e < -math.pi:
                e += 2*math.pi

            v = kp_d* d
            w = kp_t * e
        else :
            v = 0.0
            w = 0.0
            self.eat_pizza()
            if len(self.Queue) > 0:
                self.Queue.pop(0)

        
        self.cmdvel(v, w)
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
