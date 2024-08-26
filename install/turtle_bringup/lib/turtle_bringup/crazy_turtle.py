#!/usr/bin/python3

from my_packages.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Twist, Point, TransformStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

from turtlesim.srv import Spawn


import numpy as np
import math


class CrazyTurtleNode(Node):
    def __init__(self):
        super().__init__('crazy_turtle_node')
        self.create_timer(0.01, self.timer_callback) # timer
        
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza') #service
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10) #publisher odom
        self.tf_boardcaster = TransformBroadcaster(self) #transform
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10) #publisher
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10) #subscriber
        
        self.create_subscription(Pose, '/crazy_pizza', self.crazy_pizza_callback, 10) #subscriber from crazy pizza
        
        self.spawn_turtle_client = self.create_client(Spawn, '/spawn_turtle') #service
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza') #service
        self.eat_pizza_client = self.create_client(Empty, '/turtle2/eat') #service
        
        self.robot_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        self.mouse_pose = np.array([0.0, 0.0, 0.0]) #x, y, z
        self.pizza_pose = np.array([0.0, 0.0]) #x, y
        self.crazy_pizza_pose = np.array([0.0, 0.0]) #x, y
        self.Queue = [[0.0, 0.0]] #Queue x, y  
        
        self.spawn_turtle(5.5, 5.5, 1.0, '/turtle2') #x, y, theta, name
        
    def crazy_pizza_callback(self, msg):
        self.crazy_pizza_pose[0] = msg.x
        self.crazy_pizza_pose[1] = msg.y
        print(self.crazy_pizza_pose)
        
        position_request = GivePosition.Request()
        position_request.x = self.crazy_pizza_pose[0]
        position_request.y = self.crazy_pizza_pose[1]
        self.spawn_pizza_client.call_async(position_request)
        
        self.Queue.append([self.crazy_pizza_pose[0], self.crazy_pizza_pose[1]])
        
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
        
        
    def spawn_turtle(self, x, y, theta, name):
        position_request = Spawn.Request()
        position_request.x = x
        position_request.y = y
        position_request.theta = theta
        position_request.name = name
        self.spawn_turtle_client.call_async(position_request)
        
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)
    
    #------------------------------------------------------timer----------------------------------------------------------------    
    def timer_callback(self):
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
    node = CrazyTurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
