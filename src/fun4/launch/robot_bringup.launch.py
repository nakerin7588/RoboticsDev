#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('fun4')
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    environment_node = Node(
        package='fun4',
        executable='environment_node_script.py'
        # name='environment_node'
    )
    
    scheduler_node = Node(
        package='fun4',
        executable='scheduler_node_script.py'
        # name='environment_node'
    )
    
    inverse_kinematic_node = Node(
        package='fun4',
        executable='inverse_kinematic_node_script.py'
        # name='environment_node'
    )
    
    differencetial_kinematic_node = Node(
        package='fun4',
        executable='differencetial_kinematic_node_script.py'
        # name='environment_node'
    )
    
    launch_description = LaunchDescription()
    
    node = [rviz, robot_state_publisher, environment_node, scheduler_node, inverse_kinematic_node, differencetial_kinematic_node]
    for i in node:
        launch_description.add_action(i)
    
    return launch_description