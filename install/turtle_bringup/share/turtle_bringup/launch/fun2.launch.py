import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Create launch_description object
    launch_description = LaunchDescription()
    
    
    # Create parameters launch file
    namespace1 = LaunchConfiguration('namespace1')
    namespace1_launch_arg = DeclareLaunchArgument(
        'namespace1',
        default_value = 'default'
    )
    launch_description.add_action( namespace1_launch_arg )
    
    namespace2 = LaunchConfiguration('namespace2')
    namespace2_launch_arg = DeclareLaunchArgument(
        'namespace2',
        default_value = 'default'
    )
    launch_description.add_action( namespace2_launch_arg )
    
    odom1 = LaunchConfiguration('odom1')
    odom1_launch_arg = DeclareLaunchArgument(
        'odom1',
        default_value = 'odom1'
    )
    launch_description.add_action( odom1_launch_arg )
    
    odom2 = LaunchConfiguration('odom2')
    odom2_launch_arg = DeclareLaunchArgument(
        'odom2',
        default_value = 'odom2'
    )
    launch_description.add_action( odom2_launch_arg )
    
    # Launch file configurations
    package_name = ['turtlesim_plus', 'turtle_bringup']
    executable_name = ['turtlesim_plus_node.py', 'controller.py', 'crazy_pizza.py', 'crazy_turtle.py', 'odom_publisher.py']
    namespace_list = [namespace1, namespace2]
    odom_list = [odom1, odom2]
    
    
    # Run turtlesim plus node
    turtlesim_plus = Node(
            package=package_name[0],
            namespace='',
            executable=executable_name[0],
            name='turtlesim_plus'
    )
    launch_description.add_action( turtlesim_plus )
    
    # Run controller node
    controller = Node(
            package=package_name[1],
            namespace='',
            executable=executable_name[1],
            name='controller',
            parameters=[
                {"namespace1":namespace1}
            ]
    )
    launch_description.add_action( controller )
    
    # Run crazy turtle node
    crazy_turtle = Node(
            package=package_name[1],
            namespace='',
            executable=executable_name[3],
            parameters=[
                {"namespace1":namespace2}
            ]
    )
    launch_description.add_action( crazy_turtle )
    
    # Run crazy pizza node
    crazy_pizza = Node(
            package=package_name[1],
            namespace='',
            executable=executable_name[2]
    )
    launch_description.add_action( crazy_pizza )
    
    # Run odometry publisher node
    for i in range (2):
        odom_publisher = Node(
                package=package_name[1],
                namespace='',
                executable=executable_name[4],
                parameters=[
                    {"namespace1":namespace_list[i]},
                    {"odom1":odom_list[i]}
                ]
        )
        launch_description.add_action( odom_publisher )
    
    # Get the path to the Rviz2 config file
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtle_bringup'),  # Replace with your package name
        'rvizconfig',
        'fun2.rviz'  # Replace with your RViz configuration file name
    )
    
    # Define the command to start Rviz2 with the configuration file
    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_dir],
        output='screen'
    )
    launch_description.add_action( start_rviz_cmd )
    
    return launch_description
