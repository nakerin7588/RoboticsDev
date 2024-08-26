from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    rate_mux = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value = '5.0'
    )
    launch_description.add_action( rate_launch_arg )
    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
    )
    launch_description.add_action( turtlesim_node )
    
    package_name = ''
    executable_name = ''
    namespace = ['linear', 'angular']
    rate = [10.0, 30.0]
    for name in namespace:
        noise_gen = Node(
            package='lab1',
            namespace=name,
            executable='noise_generator_script.py',
            name=name + '_noise',
            parameters=[
                {"rate":rate[0]}
            ]
        )
        launch_description.add_action( noise_gen )
     
    velo_mux = Node(
            package='lab1',
            namespace='',
            executable='velocity_mux_script.py',
            name='mux',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')
            ],
            parameters=[
                {'rate':rate_mux}
            ]
    )
    launch_description.add_action( velo_mux )
    
    return launch_description
