from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_name = "upf2anafi"
    count = int(context.perform_substitution(LaunchConfiguration('count')))
    
    drone_count = count
    launch_array =  []
    
    for i in range(0,drone_count):
        anafi_interface_node = Node(
            package=pkg_name,
            executable="anafi_interface",
            name="anafi_interface" + str(i),
            parameters=[
                {'drone_prefix': 'vhcl'+ str(i) +'/'}
            ],
            emulate_tty=True,
            output='screen')
        launch_array.append(anafi_interface_node)
        
    return launch_array

def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument('count', default_value='3')] + [OpaqueFunction(function=launch_setup)]
    )
