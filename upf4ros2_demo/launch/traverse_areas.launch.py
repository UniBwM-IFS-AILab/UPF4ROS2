from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_name = "upf4ros2_demo"
    count = int(context.perform_substitution(LaunchConfiguration('count')))
    
    drone_count = count
    launch_array =  []
    
    for i in range(0,drone_count):
        plan_executor_node = Node(
            package=pkg_name,
            executable="plan_executor",
            name="plan_executor" + str(i),
            parameters=[
                {'drone_prefix': 'vhcl'+ str(i) +'/'}
            ],
            emulate_tty=True,
            output='screen')
        launch_array.append(plan_executor_node)
        
    return launch_array

def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument('count', default_value='3')] + [OpaqueFunction(function=launch_setup)]
    )
