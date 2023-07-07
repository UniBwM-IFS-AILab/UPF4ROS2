from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "upf4ros2_demo"
    
    drone_count = 3
    ld = LaunchDescription()
    for i in range(0,drone_count):
        plan_executor_node = Node(
            package=pkg_name,
            executable="plan_executor",
            name="plan_executor" + str(i),
            parameters=[
                {'drone_prefix': 'vhcl'+ str(i) +'/'},
            ],
            emulate_tty=True,
            output='screen')
        ld.add_action(plan_executor_node)

    return ld
    """
    plan_executor_node = Node(
        package=pkg_name,
        executable="plan_executor",
        output='screen')

    ld = LaunchDescription()
    ld.add_action(plan_executor_node)
    return ld"""

