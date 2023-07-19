from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "upf4ros2_demo"

    #
    # NODES
    #

    drone_count = 2
    upf4ros2_demo_cmd = Node(
        package=pkg_name,
        executable="manager",
        name="mission_manager",
        output='screen')

    ld = LaunchDescription()
    ld.add_action(upf4ros2_demo_cmd)

    return ld
