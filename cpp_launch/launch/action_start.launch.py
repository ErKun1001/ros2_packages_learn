#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_action',
            executable='my_acttion_rbt_exe',
            name='node_action_robot',
            output='screen'),
        Node(
            package='cpp_action',
            executable='my_acttion_ctrl_exe',
            name='node_action_ctrl',
            output='screen'),
    ])
