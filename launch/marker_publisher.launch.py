from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'),

        Node(
            package='auto_mapper', 
            executable='marker_publisher', 
            name='marker_publisher_node',
            output='screen', 
            parameters=[{'use_sim_time':use_sim_time}]
        ),
    ])