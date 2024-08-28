import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "map_path",
            description="Map file path",
        ),
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_robot_node() -> list:
    """
    :rtype: list
    """
    map_path = LaunchConfiguration("map_path")
    is_sim = LaunchConfiguration('is_sim')
    package_name = 'auto_mapper'

    auto_mapper = Node(
        package=package_name,
        executable="auto_mapper",
        name="auto_mapper_cartographer",
        parameters=[
            {'use_sim_time': is_sim},
            {"map_path": map_path}
        ]
    )

    return [
        GroupAction(
            actions=[
                auto_mapper
            ]
        )
    ]
