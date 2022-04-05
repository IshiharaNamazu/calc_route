import os
import yaml

import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

robot_ns = "R2"


def generate_launch_description():

    route_calcrater = Node(
        package='calc_route',
        executable='calc_route'
    )

    visualizer = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", "src/calc_route/launch/rvizconfig.rviz"]
    )

    list = [
        route_calcrater,
        visualizer
    ]

    # else:
    #     list.append(Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         namespace=robot_ns,
    #         arguments = ['0.0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    #     ))

    # localization_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     namespace=robot_ns,
    #     arguments = ['0.1', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    # )
    # list.append(localization_node)

    return launch.LaunchDescription(list)


# def getTopsStateComponent():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')
#     urdf = os.path.join(
#         get_package_share_directory('r2_robot_state'),
#         'urdf',
#         'r2.urdf')

#     with open(config_path, 'r') as f:
#         params = yaml.safe_load(f)['r2_bt_planner']['ros__parameters']
#     component = ComposableNode(
#         package='r2_bt_planner',
#         plugin='abu2022::R2BtPlanner',
#         name='r2_bt_planner',
#         namespace=robot_ns,
#         parameters=[params]
#     )
#     return component
