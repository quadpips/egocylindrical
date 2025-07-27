import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():

    #################
    # Include Nodes #
    #################
    egocylindrical_propagator_node = Node(
        package="egocylindrical",
        executable="egocylindrical_propagator_node",
        name="egocylindrical_propagator_node",
        output="screen",
        parameters=[
            {
                'observation_sources': 'semantic_depth',
                'image_in': '/camera/depth/image_raw',
                'info_in': '/camera/depth/camera_info',
                'publish_update', True,
                'raytrace', True,
                'points_out': 'data',
                'filtered_points', 'filtered_points',
                'fixed_frame_id': 'odom',
                'orientation_fixed_frame_id' : 'base_aligned',
                'origin_fixed_frame_id': 'camera_depth_optical_frame',
            }
        ]
    )

    point_cloud_node = Node(
        package="egocylindrical",
        executable="point_cloud_node",
        name="point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
            ('cylindrical', 'points'),
        ],
    )

    projected_point_cloud_node = Node(
        package="egocylindrical",
        executable="projected_point_cloud_node",
        name="projected_point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
        ],
    )

    semantic_point_cloud_node = Node(
        package="egocylindrical",
        executable="semantic_point_cloud_node",
        name="semantic_point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
        ],
    )

    normal_point_cloud_node = Node(
        package="egocylindrical",
        executable="normal_point_cloud_node",
        name="normal_point_cloud_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
        ],
    )

    floor_image_node = Node(
        package="egocylindrical",
        executable="floor_image_node",
        name="floor_image_node",
        output="screen",
        remappings=[
            ('egocylindrical_points', 'data'),
            ('use_raw', 'False'),
            ('floor_image_topic', 'floor_image'),
            ('floor_labels_topic', 'floor_labels'),
            ('floor_labels_colored_topic', 'floor_labels_colored'),
            ('floor_normals_topic', 'floor_normals'),
            ('floor_labels_colored_topic', 'floor_labels_colored')
        ],
    )

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
        [
            egocylindrical_propagator_node,
            point_cloud_node,
            projected_point_cloud_node,
            semantic_point_cloud_node,
            normal_point_cloud_node,
            floor_image_node,
        ]
    )
