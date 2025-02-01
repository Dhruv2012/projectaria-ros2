from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import (
    IfCondition,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
    
    
def launch_setup(context, *args, **kwargs):
    
    
    stream_aria_ros_node = Node(
        package='eve',
        executable='stream_aria_ros',
        name='cam_aria',
        output='screen'
    )
    
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', LaunchConfiguration('aloha_rvizconfig')
        ],
        condition=IfCondition(LaunchConfiguration('use_aloha_rviz')),
    )
    
    return [
        stream_aria_ros_node,
        rviz2_node,
    ]
    
    
def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_cameras',
            default_value='true',
            choices=('true', 'false'),
            description='if `true`, launches the camera drivers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cam_high_name',
            default_value='cam_high',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_aloha_rviz',
            default_value='false',
            choices=('true', 'false'),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'aloha_rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('eve'),
                'rviz',
                'aloha.rviz',
            ]),
        )
    )
    declared_arguments.append(Node(
            package='eve',
            executable='stream_aria_ros',
            name='cam_aria',
            output='screen'
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])