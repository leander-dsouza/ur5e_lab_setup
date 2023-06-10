#! /usr/bin/env python
"""
Spawn Robot in RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """
    # .................. Configurable Arguments .....................

    gui = True

    gazebo_enabled = False
    rviz_config = 'urdf.rviz'

    # ...............................................................


    pkg_dir = get_package_share_directory('mit_robot_description')

    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument("gazebo_enabled", \
            default_value=str(gazebo_enabled), \
                description="Gazebo has started"),

        DeclareLaunchArgument("rviz_config", \
            default_value=rviz_config, \
                description="RViz Config"),

        # Nodes
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(PythonExpression( \
                ["'", LaunchConfiguration('gui'), "' == 'True' and '", \
                    LaunchConfiguration('gazebo_enabled'), "' == 'False'"]))
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(PythonExpression( \
                ["'", LaunchConfiguration('gui'), "' == 'False' and '", \
                    LaunchConfiguration('gazebo_enabled'), "' == 'False'"]))
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command( \
                ['xacro ', os.path.join(pkg_dir, 'urdf/mit_robot.urdf.xacro'),
                ])}],
            condition=UnlessCondition(LaunchConfiguration('gazebo_enabled'))
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', [os.path.join(pkg_dir, 'rviz/'), \
                LaunchConfiguration("rviz_config")]],
        ),

    ])
