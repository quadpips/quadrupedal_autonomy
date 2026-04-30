import os
import sys

import launch
import launch_ros.actions
from launch.actions import (DeclareLaunchArgument, ExecuteProcess)
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    go2_description_path = get_package_share_directory("go2_description")
    go2_interface_path = get_package_share_directory("go2_interface")
    quad_pips_path = get_package_share_directory("quad_pips")

    ####################
    # Launch Arguments #
    ####################
    urdfFile = os.path.join(go2_description_path, "urdf/go2_description.urdf")
    frameFile = os.path.join(go2_interface_path, "config/frame_declaration.info")

    taskFile = os.path.join(quad_pips_path, "config/mpc/task.info")
    sqpFile = os.path.join(quad_pips_path, "config/mpc/sqp.info")
    controllerConfigFile = os.path.join(quad_pips_path, "config/wbc/wbc.info")

    use_force_sensor = False
    contact_threshold = 0
    rviz = "True"

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
        ),        
        launch_ros.actions.Node(
            package='go2_interface',
            executable='go2_gazebo_main',
            name='go2_gazebo_main',
            output='log',
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    'taskFile': taskFile,
                    'frameFile': frameFile,
                    'sqpFile': sqpFile,
                    'controllerConfigFile': controllerConfigFile,
                    'urdfFile': urdfFile,
                    'use_force_sensor': use_force_sensor,
                    'contact_threshold': contact_threshold,
                }
            ]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
            arguments=[
                "-d",
                os.path.join(
                    go2_interface_path, "rviz", "go2_quadpips.rviz",
                )
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_base_link_tf2",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "base",
                "base_link"
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        )      
    ])

    return ld



if __name__ == '__main__':
    generate_launch_description()