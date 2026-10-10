import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PATHS
    # =========================================================

    bringup_share = get_package_share_directory(
        'resilient_bringup'
    )

    resilient_cell_launch = os.path.join(
        bringup_share,
        'launch',
        'resilient_cell.launch.py'
    )


    # =========================================================
    # 1. START COMPLETE ROBOTIC CELL
    #
    # This already starts:
    # - Gazebo
    # - UR5e
    # - ros2_control
    # - MoveIt
    # - RViz
    # - RGB-D camera
    # - perception
    # - TF
    # - virtual gripper
    # =========================================================

    cell = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            resilient_cell_launch
        )
    )


    # =========================================================
    # 2. INITIALISE MOVEIT COLLISION SCENE
    #
    # Wait for Gazebo + MoveIt to finish starting.
    # =========================================================

    planning_scene = TimerAction(

        period=15.0,

        actions=[

            Node(
                package='resilient_manipulation',

                executable='planning_scene_initializer',

                name='planning_scene_initializer',

                parameters=[
                    {
                        'use_sim_time': True
                    }
                ],

                output='screen',
            )
        ]
    )


    # =========================================================
    # 3. START AUTONOMOUS PICK-AND-PLACE
    #
    # Give the planning scene a few more seconds to settle.
    # =========================================================

    manipulation_task = TimerAction(

        period=20.0,

        actions=[

            Node(
                package='resilient_manipulation',

                executable='vision_motion_executor',

                name='vision_motion_executor',

                parameters=[
                    {
                        'use_sim_time': True
                    }
                ],

                output='screen',
            )
        ]
    )


    # =========================================================
    # COMPLETE DAY-3 SYSTEM
    # =========================================================

    return LaunchDescription([

        cell,

        planning_scene,

        manipulation_task,

    ])
