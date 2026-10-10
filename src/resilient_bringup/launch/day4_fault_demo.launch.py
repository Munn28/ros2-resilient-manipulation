import os

from ament_index_python.packages import (
    get_package_share_directory,
)

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

    bringup_share = (
        get_package_share_directory(
            'resilient_bringup'
        )
    )


    cell_launch = os.path.join(
        bringup_share,
        'launch',
        'resilient_cell.launch.py'
    )


    # Try to identify workspace root from the
    # package installation directory.
    workspace_root = os.path.abspath(
        os.path.join(
            bringup_share,
            '..',
            '..',
            '..',
            '..'
        )
    )


    log_file = os.path.join(
        workspace_root,
        'results',
        'day4_fault_events.csv'
    )


    # =========================================================
    # BASE ROBOT CELL
    # =========================================================

    cell = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            cell_launch
        )
    )


    # =========================================================
    # FAULT INJECTOR
    # =========================================================

    fault_injector = Node(

        package='resilient_faults',

        executable='fault_injector',

        name='fault_injector',

        parameters=[
            {
                'use_sim_time': True
            }
        ],

        output='screen',
    )


    # =========================================================
    # MOVEIT PATH BLOCKER
    # =========================================================

    path_blocker = Node(

        package='resilient_faults',

        executable='path_blocker',

        name='path_blocker',

        parameters=[
            {
                'use_sim_time': True
            }
        ],

        output='screen',
    )


    # =========================================================
    # EVENT LOGGER
    # =========================================================

    event_logger = Node(

        package='resilient_faults',

        executable='fault_event_logger',

        name='fault_event_logger',

        parameters=[
            {
                'use_sim_time': True,
                'output_file': log_file,
            }
        ],

        output='screen',
    )


    # =========================================================
    # MOVEIT COLLISION SCENE
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
    # NOTE
    #
    # We intentionally DO NOT automatically start the
    # manipulation task here.
    #
    # Day 4 is an experimental fault-injection environment.
    #
    # The operator selects the desired fault first and can
    # then run the executor through the faulted perception
    # topic.
    # =========================================================


    return LaunchDescription([

        cell,

        fault_injector,

        path_blocker,

        event_logger,

        planning_scene,

    ])
