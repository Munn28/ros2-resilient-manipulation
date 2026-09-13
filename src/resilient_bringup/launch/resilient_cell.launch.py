import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ---------------------------------------------------------
    # Locate our package
    # ---------------------------------------------------------

    bringup_share = get_package_share_directory(
        'resilient_bringup'
    )


    # ---------------------------------------------------------
    # Locate our Gazebo world
    # ---------------------------------------------------------

    world_file = os.path.join(
        bringup_share,
        'worlds',
        'resilient_cell.sdf'
    )


    # ---------------------------------------------------------
    # Locate the official Universal Robots simulator
    # ---------------------------------------------------------

    ur_sim_share = get_package_share_directory(
        'ur_simulation_gz'
    )


    ur_moveit_launch = os.path.join(
        ur_sim_share,
        'launch',
        'ur_sim_moveit.launch.py'
    )


    # ---------------------------------------------------------
    # Launch official UR simulation using OUR world
    # ---------------------------------------------------------

    ur_simulation = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            ur_moveit_launch
        ),

        launch_arguments={

            'ur_type': 'ur5e',

            'world_file': world_file,

        }.items(),

    )


    return LaunchDescription([

        ur_simulation

    ])
