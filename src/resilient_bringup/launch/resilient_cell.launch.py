import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    bringup_share = get_package_share_directory(
        'resilient_bringup'
    )

    ur_sim_share = get_package_share_directory(
        'ur_simulation_gz'
    )

    # =========================================================
    # PROJECT FILES
    # =========================================================

    world_file = os.path.join(
        bringup_share,
        'worlds',
        'resilient_cell.sdf'
    )

    bridge_config = os.path.join(
        bringup_share,
        'config',
        'rgbd_bridge.yaml'
    )

    ur_moveit_launch = os.path.join(
        ur_sim_share,
        'launch',
        'ur_sim_moveit.launch.py'
    )

    # =========================================================
    # UR5e + GAZEBO + MOVEIT
    # =========================================================

    ur_simulation = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            ur_moveit_launch
        ),

        launch_arguments={
            'ur_type': 'ur5e',
            'world_file': world_file,
        }.items(),
    )

    # =========================================================
    # GAZEBO -> ROS RGB-D BRIDGE
    # =========================================================

    rgbd_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        name='rgbd_bridge',

        parameters=[
            {
                'config_file': bridge_config
            }
        ],

        output='screen',
    )

    # =========================================================
    # CAMERA CALIBRATION TRANSFORM
    #
    # world -> camera_optical_frame
    #
    # Camera position:
    #   x = 0.65
    #   y = 0.00
    #   z = 1.65
    #
    # Optical axes:
    #   +X camera -> -Y world
    #   +Y camera -> -X world
    #   +Z camera -> -Z world
    # =========================================================

    camera_static_tf = Node(

        package='tf2_ros',

        executable='static_transform_publisher',

        name='camera_static_tf',

        arguments=[
            '--x', '0.42',
            '--y', '0.0',
            '--z', '1.15',

            '--qx', '0.7071068',
            '--qy', '-0.7071068',
            '--qz', '0.0',
            '--qw', '0.0',

            '--frame-id', 'world',

            '--child-frame-id',
            'camera_optical_frame',
        ],

        output='screen',
    )

    # =========================================================
    # PERCEPTION
    # =========================================================

    red_object_detector = Node(

        package='resilient_perception',

        executable='red_object_detector',

        name='red_object_detector',

        output='screen',
    )

    # =========================================================
    # CAMERA POSE -> WORLD POSE
    # =========================================================

    object_pose_transformer = Node(

        package='resilient_perception',

        executable='object_pose_transformer',

        name='object_pose_transformer',

        output='screen',
    )

    # =========================================================
    # GAZEBO ENTITY POSE SERVICE
    # Used by the virtual gripper.
    # =========================================================

    set_pose_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        name='set_pose_bridge',

        arguments=[
            '/world/resilient_cell/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose'
        ],

        output='screen',
    )


    # =========================================================
    # SIMULATION VIRTUAL GRIPPER
    # =========================================================

    virtual_gripper = Node(

        package='resilient_manipulation',

        executable='virtual_gripper',

        name='virtual_gripper',

        parameters=[
            {
                'use_sim_time': True
            }
        ],

        output='screen',
    )

    # =========================================================
    # DAY 4 GRIPPER FAULT PROXY
    # =========================================================

    gripper_fault_proxy = Node(

        package='resilient_faults',

        executable='gripper_fault_proxy',

        name='gripper_fault_proxy',

        parameters=[
            {
                'use_sim_time': True
            }
        ],

        output='screen',
    )

    # =========================================================
    # COMPLETE SYSTEM
    # =========================================================

    return LaunchDescription([

        ur_simulation,

        rgbd_bridge,

	set_pose_bridge,

        camera_static_tf,

        red_object_detector,

        object_pose_transformer,

        gripper_fault_proxy,

	virtual_gripper,
    ])
