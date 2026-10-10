#!/usr/bin/env python3

import copy

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class FaultInjector(Node):

    VALID_MODES = {
        'normal',
        'object_missing',
        'stale_pose',
        'pose_offset',
        'unreachable_pose',
	'missed_grasp',
	'object_drop',
	'path_blocked',
	'heartbeat_loss',
    }


    def __init__(self):

        super().__init__('fault_injector')

        # -----------------------------------------------------
        # Current fault state
        # -----------------------------------------------------

        self.mode = 'normal'

        self.last_pose = None

        self.stale_pose = None

        self.heartbeat_counter = 0

        self.heartbeat_publisher = self.create_publisher( String, '/faults/heartbeat', 10 )
        # -----------------------------------------------------
        # INPUT
        #
        # This is the REAL perception result from Day 2.
        # -----------------------------------------------------

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/perception/red_object_pose_world',
            self.pose_callback,
            10
        )


        # -----------------------------------------------------
        # FAULT COMMAND
        #
        # Example:
        #
        # /faults/command = "stale_pose"
        # -----------------------------------------------------

        self.command_subscriber = self.create_subscription(
            String,
            '/faults/command',
            self.command_callback,
            10
        )


        # -----------------------------------------------------
        # OUTPUT
        #
        # Manipulation will eventually consume this instead of
        # the raw perception topic.
        # -----------------------------------------------------

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/faults/red_object_pose',
            10
        )


        # -----------------------------------------------------
        # Publish currently active fault mode.
        # -----------------------------------------------------

        self.state_publisher = self.create_publisher(
            String,
            '/faults/active',
            10
        )


        # Used mainly for stale-pose publishing.
        self.timer = self.create_timer(
            0.10,
            self.timer_callback
        )


        self.publish_state()


        self.get_logger().info(
            'Fault injector started.'
        )

        self.get_logger().info(
            'Initial mode: NORMAL'
        )


    # =========================================================
    # COMMAND PROCESSING
    # =========================================================

    def command_callback(self, msg):

        requested_mode = (
            msg.data.strip().lower()
        )


        if requested_mode not in self.VALID_MODES:

            self.get_logger().error(
                f'Unknown fault mode: {requested_mode}'
            )

            self.get_logger().info(
                'Valid modes: '
                + ', '.join(sorted(self.VALID_MODES))
            )

            return


        self.mode = requested_mode


        # Capture the last known valid pose at the moment
        # stale-pose injection begins.

        if self.mode == 'stale_pose':

            if self.last_pose is not None:

                self.stale_pose = copy.deepcopy(
                    self.last_pose
                )

            else:

                self.stale_pose = None


        else:

            self.stale_pose = None


        self.get_logger().warn(
            f'FAULT MODE CHANGED → {self.mode.upper()}'
        )


        self.publish_state()


    # =========================================================
    # REAL PERCEPTION CALLBACK
    # =========================================================

    def pose_callback(self, msg):

        self.last_pose = copy.deepcopy(
            msg
        )


        # -----------------------------------------------------
        # NORMAL
        #
        # Perfect pass-through.
        # -----------------------------------------------------

        if self.mode == 'normal':

            self.pose_publisher.publish(
                msg
            )

            return


        # -----------------------------------------------------
        # OBJECT MISSING
        #
        # Completely suppress perception output.
        # -----------------------------------------------------

        if self.mode == 'object_missing':

            return


        # -----------------------------------------------------
        # STALE POSE
        #
        # Do not use new measurements.
        # timer_callback() republishes the frozen old pose.
        # -----------------------------------------------------

        if self.mode == 'stale_pose':

            return


        # -----------------------------------------------------
        # INCORRECT POSE
        #
        # Deliberately bias an otherwise valid measurement.
        # -----------------------------------------------------

        if self.mode == 'pose_offset':

            corrupted_pose = copy.deepcopy(
                msg
            )

            corrupted_pose.pose.position.x += 0.20

            corrupted_pose.pose.position.y += 0.10

            corrupted_pose.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            self.pose_publisher.publish(
                corrupted_pose
            )

            return


        # -----------------------------------------------------
        # UNREACHABLE POSE
        #
        # Deliberately generate a pose outside the practical
        # UR5e working envelope.
        # -----------------------------------------------------

        if self.mode == 'unreachable_pose':

            corrupted_pose = copy.deepcopy(
                msg
            )

            corrupted_pose.pose.position.x = 1.25

            corrupted_pose.pose.position.y = 0.00

            corrupted_pose.pose.position.z = 1.00

            corrupted_pose.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            self.pose_publisher.publish(
                corrupted_pose
            )

            return

        # -----------------------------------------------------
        # NON-PERCEPTION FAULTS
        #
        # These faults affect another subsystem, therefore
        # perception must continue normally.
        # -----------------------------------------------------

        if self.mode in {
            'missed_grasp',
            'object_drop',
            'path_blocked',
            'heartbeat_loss',
        }:

            self.pose_publisher.publish(
                msg
            )

            return

    # =========================================================
    # TIMER
    #
    # Used to repeatedly publish a genuinely stale measurement.
    #
    # Notice that we DO NOT update its timestamp.
    # =========================================================

    def timer_callback(self):

        if self.mode != 'heartbeat_loss':

            heartbeat = String()

            heartbeat.data = (
               f'heartbeat_{self.heartbeat_counter}'
            )

            self.heartbeat_publisher.publish(
               heartbeat
            )

            self.heartbeat_counter += 1
        # Always republish the current fault state.
        # This makes /faults/active behave like a heartbeat/state topic.
        self.publish_state()

        # Only stale_pose needs repeated frozen pose output.
        if self.mode != 'stale_pose':
            return

        if self.stale_pose is None:
            return

        self.pose_publisher.publish(
            self.stale_pose
        )

    # =========================================================
    # STATE
    # =========================================================

    def publish_state(self):

        msg = String()

        msg.data = self.mode

        self.state_publisher.publish(
            msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = FaultInjector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
