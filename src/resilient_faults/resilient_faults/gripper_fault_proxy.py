#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String


class GripperFaultProxy(Node):

    def __init__(self):

        super().__init__(
            'gripper_fault_proxy'
        )

        self.mode = 'normal'

        self.drop_deadline_ns = None

        self.currently_attached = False


        # -----------------------------------------------------
        # INPUT FROM MANIPULATION EXECUTOR
        # -----------------------------------------------------

        self.command_subscriber = self.create_subscription(
            Bool,
            '/faults/gripper_command_raw',
            self.gripper_callback,
            10
        )


        # -----------------------------------------------------
        # FAULT MODE
        # -----------------------------------------------------

        self.fault_subscriber = self.create_subscription(
            String,
            '/faults/active',
            self.fault_callback,
            10
        )


        # -----------------------------------------------------
        # OUTPUT TO REAL / VIRTUAL GRIPPER
        # -----------------------------------------------------

        self.gripper_publisher = self.create_publisher(
            Bool,
            '/virtual_gripper/command',
            10
        )


        # Timer checks whether an object-drop event is due.
        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )


        self.get_logger().info(
            'Gripper fault proxy started.'
        )


    # =========================================================
    # FAULT MODE
    # =========================================================

    def fault_callback(self, msg):

        self.mode = (
            msg.data.strip().lower()
        )


        # Cancel a previously scheduled drop if we leave
        # object_drop mode.

        if self.mode != 'object_drop':

            self.drop_deadline_ns = None


        self.get_logger().info(
            f'Gripper proxy mode: {self.mode}'
        )


    # =========================================================
    # GRIPPER COMMAND
    # =========================================================

    def gripper_callback(self, msg):

        requested_attach = msg.data


        # -----------------------------------------------------
        # MISSED GRASP
        #
        # Robot commands GRIP ON, but the gripper never
        # successfully captures the object.
        # -----------------------------------------------------

        if (
            self.mode == 'missed_grasp'
            and requested_attach
        ):

            failed_command = Bool()

            failed_command.data = False

            self.gripper_publisher.publish(
                failed_command
            )

            self.currently_attached = False


            self.get_logger().warn(
                'FAULT INJECTED: MISSED_GRASP'
            )

            return


        # -----------------------------------------------------
        # NORMAL FORWARDING
        # -----------------------------------------------------

        forwarded_command = Bool()

        forwarded_command.data = (
            requested_attach
        )

        self.gripper_publisher.publish(
            forwarded_command
        )

        self.currently_attached = (
            requested_attach
        )


        # -----------------------------------------------------
        # OBJECT DROP
        #
        # Grasp initially succeeds.
        #
        # Two seconds later the proxy sends GRIP OFF.
        # -----------------------------------------------------

        if (
            self.mode == 'object_drop'
            and requested_attach
        ):

            now_ns = (
                self.get_clock()
                .now()
                .nanoseconds
            )

            self.drop_deadline_ns = (
                now_ns
                + 2_000_000_000
            )


            self.get_logger().warn(
                'OBJECT_DROP armed: '
                'release in 2.0 seconds.'
            )


        if not requested_attach:

            self.drop_deadline_ns = None


    # =========================================================
    # OBJECT-DROP TIMER
    # =========================================================

    def timer_callback(self):

        if self.drop_deadline_ns is None:

            return


        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )


        if now_ns < self.drop_deadline_ns:

            return


        release_command = Bool()

        release_command.data = False


        self.gripper_publisher.publish(
            release_command
        )


        self.currently_attached = False

        self.drop_deadline_ns = None


        self.get_logger().warn(
            'FAULT INJECTED: OBJECT_DROP'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GripperFaultProxy()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
