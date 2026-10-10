#!/usr/bin/env python3

import csv
import os

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String


class FaultEventLogger(Node):

    def __init__(self):

        super().__init__('fault_event_logger')

        self.declare_parameter(
            'output_file',
            'results/day4_fault_events.csv'
        )

        self.output_file = (
            self.get_parameter('output_file')
            .get_parameter_value()
            .string_value
        )

        output_directory = os.path.dirname(
            self.output_file
        )

        if output_directory:

            os.makedirs(
                output_directory,
                exist_ok=True
            )

        file_exists = os.path.exists(
            self.output_file
        )

        self.csv_file = open(
            self.output_file,
            'a',
            newline=''
        )

        self.writer = csv.writer(
            self.csv_file
        )

        if not file_exists:

            self.writer.writerow([
                'ros_time_sec',
                'event_type',
                'value'
            ])

            self.csv_file.flush()


        self.last_fault_mode = None


        self.fault_subscription = (
            self.create_subscription(
                String,
                '/faults/active',
                self.fault_callback,
                10
            )
        )


        self.gripper_subscription = (
            self.create_subscription(
                Bool,
                '/virtual_gripper/command',
                self.gripper_callback,
                10
            )
        )


        self.get_logger().info(
            f'Fault event logger started: '
            f'{self.output_file}'
        )


    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )


    def write_event(
        self,
        event_type,
        value
    ):

        self.writer.writerow([
            f'{self.now_seconds():.6f}',
            event_type,
            value
        ])

        self.csv_file.flush()


    def fault_callback(
        self,
        msg
    ):

        mode = msg.data.strip()

        # /faults/active is published repeatedly,
        # so log only transitions.

        if mode == self.last_fault_mode:

            return

        self.last_fault_mode = mode

        self.write_event(
            'FAULT_MODE',
            mode
        )


        self.get_logger().info(
            f'FAULT EVENT: {mode}'
        )


    def gripper_callback(
        self,
        msg
    ):

        value = (
            'ATTACH'
            if msg.data
            else 'RELEASE'
        )

        self.write_event(
            'GRIPPER_COMMAND',
            value
        )


    def destroy_node(self):

        if not self.csv_file.closed:

            self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = FaultEventLogger()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

