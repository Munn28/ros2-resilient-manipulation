#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Pose

from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
)

from moveit_msgs.srv import ApplyPlanningScene

from shape_msgs.msg import SolidPrimitive


class PathBlocker(Node):

    def __init__(self):

        super().__init__('path_blocker')

        self.active = False

        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        self.subscription = self.create_subscription(
            String,
            '/faults/active',
            self.fault_callback,
            10
        )

        self.get_logger().info(
            'Path blocker ready.'
        )


    def fault_callback(self, msg):

        mode = msg.data.strip().lower()

        if mode == 'path_blocked':

            if not self.active:

                self.add_obstacle()

                self.active = True

        else:

            if self.active:

                self.remove_obstacle()

                self.active = False


    def add_obstacle(self):

        if not self.client.wait_for_service(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'MoveIt planning scene service unavailable.'
            )

            return


        obstacle = CollisionObject()

        obstacle.header.frame_id = 'base_link'

        obstacle.id = 'day4_path_blocker'


        box = SolidPrimitive()

        box.type = SolidPrimitive.BOX

        box.dimensions = [
            0.30,
            0.30,
            0.45,
        ]


        pose = Pose()

        # Enclose the target area so that the
        # transfer/place goal becomes invalid.

        pose.position.x = 0.42
        pose.position.y = 0.17
        pose.position.z = 0.58

        pose.orientation.w = 1.0


        obstacle.primitives.append(
            box
        )

        obstacle.primitive_poses.append(
            pose
        )

        obstacle.operation = (
            CollisionObject.ADD
        )


        scene = PlanningScene()

        scene.is_diff = True

        scene.world.collision_objects.append(
            obstacle
        )


        request = ApplyPlanningScene.Request()

        request.scene = scene


        self.client.call_async(
            request
        )


        self.get_logger().warn(
            'FAULT INJECTED: PATH_BLOCKED'
        )


    def remove_obstacle(self):

        obstacle = CollisionObject()

        obstacle.header.frame_id = 'base_link'

        obstacle.id = 'day4_path_blocker'

        obstacle.operation = (
            CollisionObject.REMOVE
        )


        scene = PlanningScene()

        scene.is_diff = True

        scene.world.collision_objects.append(
            obstacle
        )


        request = ApplyPlanningScene.Request()

        request.scene = scene


        self.client.call_async(
            request
        )


        self.get_logger().info(
            'PATH_BLOCKED obstacle removed.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PathBlocker()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
