import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener

from tf2_geometry_msgs import do_transform_pose


class ObjectPoseTransformer(Node):

    def __init__(self):

        super().__init__('object_pose_transformer')

        # -----------------------------------------------------
        # TF2
        # -----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # -----------------------------------------------------
        # Input:
        # object pose measured by the RGB-D camera
        # -----------------------------------------------------

        self.subscription = self.create_subscription(
            PoseStamped,
            '/perception/red_object_pose',
            self.pose_callback,
            10
        )

        # -----------------------------------------------------
        # Output:
        # same object expressed in world coordinates
        # -----------------------------------------------------

        self.world_pose_publisher = self.create_publisher(
            PoseStamped,
            '/perception/red_object_pose_world',
            10
        )

        self.get_logger().info(
            'Object pose transformer started.'
        )

    def pose_callback(self, camera_pose):

        try:

            # Get latest known transform:
            #
            # camera_optical_frame -> world

            transform = self.tf_buffer.lookup_transform(
                'world',
                camera_pose.header.frame_id,
                Time()
            )

        except TransformException as error:

            self.get_logger().warn(
                f'TF transform unavailable: {error}'
            )

            return

        # -----------------------------------------------------
        # Transform the detected pose
        # -----------------------------------------------------

        world_pose_data = do_transform_pose(
            camera_pose.pose,
            transform
        )

        world_pose = PoseStamped()

        world_pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        world_pose.header.frame_id = 'world'

        world_pose.pose = world_pose_data

        self.world_pose_publisher.publish(
            world_pose
        )

        # -----------------------------------------------------
        # Publish object as a TF frame in world coordinates
        # -----------------------------------------------------

        object_tf = TransformStamped()

        object_tf.header = world_pose.header

        object_tf.child_frame_id = (
            'detected_red_object'
        )

        object_tf.transform.translation.x = (
            world_pose.pose.position.x
        )

        object_tf.transform.translation.y = (
            world_pose.pose.position.y
        )

        object_tf.transform.translation.z = (
            world_pose.pose.position.z
        )

        object_tf.transform.rotation = (
            world_pose.pose.orientation
        )

        self.tf_broadcaster.sendTransform(
            object_tf
        )

        self.get_logger().info(
            (
                'WORLD RED OBJECT | '
                f'X={world_pose.pose.position.x:.3f} m '
                f'Y={world_pose.pose.position.y:.3f} m '
                f'Z={world_pose.pose.position.z:.3f} m'
            ),
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = ObjectPoseTransformer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
