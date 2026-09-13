import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RedObjectDetector(Node):

    def __init__(self):

        super().__init__('red_object_detector')

        # -----------------------------------------------------
        # OpenCV / ROS bridge
        # -----------------------------------------------------

        self.bridge = CvBridge()

        # -----------------------------------------------------
        # Latest sensor information
        # -----------------------------------------------------

        self.depth_image = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -----------------------------------------------------
        # ROS subscribers
        # -----------------------------------------------------

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # -----------------------------------------------------
        # Object pose publisher
        # -----------------------------------------------------

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/perception/red_object_pose',
            10
        )

        # -----------------------------------------------------
        # TF broadcaster
        # -----------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Red object detector started.'
        )

    # =========================================================
    # CAMERA INFO
    # =========================================================

    def camera_info_callback(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # =========================================================
    # DEPTH
    # =========================================================

    def depth_callback(self, msg):

        try:

            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as error:

            self.get_logger().error(
                f'Depth conversion failed: {error}'
            )

    # =========================================================
    # RGB DETECTION
    # =========================================================

    def rgb_callback(self, msg):

        if self.depth_image is None:
            return

        if None in (
            self.fx,
            self.fy,
            self.cx,
            self.cy
        ):
            return

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as error:

            self.get_logger().error(
                f'RGB conversion failed: {error}'
            )

            return

        # -----------------------------------------------------
        # Convert RGB image into HSV colour space
        # -----------------------------------------------------

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # Red wraps around the HSV hue boundary.
        # Therefore two red ranges are required.

        lower_red_1 = np.array(
            [0, 100, 80]
        )

        upper_red_1 = np.array(
            [10, 255, 255]
        )

        lower_red_2 = np.array(
            [170, 100, 80]
        )

        upper_red_2 = np.array(
            [180, 255, 255]
        )

        mask_1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask_2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
        )

        mask = cv2.bitwise_or(
            mask_1,
            mask_2
        )

        # -----------------------------------------------------
        # Find red regions
        # -----------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:

            self.get_logger().debug(
                'No red object detected.'
            )

            return

        # Largest red area should be our target cube.

        contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(contour)

        # Reject tiny red noise.

        if area < 100:
            return

        moments = cv2.moments(contour)

        if moments['m00'] == 0:
            return

        u = int(
            moments['m10'] /
            moments['m00']
        )

        v = int(
            moments['m01'] /
            moments['m00']
        )

        # -----------------------------------------------------
        # Read depth around object centre
        # -----------------------------------------------------

        depth = self.get_depth(
            u,
            v
        )

        if depth is None:
            return

        # -----------------------------------------------------
        # Convert pixel coordinate -> 3D camera coordinate
        #
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        # -----------------------------------------------------

        x = (
            (u - self.cx)
            * depth
            / self.fx
        )

        y = (
            (v - self.cy)
            * depth
            / self.fy
        )

        z = depth

        self.publish_pose(
            msg,
            x,
            y,
            z
        )

    # =========================================================
    # DEPTH EXTRACTION
    # =========================================================

    def get_depth(self, u, v):

        height, width = (
            self.depth_image.shape[:2]
        )

        if not (
            0 <= u < width
            and
            0 <= v < height
        ):
            return None

        # Use small neighbourhood instead of one pixel.
        # This makes the reading more robust.

        radius = 2

        x_min = max(
            0,
            u - radius
        )

        x_max = min(
            width,
            u + radius + 1
        )

        y_min = max(
            0,
            v - radius
        )

        y_max = min(
            height,
            v + radius + 1
        )

        region = self.depth_image[
            y_min:y_max,
            x_min:x_max
        ]

        values = region[
            np.isfinite(region)
        ]

        values = values[
            values > 0
        ]

        if len(values) == 0:
            return None

        depth = float(
            np.median(values)
        )

        # Some cameras use millimetres.
        # Gazebo RGB-D normally publishes metres,
        # but this protects the node from 16-bit mm data.

        if depth > 20.0:
            depth /= 1000.0

        if (
            not math.isfinite(depth)
            or depth <= 0
        ):
            return None

        return depth

    # =========================================================
    # PUBLISH 3D OBJECT POSE
    # =========================================================

    def publish_pose(
        self,
        image_msg,
        x,
        y,
        z
    ):

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.header.frame_id = 'camera_optical_frame' 

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.w = 1.0

        self.pose_publisher.publish(
            pose
        )

        # -----------------------------------------------------
        # Broadcast detected object as TF frame
        # -----------------------------------------------------

        transform = TransformStamped()

        transform.header = pose.header

        transform.child_frame_id = (
            'detected_red_object_camera' )

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(
            transform
        )

        self.get_logger().info(
            (
                'RED OBJECT | '
                f'X={x:.3f} m '
                f'Y={y:.3f} m '
                f'Z={z:.3f} m'
            ),
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = RedObjectDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
