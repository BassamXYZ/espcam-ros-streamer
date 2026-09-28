#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf2_ros


# ---------- small self-contained pose <-> 4x4 matrix helpers ----------
# (no extra ROS/py deps needed beyond numpy, which is already a transitive
#  dependency of almost every ROS2 python package)

def to_matrix(translation, rotation):
    """translation: obj with .x .y .z / rotation: obj with .x .y .z .w"""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-9:
        R = np.eye(3)
    else:
        s = 2.0 / n
        R = np.array([
            [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
            [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
        ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [translation.x, translation.y, translation.z]
    return M


def transform_to_matrix(t):
    """geometry_msgs/TransformStamped -> 4x4 matrix"""
    return to_matrix(t.transform.translation, t.transform.rotation)


def pose_to_matrix(p):
    """geometry_msgs/Pose -> 4x4 matrix"""
    return to_matrix(p.position, p.orientation)


def invert(M):
    R = M[:3, :3]
    t = M[:3, 3]
    Minv = np.eye(4)
    Minv[:3, :3] = R.T
    Minv[:3, 3] = -R.T @ t
    return Minv


def matrix_to_translation_quaternion(M):
    R = M[:3, :3]
    t = M[:3, 3]
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return (t[0], t[1], t[2]), (x, y, z, w)


class ArucoTransformer(Node):
    def __init__(self):
        super().__init__('aruco_transformer')

        # الإطار الذي سننشر فيه قياس البوز (يجب أن يطابق pose0 في ekf.yaml)
        self.declare_parameter('output_frame', 'odom')
        self.output_frame = self.get_parameter('output_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/aruco/pose_covariance',
            10)

        self.get_logger().info(
            'Aruco Transformer Node initialized (computing map-relative pose '
            'from static marker positions instead of a live TF lookup).'
        )

    def aruco_callback(self, msg):
        if len(msg.marker_ids) == 0 or len(msg.poses) == 0:
            return

        marker_id = msg.marker_ids[0]
        detected_pose = msg.poses[0]          # pose of the marker AS SEEN BY the camera
        camera_frame = msg.header.frame_id    # frame the detection is expressed in
        marker_frame = f'marker_frame_{marker_id}'

        if not camera_frame:
            self.get_logger().warn('ArucoMarkers message has an empty header.frame_id, skipping.')
            return

        # 1) map -> marker_frame_X : معروف وثابت (static_transform_publisher)
        # 2) base_link -> camera_frame : معروف وثابت (من الـ URDF عبر robot_state_publisher)
        # هذان الاثنان فقط قابلان للاستعلام عبر TF لأنهما ضمن أشجار متصلة فعليًا.
        # الترانسفورم من marker_frame الى odom غير موجود اطلاقاً بشكل مباشر,
        # وهذا هو سبب فشل الكود القديم دائماً.
        try:
            map_to_marker = self.tf_buffer.lookup_transform(
                'map', marker_frame, rclpy.time.Time(), Duration(seconds=1.0))
            baselink_to_camera = self.tf_buffer.lookup_transform(
                'base_link', camera_frame, rclpy.time.Time(), Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Static TF not available yet ({e}), skipping this detection.')
            return

        T_map_marker = transform_to_matrix(map_to_marker)
        T_bl_cam = transform_to_matrix(baselink_to_camera)
        T_cam_marker = pose_to_matrix(detected_pose)  # marker pose expressed in camera frame

        # map -> camera = map -> marker * marker -> camera
        T_map_cam = T_map_marker @ invert(T_cam_marker)
        # map -> base_link = map -> camera * camera -> base_link
        T_map_bl = T_map_cam @ invert(T_bl_cam)

        translation, quat = matrix_to_translation_quaternion(T_map_bl)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.output_frame  # 'odom' (see note in main_launch.py)
        pose_msg.pose.pose.position.x = float(translation[0])
        pose_msg.pose.pose.position.y = float(translation[1])
        pose_msg.pose.pose.position.z = float(translation[2])
        pose_msg.pose.pose.orientation.x = float(quat[0])
        pose_msg.pose.pose.orientation.y = float(quat[1])
        pose_msg.pose.pose.orientation.z = float(quat[2])
        pose_msg.pose.pose.orientation.w = float(quat[3])

        pose_msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.09, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.09, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.09
        ]

        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
