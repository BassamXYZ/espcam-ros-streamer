#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import quaternion_from_matrix

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('marker_size', 0.10) # 10 cm
        self.declare_parameter('dictionary_name', 'DICT_5X5_50')
        
        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('dictionary_name').value

        # Select ArUco Dictionary
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        }
        
        selected_dict = aruco_dicts.get(dict_name, cv2.aruco.DICT_5X5_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(selected_dict)
        
        # OpenCV ArUco Detector Setup (Compatible with standard OpenCV versions)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.legacy_opencv = False
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.legacy_opencv = True

        if not self.legacy_opencv:
            self.aruco_params = cv2.aruco.DetectorParameters()
            # Relax parameters for webcams / ESP32-CAM streams
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.polygonalApproxAccuracyRate = 0.05
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.polygonalApproxAccuracyRate = 0.05
            
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.pub_poses = self.create_publisher(PoseArray, '/aruco_poses', 10)
        self.pub_markers_img = self.create_publisher(Image, '/aruco_image', 10)

        self.get_logger().info('ArUco Detector Node initialized successfully.')

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        # Convert image safely
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Enhance contrast for lower-resolution ESP32-CAM feeds
        gray = cv2.equalizeHist(gray)

        # Detect markers
        if not self.legacy_opencv:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # Log directly if any raw corners are found
        if ids is not None:
            self.get_logger().info(f"Detected Marker IDs: {ids.ravel().tolist()}")

        pose_array = PoseArray()
        pose_array.header = msg.header  # Same frame_id (camera_link_optical) and timestamp

        if ids is not None and len(ids) > 0:
            # Estimate Pose for each detected marker
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Draw axes on image for visual verification in RViz
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size * 0.5)

                pose = Pose()
                # Translation
                pose.position.x = float(tvec[0][0][0])
                pose.position.y = float(tvec[0][0][1])
                pose.position.z = float(tvec[0][0][2])

                # Rotation Matrix to Quaternion
                rmat, _ = cv2.Rodrigues(rvec[0][0])
                T = np.eye(4)
                T[:3, :3] = rmat
                q = quaternion_from_matrix(T)

                pose.orientation.x = float(q[0])
                pose.orientation.y = float(q[1])
                pose.orientation.z = float(q[2])
                pose.orientation.w = float(q[3])

                pose_array.poses.append(pose)

        # Publish results
        self.pub_poses.publish(pose_array)
        
        # Publish processed image for debugging
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_markers_img.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
