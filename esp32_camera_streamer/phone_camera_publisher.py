#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
import requests
from ament_index_python.packages import get_package_share_directory
import numpy as np

class PhoneCameraPublisher(Node):
    def __init__(self):
        super().__init__('phone_camera_publisher')
        
        # المعاملات: رابط الفيديو من الهاتف واسم الموضوع
        self.declare_parameter('video_url', 'http://192.168.43.1:8080/video')
        self.declare_parameter('output_topic', '/camera/image_raw')
        
        url = self.get_parameter('video_url').get_parameter_value().string_value
        topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.bridge = CvBridge()

        self.camera_info_msg = CameraInfo()
        self.load_camera_info()

        self.get_logger().info(f'Connecting to phone camera at: {url}')
        self.cap = cv2.VideoCapture(url)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video stream from phone!')
            return

        # مؤقت لقراءة الإطارات بمعدل ~30 إطار/ثانية
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
    
    def load_camera_info(self):
        try:
            package_share_directory = get_package_share_directory('esp32_camera_streamer')
            yaml_path = os.path.join(package_share_directory, 'config', 'ost.yaml')
            
            with open(yaml_path, 'r') as file:
                calib_data = yaml.safe_load(file)
                
            self.camera_info_msg.width = calib_data['image_width']
            self.camera_info_msg.height = calib_data['image_height']
            self.camera_info_msg.k = [float(x) for x in calib_data['camera_matrix']['data']]
            self.camera_info_msg.d = [float(x) for x in calib_data['distortion_coefficients']['data']]
            self.camera_info_msg.r = [float(x) for x in calib_data['rectification_matrix']['data']]
            self.camera_info_msg.p = [float(x) for x in calib_data['projection_matrix']['data']]
            self.camera_info_msg.distortion_model = calib_data['distortion_model']
            
            self.get_logger().info("Camera calibration parameters loaded successfully!")
        except Exception as e:
            self.get_logger().warn(f"Could not load camera calibration file: {e}")
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            msg = self.bridge.cv2_to_imgmsg(rotated_frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link_optical'
            self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_msg.header.frame_id = "camera_link_optical"
            self.publisher_.publish(msg)
            self.info_pub.publish(self.camera_info_msg)
        else:
            self.get_logger().warn('Failed to capture frame from stream')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PhoneCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
