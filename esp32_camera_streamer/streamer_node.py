import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import requests
import numpy as np

class ESP32CameraStreamer(Node):
    def __init__(self):
        super().__init__('esp32_camera_streamer')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.stream_url = 'http://192.168.43.20:81/stream'
        self.bridge = CvBridge()

        # Start a persistent HTTP session that reads the MJPEG stream
        self.session = requests.Session()
        try:
            self.stream = self.session.get(self.stream_url, stream=True, timeout=5)
            if self.stream.status_code != 200:
                self.get_logger().error(f"Stream returned status {self.stream.status_code}")
                return
        except Exception as e:
            self.get_logger().error(f"Could not connect to stream: {e}")
            return

        self.bytes_buffer = b''
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

    def timer_callback(self):
        try:
            # Read raw data from the stream until we have a complete JPEG frame
            self.bytes_buffer += self.stream.raw.read(4096)
            # JPEG frames in MJPEG stream start with 0xFF 0xD8 and end with 0xFF 0xD9
            start = self.bytes_buffer.find(b'\xff\xd8')
            end = self.bytes_buffer.find(b'\xff\xd9')
            if start != -1 and end != -1:
                jpg_data = self.bytes_buffer[start:end+2]
                self.bytes_buffer = self.bytes_buffer[end+2:]  # Remove processed frame
                # Decode JPEG to OpenCV image
                np_arr = np.frombuffer(jpg_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    img_msg.header.frame_id = "camera_link"
                    self.publisher_.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Error reading frame: {e}")
            # Attempt to reconnect if stream breaks
            try:
                self.stream = self.session.get(self.stream_url, stream=True, timeout=5)
            except:
                pass

    def destroy_node(self):
        if hasattr(self, 'stream'):
            self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
