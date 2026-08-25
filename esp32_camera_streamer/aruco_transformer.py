import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped

class ArucoTransformer(Node):
    def __init__(self):
        super().__init__('aruco_transformer')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_callback,
            10)
            
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/aruco/pose_covariance',
            10)
        
        self.get_logger().info('Aruco Transformer Node has been started.')

    def aruco_callback(self, msg):
        if len(msg.poses) > 0:
            pose_covariance_msg = PoseWithCovarianceStamped()
            
            marker_id = 0
            pose_covariance_msg.header = msg.header
            pose_covariance_msg.header.stamp = self.get_clock().now().to_msg()
            pose_covariance_msg.header.frame_id = f'marker_frame_{marker_id}'
            
            pose_covariance_msg.pose.pose = msg.poses[0]
            pose_covariance_msg.pose.covariance = [
                0.01, 0.0,  0.0,  0.0,  0.0,  0.0,  # X
                0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  # Y
                0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  # Z
                0.0,  0.0,  0.0,  0.09, 0.0,  0.0,  # Roll
                0.0,  0.0,  0.0,  0.0,  0.09, 0.0,  # Pitch
                0.0,  0.0,  0.0,  0.0,  0.0,  0.09  # Yaw
            ]
            
            self.publisher.publish(pose_covariance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
