import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped

class ArucoTransformer(Node):
    def __init__(self):
        super().__init__('aruco_transformer')
        
        # الاشتراك في موضوع الـ ArUco الافتراضي
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_callback,
            10)
            
        # إنشاء الناشر الجديد الذي تحتاجه حزمة robot_localization
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/aruco/pose_covariance',
            10)
        
        self.get_logger().info('Aruco Transformer Node has been started.')

    def aruco_callback(self, msg):
        # التأكد من أن الكاميرا ترى علامة واحدة على الأقل
        if len(msg.poses) > 0:
            pose_covariance_msg = PoseWithCovarianceStamped()
            
            # نسخ الـ Header (الذي يحتوي على الوقت والـ frame_id مثل camera_link أو odom)
            pose_covariance_msg.header = msg.header
            
            # أخذ أول علامة مرئية في المصفوفة
            pose_covariance_msg.pose.pose = msg.poses[0]
            
            # تعيين مصفوفة التباين (Covariance) - قيم صغيرة تعني ثقة عالية بالـ ArUco
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
