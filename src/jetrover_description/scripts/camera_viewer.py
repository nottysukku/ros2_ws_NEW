import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Camera Viewer Node Initialized')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize to actual camera resolution if needed
            cv_image = cv2.resize(cv_image, (1920, 1080))
            # Display the image
            cv2.imshow('Overhead Camera Feed (1920x1080)', cv_image)
            # Wait for key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutting down viewer...')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error displaying image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
