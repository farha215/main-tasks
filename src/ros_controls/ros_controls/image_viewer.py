import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ZedImageViewer(Node):

    def __init__(self):
        super().__init__('zed_image_viewer')
        self.bridge = CvBridge()

        # Match the exact QoS profile of the publisher for data compatibility
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the image topic containing the 2D bounding box drawings
        self.subscription = self.create_subscription(
            Image,
            '/detection_image',
            self.image_callback,
            qos
        )
        
        self.get_logger().info("ZED Image Viewer Node started. Press 'q' in the window to exit.")

    def image_callback(self, msg):
        self.get_logger().info("Image message received! Processing...")
        try:
            # Convert the incoming ROS 2 Image message to an OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Render the frame to a desktop window
            cv2.imshow("ZED Camera POV - 2D Detections", cv_image)
            
            # cv2.waitKey(1) is absolutely mandatory to refresh the OpenCV GUI frame
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quit signal received via keyboard.")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Failed to convert or display image: {str(e)}")

    def destroy_node(self):
        # Force close all OpenCV GUI windows on exit
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()