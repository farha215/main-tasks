import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class ZedImageViewer(Node):

    def __init__(self):
        super().__init__('zed_image_viewer')
        self.bridge = CvBridge()
        
        # Thread-safe storage for the incoming frames
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Drop old frames; visual servoing only cares about the newest frame
        )

        self.subscription = self.create_subscription(
            Image,
            '/detection_image',
            self.image_callback,
            qos
        )
        self.get_logger().info("ZED Image Receiver initialized.")

    def image_callback(self, msg):
        try:
            # Shift data to OpenCV format instantly and exit the callback immediately
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ZedImageViewer()

    # 1. Spin ROS 2 in a dedicated background thread.
    # This keeps communication pipelines free and completely independent of the GUI.
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    logger = node.get_logger()
    logger.info("OpenCV Window loop active on Main Thread. Press 'q' to close.")

    window_name = "ZED Camera POV - 2D Detections"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while rclpy.ok():
            current_frame = None
            
            # 2. Safely grab a shallow copy of the frame matrix without blocking the listener
            with node.frame_lock:
                if node.latest_frame is not None:
                    current_frame = node.latest_frame.copy()

            if current_frame is not None:
                cv2.imshow(window_name, current_frame)
            
            # 3. Process window events on the main thread. 
            # A 10ms wait loop keeps CPU usage minimal while maintaining ~100Hz responsiveness.
            if cv2.waitKey(10) & 0xFF == ord('q'):
                logger.info("Quit signal received via GUI window.")
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up window frames and drop parameters gracefully
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()