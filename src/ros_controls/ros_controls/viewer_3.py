import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading


class ZedImageViewer(Node):
    def __init__(self):
        super().__init__('zed_image_viewer')
        self.bridge = CvBridge()

        # Thread-safe storage for the incoming frames
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.declare_parameter('input_image_topic', '/detection_image')
        input_image_topic = self.get_parameter('input_image_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Drop old frames; only the newest matters
        )
        self.subscription = self.create_subscription(
            Image,
            input_image_topic,
            self.image_callback,
            qos
        )
        self.get_logger().info("ZED Image Receiver initialized.")

    def image_callback(self, msg):
        try:
            self.get_logger().info('Frame received!')
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Publish raw frame immediately even if no detections yet
            if self.pub_img:
                self.img_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedImageViewer()

    # Spin ROS 2 in a dedicated background thread so the GUI runs on the main thread.
    # daemon=True ensures the thread is killed automatically when the process exits,
    # even if rclpy.shutdown() is never reached (e.g. on an unhandled exception).
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    logger = node.get_logger()
    logger.info("OpenCV window loop active on main thread. Press 'q' to close.")

    input_image_topic = node.get_parameter('input_image_topic').value
    window_name = f"ZED Camera POV - {input_image_topic}"

    # FIX 1: Use WINDOW_NORMAL instead of WINDOW_AUTOSIZE.
    # WINDOW_AUTOSIZE locks the window to the exact pixel dimensions of the first
    # frame and ignores all subsequent resize requests. On many desktop environments
    # (especially Wayland and remote sessions) this causes the window to appear
    # invisible or a 1x1 stub until a frame actually arrives. WINDOW_NORMAL lets the
    # window manager create a properly sized, resizable window immediately.
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # FIX 2: Show a placeholder frame before any data arrives.
    # Without this, imshow is never called until the first message lands, so the
    # window exists in the taskbar but is blank/unresponsive. waitKey() still
    # processes events, but on some platforms (macOS, some Linux WMs) a window that
    # has never received an imshow call does not paint at all.
    placeholder = __import__('numpy').zeros((480, 640, 3), dtype='uint8')
    cv2.putText(placeholder, f"Waiting for {input_image_topic} ...",    
                (60, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)
    cv2.imshow(window_name, placeholder)

    try:
        while rclpy.ok():
            current_frame = None

            # Safely grab a shallow copy without blocking the subscriber thread
            with node.frame_lock:
                if node.latest_frame is not None:
                    current_frame = node.latest_frame.copy()

            if current_frame is not None:
                cv2.imshow(window_name, current_frame)

            # FIX 3: Guard waitKey() with an explicit window-existence check.
            # If the user closes the window via the OS close button (the X), the
            # window is destroyed but the loop keeps running. waitKey() returns -1
            # forever, 'q' is never seen, and the node hangs. getWindowProperty()
            # returns -1.0 when the window no longer exists, so we treat that as a
            # quit signal identical to pressing 'q'.
            key = cv2.waitKey(10) & 0xFF
            window_alive = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1.0
            if key == ord('q') or not window_alive:
                logger.info("Quit signal received ? shutting down.")
                break

    except KeyboardInterrupt:
        pass

    finally:
        # FIX 4: Call rclpy.shutdown() before node.destroy_node(), not after.
        # destroy_node() releases the node's handles; if the background spin() thread
        # is still mid-callback when destroy_node() runs it can dereference a freed
        # executor context and segfault. Calling shutdown() first signals the executor
        # to stop, giving the daemon thread a clean exit before handles are torn down.
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()