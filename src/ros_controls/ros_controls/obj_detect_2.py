import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Depending on your ZED ROS2 wrapper version, this could be:
# from zed_interfaces.msg import ObjectsStamped
# Newer distributions (Humble/Jazzy) use:
from zed_msgs.msg import ObjectsStamped

class ZedSdkDetectionBridgeNode(Node):

    def __init__(self):
        super().__init__('zed_sdk_detection_bridge_node')

        # ?? Parameter Declarations ???????????????????????????????????????????
        self.declare_parameter('input_objects_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('input_image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('publish_image', True)

        # Retrieve values at startup
        objects_topic = self.get_parameter('input_objects_topic').value
        input_image_topic = self.get_parameter('input_image_topic').value
        self.pub_img = self.get_parameter('publish_image').value

        # Initialize lightweight helper attributes
        self.bridge = CvBridge()
        self.latest_frame = None

        # ? Best-Effort QoS Setup (Matches native ZED SDK publisher profile) ???
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for downstream visual servoing controllers
        self.det_pub = self.create_publisher(Detection2DArray, 'detections_2d', qos_profile)

        # Core Subscriber: Listen directly to ZED SDK's Object Detection data
        self.obj_sub = self.create_subscription(
            ObjectsStamped, objects_topic, self.objects_callback, qos_profile)

        # OPTIMIZATION: Only subscribe/publish image data if visualization is enabled
        if self.pub_img:
            self.img_sub = self.create_subscription(
                Image, input_image_topic, self.image_callback, qos_profile)
            self.img_pub = self.create_publisher(Image, 'detection_image', qos_profile)
            self.get_logger().info("Image visualization stream ENABLED.")
        else:
            self.get_logger().info("Image visualization stream DISABLED (Optimized Mode).")

        self.get_logger().info(f'Bridge running. Input stream: {objects_topic}')

    def image_callback(self, msg):
        try:
            # Continually update the latest background canvas frame
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Conversion Failed: {str(e)}')

    def objects_callback(self, msg):
        det2d_array = Detection2DArray()
        det2d_array.header = msg.header  

        # Cache the current image frame locally if canvas streaming is active
        frame = self.latest_frame.copy() if (self.pub_img and self.latest_frame is not None) else None

        # Parse every tracked element discovered natively by the ZED SDK
        for obj in msg.objects:
            # Verify 2D bounding box data exists inside the message frame
            if not hasattr(obj, 'bounding_box_2d') or not obj.bounding_box_2d.corners:
                continue

            # Extract corners directly
            corners = obj.bounding_box_2d.corners
            x_coords = [corner.kp for corner in corners]
            y_coords = [corner.kp for corner in corners]
            
            xmin, xmax = min(x_coords), max(x_coords)
            ymin, ymax = min(y_coords), max(y_coords)

            # High-performance coordinate property reductions
            width = float(xmax - xmin)
            height = float(ymax - ymin)
            center_x = float(xmin + xmax) / 2.0
            center_y = float(ymin + ymax) / 2.0

            # Construct ROS2 standard Detection Frame
            det2d = Detection2D()
            det2d.header = msg.header
            
            # Setup properties for 2D Image Plane control laws (IBVS)
            det2d.bbox = BoundingBox2D()
            det2d.bbox.center.position.x = center_x
            det2d.bbox.center.position.y = center_y
            det2d.bbox.size_x = width
            det2d.bbox.size_y = height

            # Build structural data holder for object details & 3D space measurements
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.label
            hyp.hypothesis.score = float(obj.confidence) / 100.0  # Normalize (0.0 to 1.0)
            
            # Inject native 3D Stereo spatial translations (meters relative to camera)
            hyp.pose.pose.position.x = float(obj.position) # Left/Right
            hyp.pose.pose.position.y = float(obj.position) # Up/Down
            hyp.pose.pose.position.z = float(obj.position) # Forward depth/Distance

            det2d.results.append(hyp)
            det2d_array.detections.append(det2d)

            # Paint UI tagging indicators if visualization canvas is requested
            if frame is not None:
                cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.putText(frame, f'{obj.label} {hyp.hypothesis.score:.2f}', 
                            (int(xmin), max(int(ymin) - 10, 15)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Broadcast the optimized vector arrays to downstream nodes
        self.det_pub.publish(det2d_array)
        
        # Stream out the debug canvas image if requested
        if frame is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = msg.header
                self.img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedSdkDetectionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()