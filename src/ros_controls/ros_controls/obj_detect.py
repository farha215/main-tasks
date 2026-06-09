import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class Zed2DDetectionNode(Node):

    def __init__(self):
        super().__init__('zed_2d_detection_node')

        # ?? Parameters ????????????????????????????????????????????????????????
        self.declare_parameter('input_image_topic', '/zed2i_front/zed_node/left/image_rect_color')
        self.declare_parameter('model_path', '/home/cupcake/Prequal/src/ros_controls/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('publish_image', True)

        # Retrieve settings
        input_topic = self.get_parameter('input_image_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('confidence_threshold').value
        self.pub_img = self.get_parameter('publish_image').value

        self.bridge = CvBridge()
        
        self.get_logger().info(f'Loading inference model from: {model_path}')
        self.model = YOLO(model_path)

        # ?? Subscription & Publishers ?????????????????????????????????????????
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.img_sub = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile)

        self.det_pub = self.create_publisher(Detection2DArray, 'detections_2d', qos_profile)
        if self.pub_img:
            self.img_pub = self.create_publisher(Image, 'detection_image', qos_profile)

        self.get_logger().info(f'Node tracking input stream: {input_topic}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV BGR format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Conversion Failed: {str(e)}')
            return

        # Run inference frame processing (FIX 1: added to extract from the returned list)
        results = self.model(frame, conf=self.conf, verbose=False)

        det2d_array = Detection2DArray()
        det2d_array.header = msg.header  

        # Process found bounding boxes
        if results.boxes is not None:
            for box in results.boxes:
                # FIX 2: Target the internal bounding array layer
                xyxy = box.xyxy.cpu().numpy()
                
                # FIX 3: Individually assign unique corner scalar boundaries
                xmin = float(xyxy)
                ymin = float(xyxy)
                xmax = float(xyxy)
                ymax = float(xyxy)
                
                width = xmax - xmin
                height = ymax - ymin
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0

                # FIX 4: Use .item() to safely extract scalar values out of tensors
                cls_id = int(box.cls.item())
                label_str = self.model.names[cls_id]
                conf_score = float(box.conf.item())

                # Build ROS 2 Detection Frame
                det2d = Detection2D()
                det2d.header = msg.header
                det2d.bbox = BoundingBox2D()
                det2d.bbox.center.position.x = center_x
                det2d.bbox.center.position.y = center_y
                det2d.bbox.size_x = width
                det2d.bbox.size_y = height

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label_str
                hyp.hypothesis.score = conf_score
                det2d.results.append(hyp)
                det2d_array.detections.append(det2d)

                # Annotate image frame canvas if requested
                if self.pub_img:
                    cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                    cv2.putText(frame, f'{label_str} {conf_score:.2f}', 
                                (int(xmin), max(int(ymin) - 10, 15)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish data streams
        self.det_pub.publish(det2d_array)
        if self.pub_img:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = msg.header
            self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Zed2DDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()