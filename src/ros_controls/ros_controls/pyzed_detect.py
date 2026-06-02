import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

import pyzed.sl as sl


class ZedDirectDetectionNode(Node):

    def __init__(self):
        super().__init__('zed_direct_detection_node')

        # Parameters
        self.declare_parameter('serial_number', 38605411)        # front cam serial
        self.declare_parameter('publish_image', True)
        self.declare_parameter('confidence_threshold', 50.0)
        self.declare_parameter('detection_model', 'MULTI_CLASS_BOX_FAST')
        self.declare_parameter('frame_id', 'zed2i_front_left_camera_optical_frame')

        serial_number        = self.get_parameter('serial_number').value
        self.pub_img         = self.get_parameter('publish_image').value
        confidence_threshold = self.get_parameter('confidence_threshold').value
        detection_model_str  = self.get_parameter('detection_model').value
        self.frame_id        = self.get_parameter('frame_id').value

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.det_pub = self.create_publisher(Detection2DArray, 'detections_2d', qos)

        if self.pub_img:
            self.img_pub = self.create_publisher(Image, 'detection_image', qos)
            self.get_logger().info("Image visualization stream ENABLED.")

        # --- ZED SDK init ---
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.set_from_serial_number(serial_number)
        init_params.coordinate_units        = sl.UNIT.METER
        init_params.coordinate_system       = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_mode              = sl.DEPTH_MODE.PERFORMANCE
        init_params.camera_resolution       = sl.RESOLUTION.HD720

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f'Failed to open ZED camera (serial {serial_number}): {err}')
            raise RuntimeError(f'ZED open failed: {err}')

        self.get_logger().info(f'ZED camera opened (serial {serial_number})')

        # --- Object detection ---
        model_map = {
            'MULTI_CLASS_BOX_FAST':     sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
            'MULTI_CLASS_BOX_MEDIUM':   sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM,
            'MULTI_CLASS_BOX_ACCURATE': sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE,
        }
        model = model_map.get(detection_model_str,
                              sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST)

        od_params = sl.ObjectDetectionParameters()
        od_params.enable_tracking   = True
        od_params.detection_model   = model

        err = self.zed.enable_object_detection(od_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f'Failed to enable object detection: {err}')
            self.zed.close()
            raise RuntimeError(f'OD enable failed: {err}')

        self.get_logger().info(f'Object detection enabled (model: {detection_model_str})')

        # Runtime params
        self.rt_params = sl.RuntimeParameters()

        self.od_rt_params = sl.ObjectDetectionRuntimeParameters()
        self.od_rt_params.detection_confidence_threshold = confidence_threshold

        # ZED SDK Mat objects (reused each frame)
        self.image_zed = sl.Mat()
        self.objects   = sl.Objects()

        # Grab loop runs in background thread
        self._running = True
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        self.get_logger().info('Grab loop started.')

    # ------------------------------------------------------------------
    def _grab_loop(self):
        while self._running and rclpy.ok():
            err = self.zed.grab(self.rt_params)
            if err != sl.ERROR_CODE.SUCCESS:
                # Transient grab error ? skip frame
                continue

            # Retrieve left image (BGRA from SDK)
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)

            # Retrieve detected objects
            self.zed.retrieve_objects(self.objects, self.od_rt_params)

            # Convert image to numpy
            frame_bgra = self.image_zed.get_data()           # numpy array BGRA uint8
            frame      = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)

            # Build Detection2DArray
            now             = self.get_clock().now().to_msg()
            det2d_array     = Detection2DArray()
            det2d_array.header.stamp    = now
            det2d_array.header.frame_id = self.frame_id

            for obj in self.objects.object_list:
                bb2d = obj.bounding_box_2d          # list of 4 [x,y] corners
                if bb2d is None or len(bb2d) < 4:
                    continue

                x_coords = [pt[0] for pt in bb2d]
                y_coords = [pt[1] for pt in bb2d]

                xmin, xmax = float(min(x_coords)), float(max(x_coords))
                ymin, ymax = float(min(y_coords)), float(max(y_coords))

                width    = xmax - xmin
                height   = ymax - ymin
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0

                det2d        = Detection2D()
                det2d.header = det2d_array.header
                det2d.id     = str(obj.id)

                det2d.bbox                   = BoundingBox2D()
                det2d.bbox.center.position.x = center_x
                det2d.bbox.center.position.y = center_y
                det2d.bbox.size_x            = width
                det2d.bbox.size_y            = height

                hyp                      = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id  = str(obj.label)
                hyp.hypothesis.score     = float(obj.confidence) / 100.0

                pos = obj.position                  # [x, y, z] in meters
                hyp.pose.pose.position.x = float(pos[0])
                hyp.pose.pose.position.y = float(pos[1])
                hyp.pose.pose.position.z = float(pos[2])

                det2d.results.append(hyp)
                det2d_array.detections.append(det2d)

                # Draw on frame
                label_text = (f'{obj.label} '
                              f'{hyp.hypothesis.score:.2f} | '
                              f'{pos[2]:.1f}m')
                cv2.rectangle(frame,
                              (int(xmin), int(ymin)),
                              (int(xmax), int(ymax)),
                              (0, 255, 0), 2)
                cv2.putText(frame, label_text,
                            (int(xmin), max(int(ymin) - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.det_pub.publish(det2d_array)

            if self.pub_img:
                try:
                    img_msg              = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    img_msg.header.stamp    = now
                    img_msg.header.frame_id = self.frame_id
                    self.img_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish image: {e}')

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        self._grab_thread.join(timeout=2.0)
        self.zed.disable_object_detection()
        self.zed.close()
        self.get_logger().info('ZED camera closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedDirectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()