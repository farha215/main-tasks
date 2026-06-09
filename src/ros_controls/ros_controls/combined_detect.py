import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from collections import defaultdict
from ultralytics import YOLO
import os

from zed_msgs.msg import ObjectsStamped
from auv_msgs.msg import Detection, DetectionArray

# ?? HSV tuning for pole ???????????????????????????????????????????????????????
LOWER_HSV        = np.array([5,   0,   0])
UPPER_HSV        = np.array([138, 173, 169])
MIN_CONTOUR_AREA = 500

# ?? YOLO tuning ???????????????????????????????????????????????????????????????
GATE_CONF_THRESHOLD = 0.6
# ?????????????????????????????????????????????????????????????????????????????


class UnifiedDetectionNode(Node):
    def __init__(self):
        super().__init__('unified_detection_node')

        self.cv2_bridge  = CvBridge()
        self.imu_pose    = None
        self.zed_objects = []

        # Depth smoothing per detection id
        self.depth_history = defaultdict(list)
        self.HISTORY_SIZE  = 10

        # Load YOLO model
        #model_path = # Just point it directly to the source folder for tonight's testing
        model_path = '/home/cupcake/Prequal/src/ros_controls/ros_controls/best.onnx'
        self.yolo = YOLO(model_path, task='detect')
        self.get_logger().info(f'YOLO model loaded from {model_path}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.detect_img_pub  = self.create_publisher(Image,             '/zed2i_front/detection_image', qos)
        self.detection_pub   = self.create_publisher(DetectionArray,    '/zed2i_front/detection_msg',   qos)
        self.detection3d_pub = self.create_publisher(Detection3DArray,  '/detections_3d',               qos)

        # IMU
        self.imu_sub = self.create_subscription(
            Imu, '/zed2i_front/zed_node/imu/data', self.imu_callback, qos)

        # ZED SDK objects for 3D position lookup
        self.objects_sub = self.create_subscription(
            ObjectsStamped, '/zed2i_front/zed_node/obj_det/objects',
            self.objects_callback, qos)

        # Sync RGB + depth
        image_sub = message_filters.Subscriber(
            self, Image, '/zed2i_front/zed_node/rgb/color/rect/image',
            qos_profile=qos)
        depth_sub = message_filters.Subscriber(
            self, Image, '/zed2i_front/zed_node/depth/depth_registered',
            qos_profile=qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info('UnifiedDetectionNode started.')

    # ?? Async callbacks ???????????????????????????????????????????????????????

    def imu_callback(self, msg):
        self.imu_pose = msg.orientation

    def objects_callback(self, msg):
        self.zed_objects = msg.objects

    # ?? HSV detection ?????????????????????????????????????????????????????????

    def get_hsv_bboxes(self, frame_bgr):
        hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bboxes = []
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            bboxes.append((x, y, x + w, y + h))
        return bboxes

    # ?? YOLO detection ????????????????????????????????????????????????????????

    def get_yolo_bboxes(self, frame_bgr):
        """Returns list of (x1, y1, x2, y2, confidence, class_name)"""
        results = self.yolo(frame_bgr, verbose=False)
        detections = []
        if len(results) > 0:
            for box in results[0].boxes:
                conf     = float(box.conf[0].cpu().numpy())
                cls      = int(box.cls[0].cpu().numpy())
                cls_name = self.yolo.names.get(cls, str(cls))
                if conf < GATE_CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detections.append((int(x1), int(y1), int(x2), int(y2), conf, cls_name))
        return detections

    # ?? Depth helpers ?????????????????????????????????????????????????????????

    def get_depth_from_depthmap(self, depth_np, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        roi   = depth_np[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def find_matching_zed_object(self, bbox):
        x1, y1, x2, y2 = bbox
        hsv_cx = (x1 + x2) / 2.0
        hsv_cy = (y1 + y2) / 2.0

        best_obj  = None
        best_dist = float('inf')

        for obj in self.zed_objects:
            if not hasattr(obj, 'bounding_box_2d') or not obj.bounding_box_2d.corners:
                continue
            corners  = obj.bounding_box_2d.corners
            x_coords = [c.kp[0] for c in corners]
            y_coords = [c.kp[1] for c in corners]
            zed_cx   = (min(x_coords) + max(x_coords)) / 2.0
            zed_cy   = (min(y_coords) + max(y_coords)) / 2.0

            if x1 <= zed_cx <= x2 and y1 <= zed_cy <= y2:
                dist = np.hypot(zed_cx - hsv_cx, zed_cy - hsv_cy)
                if dist < best_dist:
                    best_dist = dist
                    best_obj  = obj
        return best_obj

    def smooth_depth(self, obj_id, raw_depth):
        self.depth_history[obj_id].append(raw_depth)
        if len(self.depth_history[obj_id]) > self.HISTORY_SIZE:
            self.depth_history[obj_id].pop(0)
        return float(np.median(self.depth_history[obj_id]))

    def get_distance(self, bbox, depth_np, obj_id):
        """Try ZED object match first, fall back to depth map."""
        zed_obj = self.find_matching_zed_object(bbox)
        if zed_obj is not None:
            raw = float(zed_obj.position[0])  # position[0] = depth/forward
            pos = (float(zed_obj.position[0]),
                   float(zed_obj.position[1]),
                   float(zed_obj.position[2]))
            conf_src = float(zed_obj.confidence) / 100.0
        else:
            raw = self.get_depth_from_depthmap(depth_np, bbox)
            raw = raw if raw is not None else 0.0
            pos = (raw, 0.0, 0.0)
            conf_src = None

        distance = self.smooth_depth(obj_id, raw)
        return distance, pos, conf_src

    # ?? Build auv_msgs Detection ??????????????????????????????????????????????

    def build_custom_det(self, header, bbox, class_id, confidence, distance, pos, tracking_id):
        x1, y1, x2, y2 = bbox
        det             = Detection()
        det.header      = header
        det.tracking_id = tracking_id
        det.class_id    = class_id
        det.confidence  = confidence

        det.bbox_x      = float((x1 + x2) / 2)
        det.bbox_y      = float((y1 + y2) / 2)
        det.bbox_width  = float(x2 - x1)
        det.bbox_height = float(y2 - y1)

        det.position.x = pos[0]  # depth/forward
        det.position.y = pos[1]  # up/down
        det.position.z = pos[2]  # left/right

        if self.imu_pose is not None:
            det.orientation = self.imu_pose
        return det

    # ?? Build vision_msgs Detection3D ?????????????????????????????????????????

    def build_det3d(self, header, bbox, class_id, confidence, pos):
        x1, y1, x2, y2 = bbox
        det3d        = Detection3D()
        det3d.header = header

        hyp                      = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id  = class_id
        hyp.hypothesis.score     = confidence

        pose              = Pose()
        pose.position     = Point(x=pos[0], y=pos[1], z=pos[2])
        pose.orientation  = Quaternion(w=1.0)
        hyp.pose.pose     = pose

        det3d.results.append(hyp)
        det3d.bbox.center   = pose
        det3d.bbox.size.x   = float(x2 - x1)
        det3d.bbox.size.y   = float(y2 - y1)
        det3d.bbox.size.z   = 0.1
        return det3d

    # ?? Main synced callback ??????????????????????????????????????????????????

    def synced_callback(self, img_msg, depth_msg):
        frame    = self.cv2_bridge.imgmsg_to_cv2(img_msg,   desired_encoding='bgr8')
        depth_np = self.cv2_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        det_array        = DetectionArray()
        det_array.header = img_msg.header

        det3d_array        = Detection3DArray()
        det3d_array.header = img_msg.header

        current_ids = set()
        tracking_id = 0

        # ?? YOLO: gate detections ?????????????????????????????????????????????
        yolo_dets = self.get_yolo_bboxes(frame)
        for (x1, y1, x2, y2, conf, cls_name) in yolo_dets:
            bbox    = (x1, y1, x2, y2)
            obj_id  = f'yolo_{tracking_id}'
            current_ids.add(obj_id)

            distance, pos, zed_conf = self.get_distance(bbox, depth_np, obj_id)
            confidence = zed_conf if zed_conf is not None else conf

            # auv_msgs
            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, cls_name,
                                      confidence, distance, pos, tracking_id))

            # vision_msgs for BT
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, cls_name, confidence, pos))

            # Draw ? blue box for YOLO gate
            label = f'{cls_name} {conf:.2f} | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label,
                        (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            self.get_logger().info(
                f'[YOLO] {cls_name} at {distance:.2f}m', throttle_duration_sec=2.0)
            tracking_id += 1

        # ?? HSV: pole detections ??????????????????????????????????????????????
        hsv_bboxes = self.get_hsv_bboxes(frame)
        for bbox in hsv_bboxes:
            x1, y1, x2, y2 = bbox
            obj_id = f'hsv_{tracking_id}'
            current_ids.add(obj_id)

            distance, pos, zed_conf = self.get_distance(bbox, depth_np, obj_id)
            confidence = zed_conf if zed_conf is not None else 0.5

            # auv_msgs
            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, 'preq_pole',
                                      confidence, distance, pos, tracking_id))

            # vision_msgs for BT
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, 'preq_pole', confidence, pos))

            # Draw ? green box for HSV pole
            label = f'preq_pole {confidence:.2f} | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label,
                        (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.get_logger().info(
                f'[HSV] preq_pole at {distance:.2f}m', throttle_duration_sec=2.0)
            tracking_id += 1

        # Clean up stale depth history
        for old_id in list(self.depth_history.keys()):
            if old_id not in current_ids:
                del self.depth_history[old_id]

        # Publish
        try:
            annotated_msg        = self.cv2_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = img_msg.header
            self.detect_img_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

        self.detection_pub.publish(det_array)
        self.detection3d_pub.publish(det3d_array)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()