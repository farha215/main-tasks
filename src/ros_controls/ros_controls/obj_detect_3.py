import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge
import cv2

from zed_msgs.msg import ObjectsStamped
from auv_msgs.msg import Detection,DetectionArray


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('obj_detection_node')
        self.cv2_bridge=CvBridge()
        self.imu_pose=None 
        self.raw_img=None
        self.objArr=None

        self.detect_img_pub=self.create_publisher(Image,"/zed2i_front/detection_image",10)
        self.detection_pub=self.create_publisher(DetectionArray,"/zed2i_front/detection_msg",10)

        self.front_raw_img_sub=self.create_subscription(Image,"/zed2i_front/zed_node/rgb/color/rect/image",self.raw_img_callback,10)
        self.front_imu_sub=self.create_subscription(Imu,"/zed2i_front/zed_node/imu/data",self.imu_callback,10)
        self.objects_sub=self.create_subscription(ObjectsStamped,"/zed2i_front/zed_node/obj_det/objects",self.obj_callback,10)

    def raw_img_callback(self,msg):
        self.raw_img = self.cv2_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #self.detect_img_pub.publish(msg)
        pass
    def imu_callback(self,msg):
        self.imu_pose=msg.orientation
        pass
    def obj_callback(self,msg):

        if self.raw_img is None:
            return
        
        frame = self.raw_img.copy()
        det_array = DetectionArray()
        det_array.header = msg.header

        for obj in msg.objects:
            # Get the 4 corners of the bounding box
            corners = obj.bounding_box_2d.corners

            x_coords = [c.kp[0] for c in corners]
            y_coords = [c.kp[1] for c in corners]

            x_min=int(min(x_coords))
            y_min=int(min(y_coords))
            x_max=int(max(x_coords))
            y_max=int(max(y_coords))

            cv2.rectangle(frame,(x_min,y_min),(x_max,y_max),(0,0,255),1)
            # Get the Z-axis distance in meters
            distance = obj.position[0] 
            
            # Write the label and distance above the bounding box
            label = f"{obj.label} | {distance:.2f}m"
            cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            custom_det = Detection()
            
            # Label and confidence
            custom_det.class_id = obj.label
            custom_det.confidence = obj.confidence
            
            # 2D Bounding Box (Calculating the center point and width/height)
            custom_det.bbox_x = float((x_min + x_max) / 2)
            custom_det.bbox_y = float((y_min + y_max) / 2)
            custom_det.bbox_width = float(x_max - x_min)
            custom_det.bbox_height = float(y_max - y_min)
            
            # 3D Position
            custom_det.position.x = float(obj.position[0])
            custom_det.position.y = float(obj.position[1])
            custom_det.position.z = float(distance)
            
            # Slap the latest IMU orientation on there if we have it
            if self.imu_pose:
                custom_det.orientation = self.imu_pose
                
            # Add this object to the array!
            det_array.detections.append(custom_det)

        annotated_msg = self.cv2_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.detect_img_pub.publish(annotated_msg)

        self.detection_pub.publish(det_array)
    
    
            

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()