import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ZedFrontCameraPublisher(Node):

    def __init__(self):
        super().__init__('zed_front_camera_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the raw ZED front camera image
        self.subscription = self.create_subscription(
            Image,
            '/zed2i_front/zed_node/left/image_rect_color',
            self.image_callback,
            qos
        )

        # Republish on a short, easy-to-find topic
        self.publisher = self.create_publisher(Image, '/front_cam/image_raw', qos)

        self.get_logger().info("Listening on /zed2i_front/zed_node/left/image_rect_color")
        self.get_logger().info("Publishing to  /front_cam/image_raw")

    def image_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Frame received and republished ? "
            f"{msg.width}x{msg.height} encoding={msg.encoding}",
            throttle_duration_sec=2.0   # log at most once every 2 seconds
        )


def main(args=None):
    rclpy.init(args=args)
    node = ZedFrontCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()