import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ClickControllerNode(Node):
    def __init__(self):
        super().__init__('click_controller_node')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.cv_image = None
        self.window_name = "Click control - camera view"

        # Init OpenCV window (only once)
        cv2.namedWindow(self.window_name)
        cv2.startWindowThread()
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("ClickControllerNode started.")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image
        cv2.imshow(self.window_name, self.cv_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.cv_image is None:
            return

        h, w, _ = self.cv_image.shape
        mid_y = h // 2

        if y < mid_y:
            self.get_logger().info(f"Kliknięcie w GÓRNEJ połowie (y={y})")
        else:
            self.get_logger().info(f"Kliknięcie w DOLNEJ połowie (y={y})")


def main(args=None):
    rclpy.init(args=args)
    node = ClickControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

