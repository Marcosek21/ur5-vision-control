import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class ClickControllerNode(Node):
    def __init__(self):
        super().__init__('click_controller_node')

        self.bridge = CvBridge()
        self.cv_image = None

        # Subskrypcja kamery
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publisher trajektorii
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Drugi joint UR5
        self.joint_name = 'shoulder_lift_joint'
        self.current_angle = 0.0

        # Okno OpenCV
        self.window_name = "Click Control - UR5"
        cv2.namedWindow(self.window_name)
        cv2.startWindowThread()
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("ClickControllerNode started — sterowanie 2. jointem (shoulder_lift_joint)")

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self.window_name, self.cv_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.cv_image is None:
            return

        h, w, _ = self.cv_image.shape
        mid_y = h // 2

        # Logika kliknięcia
        if y < mid_y:
            self.get_logger().info("Klik GÓRA → ruch +10°")
            delta = np.deg2rad(10)
        else:
            self.get_logger().info("Klik DÓŁ → ruch -10°")
            delta = -np.deg2rad(10)

        self.current_angle += delta
        self.send_trajectory(self.current_angle)

    def send_trajectory(self, angle):
        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [angle]
        point.time_from_start.sec = 1

        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info(f"[UR5] shoulder_lift_joint → {angle:.3f} rad")

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
