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

        # Subskrypcja obrazu
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',   # ← upewnij się, że to Twój topic!
            self.image_callback,
            10
        )

        # Publisher do UR5
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Zapamiętujemy aktualny kąt osi 0
        self.current_pan_angle = 0.0

        # Nazwy jointów UR5
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Okno OpenCV
        self.window_name = "Click control - camera view"
        cv2.namedWindow(self.window_name)
        cv2.startWindowThread()
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self.window_name, self.cv_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.cv_image is None:
            return

        h, w, _ = self.cv_image.shape
        mid_y = h // 2

        # Ustal kierunek ruchu
        if y < mid_y:
            self.get_logger().info("Klik GÓRA → +10°")
            delta = np.deg2rad(10)
        else:
            self.get_logger().info("Klik DÓŁ → -10°")
            delta = -np.deg2rad(10)

        self.current_pan_angle += delta
        self.send_trajectory(self.current_pan_angle)

    def send_trajectory(self, angle):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            angle,  # Sterujemy tylko pan joint
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        point.time_from_start.sec = 1

        traj.points.append(point)

        # Publikacja do UR5
        self.traj_pub.publish(traj)
        self.get_logger().info(f"Wysłano trajektorię: pan = {angle:.3f} rad")

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
