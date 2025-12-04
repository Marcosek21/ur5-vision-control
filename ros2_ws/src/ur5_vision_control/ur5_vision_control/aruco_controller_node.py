import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoControllerNode(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')

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

        # Sterujemy tylko jednym jointem — drugim (shoulder_lift_joint)
        self.joint_name = 'shoulder_lift_joint'
        self.current_angle = 0.0

        # ArUco konfiguracja
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Timer do odświeżania sterowania co 0.5s
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info("ArucoControllerNode started — sterowanie shoulder_lift_joint")

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def control_loop(self):
        if self.cv_image is None:
            return

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        h, w, _ = self.cv_image.shape
        mid_y = h // 2

        if ids is not None:
            # używamy pierwszego markera
            c = corners[0][0]
            center_y = int((c[0][1] + c[2][1]) / 2)

            # logika ruchu
            if center_y < mid_y:
                delta = np.deg2rad(5)
                self.get_logger().info("Aruco NAD środkiem → ruch +5°")
            else:
                delta = -np.deg2rad(5)
                self.get_logger().info("Aruco POD środkiem → ruch -5°")

            # aktualizacja kąta
            self.current_angle += delta

            # wyślij trajektorię
            self.send_trajectory(self.current_angle)

        # podgląd obrazu z narysowanym markerem
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids)

        cv2.imshow("Aruco control - camera view", self.cv_image)
        cv2.waitKey(1)

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
    node = ArucoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
