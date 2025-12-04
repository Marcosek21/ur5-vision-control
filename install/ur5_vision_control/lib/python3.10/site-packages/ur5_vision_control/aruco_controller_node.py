import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.duration import Duration

class ArucoControllerNode(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')

        self.bridge = CvBridge()

        # Subskrypcja obrazu
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publisher do UR5
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Nazwy jointów UR5
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',  # ← TERAZ STERUJEMY TYM
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Aktualny kąt LIFT (joint 2)
        self.current_lift = 0.0

        # OpenCV okno
        self.window_name = "ArUco control - UR5"
        cv2.namedWindow(self.window_name)
        cv2.startWindowThread()

        # ArUco parametry
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Anti-spam: min przerwa między komendami
        self.last_cmd_time = self.get_clock().now()
        self.min_cmd_interval = Duration(seconds=0.5)

        self.get_logger().info("ArucoControllerNode started (sterowanie shoulder_lift_joint).")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        h, w = frame.shape[:2]
        mid_y = h // 2
        direction = 0

        # Środkowa linia
        cv2.line(frame, (0, mid_y), (w, mid_y), (255, 0, 0), 2)

        if ids is not None and len(ids) > 0:
            c = corners[0][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

            dead_zone = int(h * 0.05)

            if cy < mid_y - dead_zone:
                direction = +1
                txt = "Marker POWYZEJ → LIFT +5°"
            elif cy > mid_y + dead_zone:
                direction = -1
                txt = "Marker PONIZEJ → LIFT -5°"
            else:
                txt = "Marker w strefie martwej"

            cv2.putText(frame, txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Brak markera ArUco", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

        # Warunek wysyłania komend
        now = self.get_clock().now()
        if direction != 0 and (now - self.last_cmd_time) > self.min_cmd_interval:
            delta = np.deg2rad(5) * direction
            self.current_lift += delta
            self.send_trajectory(self.current_lift)
            self.last_cmd_time = now

    def send_trajectory(self, lift_angle_rad):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            0.0,              # joint 1 PAN (nie ruszamy)
            lift_angle_rad,   # joint 2 LIFT (sterujemy)
            0.0,
            0.0,
            0.0,
            0.0
        ]
        point.time_from_start.sec = 1

        traj.points.append(point)
        self.traj_pub.publish(traj)

        self.get_logger().info(
            f"[ArUco] UR5 → shoulder_lift_joint = {lift_angle_rad:.3f} rad"
        )


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
