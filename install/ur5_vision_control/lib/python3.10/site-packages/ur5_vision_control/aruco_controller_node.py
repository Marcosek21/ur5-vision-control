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

        # TODO: podmień topic obrazu na ten, którego używasz w click_controller_node!
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',     # ← jeśli u Ciebie jest np. /usb_cam/image_raw, zmień tutaj
            self.image_callback,
            10
        )

        # Publisher do UR5 – ten sam co w node klikowym
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Nazwy jointów UR5
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Aktualny kąt pierwszego przegubu
        self.current_pan = 0.0

        # OpenCV okno
        self.window_name = "ArUco control - UR5"
        cv2.namedWindow(self.window_name)
        cv2.startWindowThread()

        # ArUco – słownik i parametry
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Aby nie spamować komendami – zapamiętujemy czas ostatniej wysłanej trajektorii
        self.last_cmd_time = self.get_clock().now()
        self.min_cmd_interval = Duration(seconds=0.5)  # min 0.5s między komendami

        self.get_logger().info("ArucoControllerNode started.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detekcja markerów ArUco
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        h, w = frame.shape[:2]
        mid_y = h // 2

        # Rysujemy linię środka dla wizualizacji
        cv2.line(frame, (0, mid_y), (w, mid_y), (255, 0, 0), 2)

        direction = 0  # +1 – góra, -1 – dół, 0 – brak ruchu

        if ids is not None and len(ids) > 0:
            # Bierzemy pierwszy wykryty marker
            c = corners[0][0]  # (4, 2)
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            # Rysujemy środek markera
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

            # Deadzone – żeby nie latał co piksel
            dead_zone = int(h * 0.05)  # 5% wysokości

            if cy < mid_y - dead_zone:
                direction = +1
                txt = "Marker POWYZEJ srodka → RUCH W GORE"
            elif cy > mid_y + dead_zone:
                direction = -1
                txt = "Marker PONIZEJ srodka → RUCH W DOL"
            else:
                txt = "Marker w strefie martwej – brak ruchu"

            cv2.putText(frame, txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Brak markera ArUco", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Wyświetlenie obrazu
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

        # Wysyłamy komendę tylko, jeśli minęło trochę czasu i jest ruch
        now = self.get_clock().now()
        if direction != 0 and (now - self.last_cmd_time) > self.min_cmd_interval:
            delta = np.deg2rad(5) * direction  # mniejszy krok niż klik – 5°
            self.current_pan += delta
            self.send_trajectory(self.current_pan)
            self.last_cmd_time = now

    def send_trajectory(self, pan_angle_rad):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            pan_angle_rad,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        point.time_from_start.sec = 1

        traj.points.append(point)
        self.traj_pub.publish(traj)

        self.get_logger().info(f"[ArUco] UR5 → pan={pan_angle_rad:.3f} rad")


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
