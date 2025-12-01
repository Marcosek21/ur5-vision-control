import rclpy
from rclpy.node import Node

class ArucoControllerNode(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')
        self.get_logger().info("ArucoControllerNode started (empty).")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()