import rclpy
from rclpy.node import Node

class UR3eControlNode(Node):
    def __init__(self):
        super().__init__('ur3e_control_node')
        self.get_logger().info('UR3e control node has started.')

def main(args=None):
    rclpy.init(args=args)
    node = UR3eControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

