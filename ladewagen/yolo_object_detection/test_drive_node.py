import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TestDriveNode(Node):
    def __init__(self):
        super().__init__('test_drive_node')

        # RELIABLE QoS für Turtlebot3
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        self.create_timer(0.1, self.drive_forward)  # 10 Hz

        self.get_logger().info("🚦 TestDriveNode aktiv – sende Befehle auf /cmd_vel")

    def drive_forward(self):
        twist = Twist()
        twist.linear.x = 0.3  # oder 0.4, je nach Modell
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TestDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

