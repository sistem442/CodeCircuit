import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestMover(Node):
    def __init__(self):
        super().__init__('test_mover')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)

    def move(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.pub.publish(twist)

rclpy.init()
node = TestMover()
rclpy.spin(node)

