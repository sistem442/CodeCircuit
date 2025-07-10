import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy




class ApproachWithYolo(Node):
    def __init__(self):
        super().__init__('approach_with_yolo')

        self.bridge = CvBridge()
        self.model = YOLO('/home/boris/ros2_ws/src/yolo_model/best.pt')

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.ultra_sub = self.create_subscription(Float32, '/distance_ultra', self.ultrasonic_callback, 10)
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)

        self.min_distance = 0.10  # 10 cm
        self.moving = False
        self.approaching = False
        self.last_msg_time = self.get_clock().now()

        self.create_timer(0.5, self.check_for_timeout)
        self.last_twist = Twist()
        self.cmd_timer = self.create_timer(0.1, self.send_last_twist)  # 10 Hz
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

    def send_last_twist(self):
        if self.moving:
            self.cmd_pub.publish(self.last_twist)


    def check_for_timeout(self):
        now = self.get_clock().now()

        if self.moving and self.moving and (now - self.last_msg_time) > Duration(seconds=2):
            self.get_logger().warn(f"Timeout: Letztes Bild vor {(now - self.last_msg_time).nanoseconds / 1e9:.2f} s – stoppe Roboter.")
            self.cmd_pub.publish(Twist())
            self.moving = False
            self.approaching = False

    def ultrasonic_callback(self, msg):
       
        distance = msg.data
        if self.approaching:
            if distance < self.min_distance:
                self.get_logger().info(f"Ziel in {distance:.2f} m – Stop!")
                self.cmd_pub.publish(Twist())
                self.moving = False
                self.approaching = False
            else:
                twist = Twist()
                twist.linear.x = 0.3
                self.last_twist = twist
                self.moving = True
                self.get_logger().info(f"Ultraschall: {msg.data:.2f} m – approaching={self.approaching}")
                self.last_twist = twist


    def image_callback(self, msg):
        try:
            self.last_msg_time = self.get_clock().now()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)[0]

            bale_found = False
            for det in results.boxes:
                cls = int(det.cls)
                label = self.model.names[cls]
                conf = float(det.conf)
                if conf > 0.5 and label.lower() in ['bale', 'walze']:
                    bale_found = True
                    x1, y1, x2, y2 = map(int, det.xyxy[0])

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    self.get_logger().info("Ballen erkannt – Annäherung aktiviert")
                    self.approaching = True
                    break

            if not bale_found:
                if self.moving:
                    self.get_logger().info("Kein Objekt erkannt – stoppe Roboter.")
                    self.cmd_pub.publish(Twist())
                    self.moving = False
                self.approaching = False

            img_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(img_out)

        except Exception as e:
            self.get_logger().error(f"Fehler bei YOLO-Analyse: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachWithYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

