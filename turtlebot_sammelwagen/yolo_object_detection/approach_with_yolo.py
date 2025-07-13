import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class ApproachWithYolo(Node):
    def __init__(self):
        super().__init__('approach_with_yolo')

        self.bridge = CvBridge()
        self.model = YOLO('/home/boris/turtlebot3_ws/src/yolo_model/best.pt')

        # Kamera → BEST_EFFORT
        image_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, image_qos)

        # Ultraschall
        self.ultra_sub = self.create_subscription(Float32, '/distance_ultra', self.ultrasonic_callback, 10)

        # Annotiertes Bild
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)

        # Bewegungssteuerung → RELIABLE
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)

        self.approaching = False
        self.ultra_too_close = False
        self.last_msg_time = self.get_clock().now()
        self.last_twist = Twist()

        self.create_timer(0.1, self.send_last_twist)
        self.create_timer(0.5, self.check_for_timeout)

    def send_last_twist(self):
        if self.approaching and not self.ultra_too_close:
            self.last_twist.linear.x = 0.05
            self.cmd_pub.publish(self.last_twist)
            self.get_logger().info("🚀 Fahre vorwärts…")
        else:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🧯 Fahre nicht – Sicherheitsstopp aktiv.")

    def check_for_timeout(self):
        now = self.get_clock().now()
        if self.approaching and (now - self.last_msg_time) > Duration(seconds=2):
            self.get_logger().warn("⚠️ Timeout – kein Bildsignal. Roboter gestoppt.")
            self.cmd_pub.publish(Twist())
            self.approaching = False

    def ultrasonic_callback(self, msg):
        distance = msg.data
        self.ultra_too_close = distance < 0.15
        if self.ultra_too_close:
             self.get_logger().info(f"🛑 Ultraschall: {distance:.2f} m – Stop!")
             self.approaching = False  # optional – Block auch YOLO-Befehle
             self.cmd_pub.publish(Twist())  # 🚫 Sofort stoppen!
             return
        else:
            self.get_logger().info(f"📏 Ultraschall: {distance:.2f} m – Weiterfahren erlaubt")

    def image_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)[0]

            bale_found = False
            for det in results.boxes:
                cls = int(det.cls)
                label = self.model.names[cls]
                conf = float(det.conf)

                if conf > 0.5 and label.lower() in ['bale', 'walze'] and not self.ultra_too_close:
                    x1, y1, x2, y2 = map(int, det.xyxy[0])
                    box_height = y2 - y1

                    # Heuristik: Wenn Box kleiner als Schwelle, ist Objekt noch "weit entfernt"
                    self.approaching = box_height < 300

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    self.get_logger().info(f"🎯 Ballen erkannt – Höhe: {box_height} px – approaching={self.approaching}")
                    bale_found = True
                    break

            if not bale_found:
                self.approaching = False
                self.get_logger().info("❌ Kein Ballen erkannt – Halte an.")

            img_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(img_out)

        except Exception as e:
            self.get_logger().error(f"Fehler bei YOLO-Verarbeitung: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachWithYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Abbruch erkannt – Node wird heruntergefahren.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

