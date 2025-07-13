import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

#fur fahren auf rechte seiten des Ballen
#from yolo_object_detection.alignment import align_for_loading

class ApproachWithYolo(Node):
    def __init__(self):
        super().__init__('approach_with_yolo')

        self.bridge = CvBridge()
        self.model = YOLO('/home/boris/ros2_ws/src/yolo_model/best.pt')

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.approaching = False
        self.aligned = False
        self.moving = False
        self.last_msg_time = self.get_clock().now()

        self.create_timer(0.5, self.check_for_timeout)

    def check_for_timeout(self):
        now = self.get_clock().now()
        if self.moving and (now - self.last_msg_time) > Duration(seconds=0.3):
            self.get_logger().warn("Timeout: Kein Bildsignal – stoppe Roboter.")
            self.cmd_pub.publish(Twist())
            self.moving = False
            self.approaching = False
            self.aligned = False

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
                    box_width = x2 - x1

                    # Visualisierung
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    if not self.aligned:
                        if box_width < 180:
                            twist = Twist()
                            twist.linear.x = 0.05
                            self.cmd_pub.publish(twist)
                            self.moving = True
                            self.approaching = True
                            self.get_logger().info("Fahre auf Ballen zu...")
                        else:
                            self.cmd_pub.publish(Twist())
                            self.moving = False
                            self.approaching = False
                            self.aligned = True
                            self.get_logger().info("Ballen nahe genug – starte Ausrichtung")
                            align_for_loading(self)
                    break

            if not bale_found:
                if self.moving:
                    self.get_logger().info("Kein Objekt erkannt – stoppe Roboter.")
                    self.cmd_pub.publish(Twist())
                    self.moving = False
                self.approaching = False
                self.aligned = False

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

