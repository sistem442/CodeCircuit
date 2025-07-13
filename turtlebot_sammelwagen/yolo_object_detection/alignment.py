import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

def get_yaw_from_quat(quat):
    orientation = [
        quat.x,
        quat.y,
        quat.z,
        quat.w
    ]
    _, _, yaw = euler_from_quaternion(orientation)
    return yaw

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def rotate_to_relative_angle(node, target_angle_rad, angular_speed=0.3):
    twist = Twist()

    odom_msg = rclpy.task.wait_for_message(node, Odometry, '/odom', timeout_sec=1.0)
    if not odom_msg:
        node.get_logger().warn("Keine Odometrie empfangen – Drehung abgebrochen.")
        return

    start_yaw = get_yaw_from_quat(odom_msg.pose.pose.orientation)
    target_yaw = normalize_angle(start_yaw + target_angle_rad)

    node.get_logger().info(f"Drehe um {math.degrees(target_angle_rad):.1f}°")

    r = node.create_rate(10)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        odom = node.get_parameter_or('/odom', None)
        if odom is None:
            continue
        current_yaw = get_yaw_from_quat(odom.pose.pose.orientation)
        angle_error = normalize_angle(target_yaw - current_yaw)

        if abs(angle_error) < math.radians(2):
            break

        twist.angular.z = angular_speed if angle_error > 0 else -angular_speed
        node.cmd_pub.publish(twist)
        r.sleep()

    node.cmd_pub.publish(Twist())  # Stopp

def drive_forward_distance(node, distance_m, linear_speed=0.05):
    twist = Twist()

    odom_msg = rclpy.task.wait_for_message(node, Odometry, '/odom', timeout_sec=1.0)
    if not odom_msg:
        node.get_logger().warn("Keine Odometrie empfangen – Vorwärtsfahrt abgebrochen.")
        return

    start_x = odom_msg.pose.pose.position.x
    start_y = odom_msg.pose.pose.position.y

    node.get_logger().info(f"Fahre geradeaus {distance_m*100:.0f} cm")

    r = node.create_rate(10)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        odom = node.get_parameter_or('/odom', None)
        if odom is None:
            continue
        pos = odom.pose.pose.position
        dx = pos.x - start_x
        dy = pos.y - start_y
        traveled = math.sqrt(dx**2 + dy**2)

        if traveled >= distance_m:
            break

        twist.linear.x = linear_speed
        node.cmd_pub.publish(twist)
        r.sleep()

    node.cmd_pub.publish(Twist())  # Stopp

def align_for_loading(node):
    node.get_logger().info("Starte präzise Ausrichtung zur Ladeposition")

    # 45° nach rechts (negativ)
    rotate_to_relative_angle(node, math.radians(-45))

    # 20 cm vor
    drive_forward_distance(node, 0.2)

    # 45° nach links (positiv)
    rotate_to_relative_angle(node, math.radians(45))

    # Geradeaus zur Ladeposition
    drive_forward_distance(node, 0.3)

    node.get_logger().info("Ladeposition erreicht ")
    node.moving = False
    node.approaching = False
    node.aligned = True

