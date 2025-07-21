#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

class AckermannToTwistNode(Node):
    def __init__(self):
        super().__init__("ackermann_to_twist")

        # Could parameterize wheelbase if you want a better Ackermann approximation
        self.declare_parameter('wheelbase', 1.0)
        self.wheelbase = self.get_parameter('wheelbase').value

        # Subscribe to AckermannDrive commands (e.g. on "/cmd_ackermann")
        self.sub_ack = self.create_subscription(
            AckermannDrive,
            '/cmd_ackermann',
            self.ackermann_cb,
            10
        )

        # Publish standard Twist on "/cmd_vel"
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("AckermannToTwistNode: Subscribed to /cmd_ackermann, publishing /cmd_vel (Twist)")

    def ackermann_cb(self, ack_msg):
        speed = ack_msg.speed                  # [m/s]
        steering = ack_msg.steering_angle      # [rad]
        wheelbase = self.wheelbase

        twist = Twist()
        twist.linear.x = float(speed)

        # Approximate turning using v * tan(steering)/wheelbase
        if abs(steering) > 1e-6:
            twist.angular.z = float(speed * math.tan(steering) / wheelbase)
        else:
            twist.angular.z = 0.0

        self.pub_twist.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwistNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()