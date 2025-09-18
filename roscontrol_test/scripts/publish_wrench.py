#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from geometry_msgs.msg import WrenchStamped


class ChainPub(Node):

    def __init__(self):
        super().__init__("wrench_node_pub")

        self.publisher: Publisher = self.create_publisher(
            WrenchStamped, "/auv_wrench_controller/body_wrench_command", 10
        )

        self.msg = WrenchStamped()
        self.msg.wrench.force.x = 1.0

        self.timer: Timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = ChainPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
