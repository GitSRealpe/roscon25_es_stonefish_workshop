#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import Twist


class ChainPub(Node):

    def __init__(self):
        super().__init__("chain_node_pub")

        self.publisher: Publisher = self.create_publisher(
            Twist, "/auv_velocity_controller/body_velocity_command", 10
        )

        self.msg = Twist()
        self.msg.linear.x = 1.0

        self.timer: Timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = ChainPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
