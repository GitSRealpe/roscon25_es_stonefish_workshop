#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from std_msgs.msg import Float64MultiArray


class PropellerPub(Node):

    def __init__(self):
        super().__init__("propeller_node_pub")

        self.publisher: Publisher = self.create_publisher(
            Float64MultiArray, "/bluerov_roscon/sf_controller/thruster_setpoints", 10
        )
        self.timer: Timer = self.create_timer(0.5, self.timer_callback)
        self.msg = Float64MultiArray()
        self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1]

    def timer_callback(self):
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = PropellerPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
