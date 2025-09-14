#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from control_msgs.msg import MultiDOFCommand


class ChainPub(Node):

    def __init__(self):
        super().__init__("chain_node_pub")

        self.publisher: Publisher = self.create_publisher(
            MultiDOFCommand, "pid_controller/reference", 10
        )

        self.msg = MultiDOFCommand()
        self.msg.dof_names = [
            "auv_velocity_controller/x",
            "auv_velocity_controller/yaw",
        ]
        self.msg.values = [1.0, 5.0]

        self.timer: Timer = self.create_timer(0.5, self.timer_callback)

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
