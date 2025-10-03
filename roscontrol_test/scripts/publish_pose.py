#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from geometry_msgs.msg import PoseStamped


class PosePub(Node):

    def __init__(self):
        super().__init__("pose_command_pub")

        self.publisher: Publisher = self.create_publisher(
            PoseStamped, "/auv_pose_controller/body_pose_command", 10
        )

        self.msg = PoseStamped()
        self.msg.pose.position.x = 2.0
        self.msg.pose.position.y = 0.0
        self.msg.pose.position.z = 3.0
        self.msg.pose.orientation.w = 1.0

        self.timer: Timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


if __name__ == "__main__":

    rclpy.init()
    node = PosePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
