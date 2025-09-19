#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Publisher, Timer

from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import Twist


class ChainPub(Node):

    def __init__(self):
        super().__init__("chain_node_pub")

        # Publisher
        self.publisher: Publisher = self.create_publisher(
            MultiDOFCommand, "pid_controller/reference", 10
        )

        # Subscriber
        self.sub_twist = self.create_subscription(
            Twist,
            "/cmd_vel",  # change if needed
            self.twist_callback,
            10,
        )

        self.msg = MultiDOFCommand()
        self.msg.dof_names = [
            "auv_velocity_controller/x",
            "auv_velocity_controller/yaw",
        ]

    def twist_callback(self, msg: Twist):
        self.msg.values = [msg.linear.x, msg.angular.z]

        # Publish converted message
        self.publisher.publish(self.msg)
        self.get_logger().info(
            f"Converted Twist to PID command: lin=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), "
            f"ang=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        )


if __name__ == "__main__":

    rclpy.init()
    node = ChainPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
