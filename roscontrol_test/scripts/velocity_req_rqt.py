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
            MultiDOFCommand, "/auv_velocity_controller/reference", 10
        )

        # Subscriber
        self.sub_twist_h = self.create_subscription(
            Twist,
            "/cmd_vel/horizontal",  # change if needed
            self.horizontal_twist_callback,
            10,
        )

        self.sub_twist_v = self.create_subscription(
            Twist,
            "/cmd_vel/vertical",  # change if needed
            self.vertical_twist_callback,
            10,
        )

        self.cmd_msg = MultiDOFCommand()
        self.cmd_msg.dof_names = [
            "auv_wrench_controller/x/velocity",
            "auv_wrench_controller/z/velocity",
            "auv_wrench_controller/yaw/velocity",
        ]

        self.cmd_msg.values = [0, 0, 0]
        self.timer: Timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.cmd_msg)

    def horizontal_twist_callback(self, msg: Twist):
        self.cmd_msg.values[0] = msg.linear.x
        self.cmd_msg.values[2] = msg.angular.z
        # Publish converted message
        self.get_logger().info(
            f"Converted Twist-H to PID command: lin=({msg.linear.x:.2f}), "
            f"ang=({msg.angular.z:.2f})"
        )

    def vertical_twist_callback(self, msg: Twist):
        self.cmd_msg.values[1] = msg.linear.x
        # Publish converted message
        self.get_logger().info(
            f"Converted Twist-V to PID command: lin=({msg.linear.x:.2f}) "
        )


if __name__ == "__main__":

    rclpy.init()
    node = ChainPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
