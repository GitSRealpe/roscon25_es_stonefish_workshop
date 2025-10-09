#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class FramePublisher(Node):

    def __init__(self):
        super().__init__('odom_to_tf_node')

        # Declare and acquire parameters
        self.fixed_frame_name = self.declare_parameter('fixed_frame', 'fixed_frame').get_parameter_value().string_value
        self.base_link_name = self.declare_parameter('base_link', 'base_link').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribe to odometry topic
        self.subscription = self.create_subscription(Odometry, '/odom_topic', self.odom_sub,1)
        self.subscription  # prevent unused variable warning

    def odom_sub(self, msg:Odometry):
        print("got msg")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.fixed_frame_name
        t.child_frame_id = self.base_link_name

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

if __name__=="__main__":

    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()