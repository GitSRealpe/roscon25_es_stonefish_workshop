#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class AUVFeedbackNode : public rclcpp::Node
{
public:
  AUVFeedbackNode() : Node("auv_feedback_node")
  {

    std::string frame_id = declare_parameter("frame_id", "empty");
    rclcpp::Parameter param = get_parameter("frame_id");

    twist_msg_.header.frame_id = param.as_string();
    pose_msg_.header.frame_id = param.as_string();

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/pub_twist_topic", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pub_pose_topic", 10);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sub_odom_topic", 10, std::bind(&AUVFeedbackNode::odomCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  geometry_msgs::msg::TwistStamped twist_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    twist_msg_.header.stamp = get_clock()->now();
    twist_msg_.twist = msg->twist.twist;

    pose_msg_.header.stamp = get_clock()->now();
    pose_msg_.pose = msg->pose.pose;

    twist_pub_->publish(twist_msg_);
    pose_pub_->publish(pose_msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AUVFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}