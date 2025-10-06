#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>

class PIDFeedbackVelNode : public rclcpp::Node
{
public:
  PIDFeedbackVelNode() : Node("pid_vel_feedback_node")
  {

    this->declare_parameter<std::vector<std::string>>("dof_state_names", {"dof1", "dof2", "dof3"});
    auto dof_names = this->get_parameter("dof_state_names").as_string_array();

    for (auto &name : dof_names)
    {
      pid_msg_.dof_names.push_back(name);
    }
    pid_pub_ = create_publisher<control_msgs::msg::MultiDOFCommand>("/pub_pid_measured_topic", 10);

    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/sub_twist_topic", 10, std::bind(&PIDFeedbackVelNode::velCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr pid_pub_;

  geometry_msgs::msg::TwistStamped twist_msg_;
  control_msgs::msg::MultiDOFCommand pid_msg_;

  void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    pid_msg_.values = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    pid_pub_->publish(pid_msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDFeedbackVelNode>());
  rclcpp::shutdown();
  return 0;
}