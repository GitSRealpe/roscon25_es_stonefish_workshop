#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    subscription_ = create_subscription<std_msgs::msg::Float64>(
        "/controller_output", 10, std::bind(&TestNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received controller output: %f", msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}