/* Pilot node responsible for controlling pose of the AUV  */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class AUVPilotNode : public rclcpp::Node
{
public:
    AUVPilotNode() : Node("auv_pilot_node")
    {
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/auv_pose_controller/body_pose_command", 10);
        sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/slides/pose_command", 10, std::bind(&AUVPilotNode::goalCallback, this, std::placeholders::_1));

        goal_.header.stamp = get_clock()->now();
        goal_.pose.position.z = 1.0;
        goal_.pose.orientation.w = 1;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&AUVPilotNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Pilot node initialized");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    geometry_msgs::msg::PoseStamped goal_;

    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Got goal command");
        goal_.pose = *msg;
    }

    void timerCallback()
    {
        goal_.header.stamp = get_clock()->now();
        pose_pub_->publish(goal_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AUVPilotNode>());
    rclcpp::shutdown();
    return 0;
}