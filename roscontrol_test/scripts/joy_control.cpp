#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class JoyToWrench : public rclcpp::Node
{
public:
    JoyToWrench() : Node("joy_to_wrench")
    {
        // Publishers and subscribers
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/auv_wrench_controller/body_wrench_command", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToWrench::joyCallback, this, std::placeholders::_1));

        // Scaling factors
        this->declare_parameter("force_scale", 1.0);
        this->declare_parameter("torque_scale", 5.0);
        force_scale_ = this->get_parameter("force_scale").as_double();
        torque_scale_ = this->get_parameter("torque_scale").as_double();

        RCLCPP_INFO(this->get_logger(), "JoyToWrench node started.");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 6)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Joystick message has fewer axes than expected.");
            return;
        }

        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header.stamp = this->get_clock()->now();
        wrench.header.frame_id = "base_link";

        // Xbox One typical mapping
        double lx = msg->axes[0]; // Left stick X
        double ly = msg->axes[1]; // Left stick Y
        double rx = msg->axes[3]; // Right stick X
        double ry = msg->axes[4]; // Right stick Y

        // Map joystick to wrench
        wrench.wrench.force.x = ly * force_scale_;    // Forward/backward
        wrench.wrench.force.y = -lx * force_scale_;   // Strafe left/right
        wrench.wrench.force.z = -ry * force_scale_;   // Up/down via triggers
        wrench.wrench.torque.z = -rx * torque_scale_; // Yaw

        publisher_->publish(wrench);
    }

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    double force_scale_;
    double torque_scale_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToWrench>());
    rclcpp::shutdown();
    return 0;
}
