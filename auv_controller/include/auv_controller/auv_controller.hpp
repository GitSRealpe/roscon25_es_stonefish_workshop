#ifndef AUV_CTRL_HPP
#define AUV_CTRL_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
namespace auv_controller
{
    class BodyVelocityController : public controller_interface::ControllerInterface
    {
    private:
        Eigen::MatrixXd tam_inv_;
        geometry_msgs::msg::Twist twist_command;
        realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> rt_command_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

        // void twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    public:
        BodyVelocityController(/* args */);
        ~BodyVelocityController();

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    };

};

#endif // AUV_CTRL_HPP
