#ifndef AUV_VEL_CTRL_HPP
#define AUV_VEL_CTRL_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
#include <auv_controllers/auv_velocity_controller_params.hpp>
namespace auv_controllers
{
    class BodyVelocityController : public controller_interface::ChainableControllerInterface
    {
    private:
        Eigen::MatrixXd tam_inv_;
        Eigen::VectorXd command;
        geometry_msgs::msg::Twist twist_command;
        geometry_msgs::msg::Twist twist_state;
        realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> rt_command_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

        // Parameters from ROS for
        std::shared_ptr<auv_velocity_controller::ParamListener> param_listener_;
        auv_velocity_controller::Params params_;

    public:
        BodyVelocityController(/* args */);
        ~BodyVelocityController();

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // call when controller
        bool on_set_chained_mode(bool activate) override;

        std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        // Chainable controller replaces update() with the following two functions
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
        // configure the interface to write commands to (HW)
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    };

};

#endif // AUV_VEL_CTRL_HPP
