#include <auv_controller/auv_controller.hpp>

namespace auv_controller
{
    BodyVelocityController::BodyVelocityController(/* args */)
    {
        std::cout << "constructing controller\n";
    }

    BodyVelocityController::~BodyVelocityController()
    {
    }

    controller_interface::CallbackReturn BodyVelocityController::on_init()
    {
        std::cout << "on_init controller\n";
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // parameter are read here
        std::cout << "on_configure controller\n";
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BodyVelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration BodyVelocityController::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn BodyVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type BodyVelocityController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        return controller_interface::return_type::OK;
    }

} // namespace auv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(auv_controller::BodyVelocityController, controller_interface::ControllerInterface)
