#ifndef AUV_CTRL_HPP
#define AUV_CTRL_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"

namespace auv_controller
{
    class BodyVelocityController : public controller_interface::ControllerInterface
    {
    private:
        /* data */
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
