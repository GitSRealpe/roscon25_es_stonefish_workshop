#include <stonefish_hw_interface/ThrusterGroupHWInterface.hpp>

namespace stonefish_hw_interface
{
    ThrusterGroupHWInterface::ThrusterGroupHWInterface(/* args */)
    {
        std::cout << "constructing hw interface\n";
    }

    ThrusterGroupHWInterface::~ThrusterGroupHWInterface()
    {
    }

    hardware_interface::CallbackReturn ThrusterGroupHWInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        std::cout << "on_init thruster hw interface\n";
        std::cout << info.name << "\n";
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThrusterGroupHWInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // parameter are read here
        std::cout << "on_configure thruster hw interface\n";
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ThrusterGroupHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ThrusterGroupHWInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        return hardware_interface::return_type::OK;
    }

} // namespace stonefish_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stonefish_hw_interface::ThrusterGroupHWInterface, hardware_interface::SystemInterface)
