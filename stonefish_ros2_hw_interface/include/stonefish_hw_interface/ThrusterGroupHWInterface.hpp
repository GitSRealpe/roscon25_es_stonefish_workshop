#ifndef STNFSH_HW_INTRFC_HPP
#define STNFSH_HW_INTRFC_HPP

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace stonefish_hw_interface
{
    class ThrusterGroupHWInterface : public hardware_interface::SystemInterface
    {
    private:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_group_publisher_;

        std::vector<std::string> joint_names_;
        std::vector<double> velocity_commands_;
        std::vector<double> velocity_states_;
        std::vector<double> position_states_;
        std::vector<double> torque_states_;

    public:
        ThrusterGroupHWInterface(/* args */);
        ~ThrusterGroupHWInterface();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    };

};

#endif // STNFSH_HW_INTRFC_HPP
