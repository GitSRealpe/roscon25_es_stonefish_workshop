#ifndef STNFSH_HW_INTRFC_HPP
#define STNFSH_HW_INTRFC_HPP

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"

namespace stonefish_hw_interface
{
    class ThrusterGroupHWInterface : public hardware_interface::SystemInterface
    {
    private:
        /* data */
    public:
        ThrusterGroupHWInterface(/* args */);
        ~ThrusterGroupHWInterface();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    };

};

#endif // STNFSH_HW_INTRFC_HPP
