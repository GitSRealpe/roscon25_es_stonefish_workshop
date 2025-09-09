#include <rclcpp/rclcpp.hpp>
#include <stonefish_hw_interface/ThrusterGroupHWInterface.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "starting test\n";

    stonefish_hardware_interface::ThrusterGroupHWInterface thif;

    rclcpp::shutdown();

    return 0;
}
