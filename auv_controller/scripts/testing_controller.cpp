#include <rclcpp/rclcpp.hpp>
#include <auv_controller/auv_controller.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "starting test\n";

    auv_controller::BodyVelocityController ctrl;

    rclcpp::shutdown();

    return 0;
}
