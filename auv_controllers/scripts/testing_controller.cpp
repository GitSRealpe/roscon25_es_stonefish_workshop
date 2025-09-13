#include <rclcpp/rclcpp.hpp>
#include <auv_controllers/BodyVelocityController.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "starting test\n";

    auv_controllers::BodyVelocityController ctrl;

    rclcpp::shutdown();

    return 0;
}
