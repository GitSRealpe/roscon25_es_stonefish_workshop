#include "auv_architecture/EventRenderer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventsRenderer>());
    rclcpp::shutdown();
    return 0;
}