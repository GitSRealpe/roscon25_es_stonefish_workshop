#include "auv_architecture/EventRenderer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventsRenderer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}