#ifndef EVENT_RENDERER_HPP_
#define EVENT_RENDERER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stonefish_ros2/msg/event_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

class EventsRenderer : public rclcpp::Node
{
public:
    explicit EventsRenderer();

private:
    void dataCallback(const stonefish_ros2::msg::EventArray::SharedPtr message);
    void publishImage(const cv_bridge::CvImage &image);

    rclcpp::Subscription<stonefish_ros2::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

#endif // EVENT_RENDERER_HPP_