#ifndef EVENT_RENDERER_HPP_
#define EVENT_RENDERER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stonefish_ros2/msg/event_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.hpp>

class EventsRenderer : public rclcpp::Node
{
public:
    EventsRenderer();
    ~EventsRenderer();
    void init();

private:
    void dataCallback(const stonefish_ros2::msg::EventArray::SharedPtr message);
    void publishImage(const cv_bridge::CvImage &image);

    rclcpp::Subscription<stonefish_ros2::msg::EventArray>::SharedPtr event_sub_;
    image_transport::Publisher image_pub_;
    std::shared_ptr<image_transport::ImageTransport> it_;
};

#endif // EVENT_RENDERER_HPP_