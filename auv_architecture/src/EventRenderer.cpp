// Base import
#include <auv_architecture/EventRenderer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.hpp>

EventsRenderer::EventsRenderer() : rclcpp::Node("events_renderer")
{
    // Initialize subscriber and publisher
    event_sub_ = create_subscription<stonefish_ros2::msg::EventArray>(
        "/events_topic", 10, std::bind(&EventsRenderer::dataCallback, this, std::placeholders::_1));
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_topic", 10);
}

void EventsRenderer::publishImage(const cv_bridge::CvImage &image)
{
    sensor_msgs::msg::Image::SharedPtr events_image = image.toImageMsg();
    image_pub_->publish(*events_image);
}

void EventsRenderer::dataCallback(const stonefish_ros2::msg::EventArray::SharedPtr message)
{
    int num_events = message->events.size();
    // Save image timestamp
    cv_bridge::CvImage render_image;
    if (num_events > 0)
    {
        render_image.header.stamp = message->events[num_events / 2].ts;
    }
    // Create events image
    render_image.encoding = "bgr8";
    render_image.image = cv::Mat(message->height, message->width, CV_8UC3);
    render_image.image = cv::Scalar(255, 255, 255);
    // Save events image
    for (int i = 0; i < num_events; ++i)
    {
        const int x = message->events[i].x;
        const int y = message->events[i].y;
        render_image.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (message->events[i].polarity ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
    }
    // TODO: Fix sensor in Stonefish
    // cv::flip(render_image.image, render_image.image, 0);
    publishImage(render_image);
}