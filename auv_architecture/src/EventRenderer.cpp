#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <auv_architecture/EventRenderer.hpp>

EventsRenderer::EventsRenderer() : rclcpp::Node("events_renderer")
{
    event_sub_ = create_subscription<stonefish_ros2::msg::EventArray>(
        "/events_topic", 10,
        std::bind(&EventsRenderer::dataCallback, this, std::placeholders::_1));
}

EventsRenderer::~EventsRenderer()
{
    event_sub_.reset();
    image_pub_.shutdown();
}

void EventsRenderer::init()
{
    // image_transport::ImageTransport it(shared_from_this());
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_pub_ = it_->advertise("/image_topic", 1);
}

void EventsRenderer::publishImage(const cv_bridge::CvImage &image)
{
    sensor_msgs::msg::Image::SharedPtr events_image = image.toImageMsg();
    image_pub_.publish(events_image);
}

void EventsRenderer::dataCallback(const stonefish_ros2::msg::EventArray::SharedPtr message)
{
    int num_events = message->events.size();

    cv_bridge::CvImage render_image;
    if (num_events > 0)
        render_image.header.stamp = message->events[num_events / 2].ts;

    render_image.encoding = "bgr8";
    render_image.image = cv::Mat(message->height, message->width, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < num_events; ++i)
    {
        const int x = message->events[i].x;
        const int y = message->events[i].y;
        render_image.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (message->events[i].polarity ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
    }

    publishImage(render_image);
}
