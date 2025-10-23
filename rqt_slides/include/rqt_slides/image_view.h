#ifndef rqt_slides__Slideshow_H
#define rqt_slides__Slideshow_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <rqt_slides/parse_xml.hpp>
#include <vector>
#include <memory>

namespace rqt_slides
{

  class Slideshow
      : public rqt_gui_cpp::Plugin
  {

    Q_OBJECT

  public:
    Slideshow();
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

  protected slots:
    virtual void updateTopicList();

  protected:
    virtual QSet<QString> getTopics(const QSet<QString> &message_types, const QSet<QString> &message_sub_types, const QList<QString> &transports);
    virtual void selectTopic(const QString &topic);

  protected slots:
    virtual void onTopicChanged(int index);
    virtual void onZoom1(bool checked);

    virtual void prevButtonPress();
    virtual void nextButtonPress();

  protected:
    virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    Ui::SlideshowWidget ui_;
    QWidget *widget_;
    image_transport::Subscriber subscriber_;
    cv::Mat conversion_mat_;

  private:
    QString arg_topic_name;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_rpose;
    bool pub_topic_custom_;

    std::shared_ptr<guionUtils::GuionParser> guion_;
    int slide_number = 0;
    int max_slides;

    void changeSlide(int index);
  };

}

#endif // rqt_slides__Slideshow_H
