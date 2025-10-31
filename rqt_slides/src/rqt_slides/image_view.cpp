#include <rqt_slides/image_view.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rqt_slides/parse_xml.hpp>
namespace rqt_slides
{

  Slideshow::Slideshow()
      : rqt_gui_cpp::Plugin(), widget_(0)
  {
    setObjectName("Slideshow");
  }

  void Slideshow::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1)
    {
      widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    updateTopicList();
    ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText("/bluerov_roscon/front_camera/image_color"));
    connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

    ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
    connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

    ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
    connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));

    // set topic name if passed in as argument
    // const QStringList &argv = context.argv();
    // if (!argv.empty())
    // {
    //   arg_topic_name = argv[0];
    //   selectTopic(arg_topic_name);
    // }

    ui_.image_frame->setOuterLayout(ui_.image_layout);

    // Connect button signals to slots
    connect(ui_.prev_button, SIGNAL(clicked(bool)), this, SLOT(prevButtonPress()));
    connect(ui_.next_button, SIGNAL(clicked(bool)), this, SLOT(nextButtonPress()));
    ui_.prev_button->setDisabled(true);

    guion_ = std::make_shared<guionUtils::GuionParser>(ament_index_cpp::get_package_share_directory("rqt_slides") + "/resource/guion.xml");
    tinyxml2::XMLElement *slide = guion_->getSlide(0);
    ui_.slide_title->setText(QString(slide->FirstAttribute()->Value()));
    ui_.slide_content->setText(QString(guion_->getSlideHTMLContent(slide).c_str()));
    updateTopicList();
    // selectTopic("/bluerov_roscon/front_camera/image_color");
    tinyxml2::XMLElement *imageTopicElem = slide->FirstChildElement("image_topic");
    std::cout << imageTopicElem->GetText() << "\n";
    ui_.topics_combo_box->setCurrentText(imageTopicElem->GetText());
    selectTopic(imageTopicElem->GetText());
    // ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText("/bluerov_roscon/front_camera/image_color"));
    // ui_.topics_combo_box->setCurrentIndex(1);
    // std::cout << ui_.topics_combo_box->currentIndex() << "\n";

    pub_rpose = node_->create_publisher<geometry_msgs::msg::Pose>("/slides/pose_command", 10);

    ui_.x_pos_line->setText(QString(std::to_string(0.0).c_str()));
    ui_.y_pos_line->setText(QString(std::to_string(0.0).c_str()));
    ui_.z_pos_line->setText(QString(std::to_string(0.0).c_str()));
    ui_.yaw_pos_line->setText(QString(std::to_string(0.0).c_str()));

    sub_pose = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/bluerov_roscon/navigator/pose", 1, std::bind(&Slideshow::poseCallback, this, std::placeholders::_1));
  }

  void Slideshow::shutdownPlugin()
  {
    subscriber_.shutdown();
    pub_rpose.reset();
  }

  void Slideshow::prevButtonPress()
  {
    slide_number--;
    if (slide_number <= 0)
    {
      ui_.prev_button->setDisabled(true);
      slide_number = 0;
    }
    ui_.next_button->setDisabled(false);
    ui_.lcdNumber->display(slide_number);

    changeSlide(slide_number);
  }

  void Slideshow::nextButtonPress()
  {
    slide_number++;

    if (slide_number >= guion_->slideCount - 1)
    {
      ui_.next_button->setDisabled(true);
      slide_number = guion_->slideCount - 1;
    }
    ui_.prev_button->setDisabled(false);
    ui_.lcdNumber->display(slide_number);

    changeSlide(slide_number);
  }

  void Slideshow::changeSlide(int index)
  {
    tinyxml2::XMLElement *slide = guion_->getSlide(index);
    ui_.slide_title->setText(QString(slide->FirstAttribute()->Value()));
    ui_.slide_content->setText(QString(guion_->getSlideHTMLContent(slide).c_str()));

    tinyxml2::XMLElement *imageTopicElem = slide->FirstChildElement("image_topic");
    if (imageTopicElem)
    {
      const char *topic = imageTopicElem->GetText();
      ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(topic));
    }

    tinyxml2::XMLElement *rPoseElem = slide->FirstChildElement("robot_pose");

    if (rPoseElem)
    {
      const char *xyzStr = rPoseElem->Attribute("xyz");
      const char *rpyStr = rPoseElem->Attribute("rpy");

      float xyz[3], rpy[3];
      if (!guion_->parseFloatArray(xyzStr, xyz) || !guion_->parseFloatArray(rpyStr, rpy))
      {
        RCLCPP_WARN(node_->get_logger(), "Could not parse pose values");
        return;
      }

      // Convert RPY to quaternion
      tf2::Quaternion q;
      q.setRPY(rpy[0], rpy[1], rpy[2]);
      geometry_msgs::msg::Pose pose;
      pose.position.x = xyz[0];
      pose.position.y = xyz[1];
      pose.position.z = xyz[2];
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      pub_rpose->publish(pose);
    }
  }

  void Slideshow::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    ui_.x_pos_line->setText(QString(std::to_string(msg->pose.position.x).c_str()));
    ui_.y_pos_line->setText(QString(std::to_string(msg->pose.position.y).c_str()));
    ui_.z_pos_line->setText(QString(std::to_string(msg->pose.position.z).c_str()));

    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ui_.yaw_pos_line->setText(QString(std::to_string(yaw).c_str()));
  }

  // ///////////////////////////////////////////////////////////

  void Slideshow::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
  {
    (void)plugin_settings;
    QString topic = ui_.topics_combo_box->currentText();
    // qDebug("Slideshow::saveSettings() topic '%s'", topic.toStdString().c_str());
    instance_settings.setValue("topic", topic);
    instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  }

  void Slideshow::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
  {
    (void)plugin_settings;
    bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
    ui_.zoom_1_push_button->setChecked(zoom1_checked);

    QString topic = instance_settings.value("topic", "").toString();
    // don't overwrite topic name passed as command line argument
    if (!arg_topic_name.isEmpty())
    {
      arg_topic_name = "";
    }
    else
    {
      // qDebug("Slideshow::restoreSettings() topic '%s'", topic.toStdString().c_str());

      selectTopic("/bluerov_roscon/front_camera/image_color");
    }
  }

  void Slideshow::updateTopicList()
  {
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    message_types.insert("sensor_msgs/msg/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");
    message_sub_types.insert("sensor_msgs/msg/CompressedImage");

    // get declared transports
    QList<QString> transports;
    image_transport::ImageTransport it(node_);
    std::vector<std::string> declared = it.getDeclaredTransports();
    for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
    {
      // qDebug("Slideshow::updateTopicList() declared transport '%s'", it->c_str());
      QString transport = it->c_str();

      // strip prefix from transport name
      QString prefix = "image_transport/";
      if (transport.startsWith(prefix))
      {
        transport = transport.mid(prefix.length());
      }
      transports.append(transport);
    }

    QString selected = ui_.topics_combo_box->currentText();

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    topics.append("");
    std::sort(topics.begin(), topics.end());
    ui_.topics_combo_box->clear();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
      QString label(*it);
      label.replace(" ", "/");
      ui_.topics_combo_box->addItem(label, QVariant(*it));
    }

    // restore previous selection
    selectTopic(selected);
  }

  QSet<QString> Slideshow::getTopics(const QSet<QString> &message_types, const QSet<QString> &message_sub_types, const QList<QString> &transports)
  {
    std::map<std::string, std::vector<std::string>> topic_info = node_->get_topic_names_and_types();

    QSet<QString> all_topics;
    for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin(); it != topic_info.end(); ++it)
    {
      all_topics.insert(it->first.c_str());
    }

    QSet<QString> topics;
    for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin(); it != topic_info.end(); ++it)
    {
      for (std::vector<std::string>::const_iterator msg_type_it = it->second.begin(); msg_type_it != it->second.end(); ++msg_type_it)
      {
        if (message_types.contains(msg_type_it->c_str()))
        {
          QString topic = it->first.c_str();

          // add raw topic
          topics.insert(topic);
          // qDebug("Slideshow::getTopics() raw topic '%s'", topic.toStdString().c_str());

          // add transport specific sub-topics
          for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
          {
            if (all_topics.contains(topic + "/" + *jt))
            {
              QString sub = topic + " " + *jt;
              topics.insert(sub);
              // qDebug("Slideshow::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
            }
          }
        }
        if (message_sub_types.contains(msg_type_it->c_str()))
        {
          QString topic = it->first.c_str();
          int index = topic.lastIndexOf("/");
          if (index != -1)
          {
            topic.replace(index, 1, " ");
            topics.insert(topic);
            // qDebug("Slideshow::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
          }
        }
      }
    }
    return topics;
  }

  void Slideshow::selectTopic(const QString &topic)
  {
    int index = ui_.topics_combo_box->findText(topic);
    if (index == -1)
    {
      // add topic name to list if not yet in
      QString label(topic);
      label.replace(" ", "/");
      ui_.topics_combo_box->addItem(label, QVariant(topic));
      index = ui_.topics_combo_box->findText(topic);
    }
    ui_.topics_combo_box->setCurrentIndex(index);
  }

  void Slideshow::onTopicChanged(int index)
  {
    conversion_mat_.release();
    subscriber_.shutdown();

    // reset image on topic change
    ui_.image_frame->setImage(QImage());

    QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    if (!topic.isEmpty())
    {
      const image_transport::TransportHints hints(node_.get(), transport.toStdString());
      // const image_transport::TransportHints hints(node_.get(), "compressed");
      try
      {
        auto subscription_options = rclcpp::SubscriptionOptions();
        subscriber_ = image_transport::create_subscription(
            node_.get(),
            topic.toStdString(),
            std::bind(&Slideshow::callbackImage, this, std::placeholders::_1),
            hints.getTransport(), rmw_qos_profile_default, subscription_options);
        // qDebug("Slideshow::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
        // RCLCPP_INFO(node_->get_logger(), "Slideshow::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
      }
      catch (image_transport::TransportLoadException &e)
      {
        QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
      }
    }
  }

  void Slideshow::onZoom1(bool checked)
  {
    if (checked)
    {
      if (ui_.image_frame->getImage().isNull())
      {
        return;
      }
      ui_.image_frame->setInnerFrameFixedSize(ui_.image_frame->getImage().size());
    }
    else
    {
      ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
      ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
      widget_->setMinimumSize(QSize(80, 60));
      widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    }
  }

  void Slideshow::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv::Mat cv_image;
    const std::string &encoding = msg->encoding;

    try
    {
      // Step 1: Try direct conversion to RGB8 (most common case)
      if (encoding == sensor_msgs::image_encodings::RGB8)
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_ptr->image;
      }
      // Step 2: Handle BGR8 (very common from cameras)
      else if (encoding == sensor_msgs::image_encodings::BGR8)
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, cv_image, cv::COLOR_BGR2RGB);
      }
      // Step 3: Monochrome 8-bit
      else if (encoding == sensor_msgs::image_encodings::MONO8 ||
               encoding == sensor_msgs::image_encodings::TYPE_8UC1)
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        cv::cvtColor(cv_ptr->image, cv_image, cv::COLOR_GRAY2RGB);
      }
      // Step 4: 16-bit depth (common: 16UC1)
      else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
               encoding == sensor_msgs::image_encodings::MONO16)
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        const cv::Mat &depth = cv_ptr->image;
        double max_val = 3000.0; // Default: 4 meters
        cv::Mat scaled;
        double scale = 255.0 / max_val;
        depth.convertTo(scaled, CV_8U, scale);
        cv::cvtColor(scaled, cv_image, cv::COLOR_GRAY2RGB);
      }
      // Step 5: 32-bit float depth (e.g., from RealSense)
      else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        const cv::Mat &depth = cv_ptr->image;
        double min_val, max_val;
        cv::minMaxLoc(depth, &min_val, &max_val);
        if (max_val <= min_val)
          max_val = min_val + 1.0;

        cv::Mat scaled;
        cv::Mat(depth - min_val).convertTo(scaled, CV_8U, 255.0 / (max_val - min_val));
        cv::cvtColor(scaled, cv_image, cv::COLOR_GRAY2RGB);
      }
      // Step 6: Fallback — let cv_bridge try
      else
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv_image = cv_ptr->image;
      }
    }
    catch (const cv_bridge::Exception &e)
    {
      qWarning("Slideshow: Failed to convert image from '%s': %s", encoding.c_str(), e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }

    // Convert to QImage (zero-copy if possible)
    if (cv_image.empty())
    {
      ui_.image_frame->setImage(QImage());
      return;
    }

    QImage qimage;
    if (cv_image.type() == CV_8UC3 && cv_image.channels() == 3)
    {
      // Fast path: direct RGB888
      qimage = QImage(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888);
    }
    else
    {
      // Fallback: ensure RGB8
      cv::Mat rgb;
      if (cv_image.channels() == 1)
        cv::cvtColor(cv_image, rgb, cv::COLOR_GRAY2RGB);
      else if (cv_image.depth() != CV_8U)
        cv_image.convertTo(rgb, CV_8U);
      else
        rgb = cv_image;

      if (rgb.channels() == 4)
        cv::cvtColor(rgb, rgb, cv::COLOR_RGBA2RGB);

      qimage = QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
    }

    // Note: QImage does NOT copy data unless modified or .copy() is called
    ui_.image_frame->setImage(qimage.rgbSwapped()); // Qt expects RGB, OpenCV gives BGR sometimes

    // Enable zoom button if needed
    if (!ui_.zoom_1_push_button->isEnabled())
    {
      ui_.zoom_1_push_button->setEnabled(true);
    }
    onZoom1(ui_.zoom_1_push_button->isChecked());
  }
}

PLUGINLIB_EXPORT_CLASS(rqt_slides::Slideshow, rqt_gui_cpp::Plugin)
