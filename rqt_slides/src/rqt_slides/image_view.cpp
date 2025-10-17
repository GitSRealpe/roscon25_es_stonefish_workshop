#include <rqt_slides/image_view.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rqt_slides/parse_xml.hpp>
namespace rqt_slides
{

  ImageView::ImageView()
      : rqt_gui_cpp::Plugin(), widget_(0)
  {
    setObjectName("ImageView");
  }

  void ImageView::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1)
    {
      widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    updateTopicList();
    ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText("/bluerov_roscon/main_camera/image_color"));
    connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

    ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
    connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

    ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
    connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));

    // set topic name if passed in as argument
    const QStringList &argv = context.argv();
    if (!argv.empty())
    {
      arg_topic_name = argv[0];
      selectTopic(arg_topic_name);
    }
    pub_topic_custom_ = false;

    ui_.image_frame->setOuterLayout(ui_.image_layout);

    hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
    hide_toolbar_action_->setCheckable(true);
    ui_.image_frame->addAction(hide_toolbar_action_);
    connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));

    // Connect button signals to slots
    connect(ui_.prev_button, SIGNAL(clicked(bool)), this, SLOT(prevButtonPress()));
    connect(ui_.next_button, SIGNAL(clicked(bool)), this, SLOT(nextButtonPress()));
    ui_.prev_button->setDisabled(true);

    guion_ = std::make_shared<guionUtils::GuionParser>(ament_index_cpp::get_package_share_directory("rqt_slides") + "/resource/guion.xml");
    tinyxml2::XMLElement *slide = guion_->getSlide(0);
    ui_.slide_title->setText(QString(slide->FirstAttribute()->Value()));
    ui_.slide_content->setText(QString(guion_->getSlideHTMLContent(slide).c_str()));
    updateTopicList();
  }

  void ImageView::shutdownPlugin()
  {
    subscriber_.shutdown();
    pub_mouse_left_.reset();
  }

  void ImageView::prevButtonPress()
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

  void ImageView::nextButtonPress()
  {
    slide_number++;

    if (slide_number >= 1)
    {
      ui_.next_button->setDisabled(true);
      slide_number = 1;
    }
    ui_.prev_button->setDisabled(false);
    ui_.lcdNumber->display(slide_number);

    changeSlide(slide_number);
  }

  void ImageView::changeSlide(int index)
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
  }

  // ///////////////////////////////////////////////////////////

  void ImageView::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
  {
    (void)plugin_settings;
    QString topic = ui_.topics_combo_box->currentText();
    // qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
    instance_settings.setValue("topic", topic);
    instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  }

  void ImageView::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
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
      // qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
      selectTopic(topic);
    }

    bool toolbar_hidden = instance_settings.value("toolbar_hidden", false).toBool();
    hide_toolbar_action_->setChecked(toolbar_hidden);
  }

  void ImageView::updateTopicList()
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
      // qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
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

  QSet<QString> ImageView::getTopics(const QSet<QString> &message_types, const QSet<QString> &message_sub_types, const QList<QString> &transports)
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
          // qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

          // add transport specific sub-topics
          for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
          {
            if (all_topics.contains(topic + "/" + *jt))
            {
              QString sub = topic + " " + *jt;
              topics.insert(sub);
              // qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
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
            // qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
          }
        }
      }
    }
    return topics;
  }

  void ImageView::selectTopic(const QString &topic)
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

  void ImageView::onTopicChanged(int index)
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
      try
      {
        auto subscription_options = rclcpp::SubscriptionOptions();
        subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        subscriber_ = image_transport::create_subscription(
            node_.get(),
            topic.toStdString(),
            std::bind(&ImageView::callbackImage, this, std::placeholders::_1),
            hints.getTransport());
        qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
      }
      catch (image_transport::TransportLoadException &e)
      {
        QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
      }
    }
  }

  void ImageView::onZoom1(bool checked)
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

  void ImageView::onHideToolbarChanged(bool hide)
  {
    ui_.toolbar_widget->setVisible(!hide);
  }

  void ImageView::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try
    {
      // First let cv_bridge do its magic
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
      try
      {
        // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        if (msg->encoding == "CV_8UC3")
        {
          // assuming it is rgb
          conversion_mat_ = cv_ptr->image;
        }
        else if (msg->encoding == "8UC1")
        {
          // convert gray to rgb
          cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
        }
        else if (msg->encoding == "16UC1" || msg->encoding == "32FC1")
        {
          // scale / quantify
          double min = 0;
          double max = 3.0;
          if (msg->encoding == "16UC1")
            max *= 1000;
          // dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max)
          {
            // completely homogeneous images are displayed in gray
            min = 0;
            max = 2;
          }
          cv::Mat img_scaled_8u;
          cv::Mat(cv_ptr->image - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
          cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
        }
        else
        {
          qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
          ui_.image_frame->setImage(QImage());
          return;
        }
      }
      catch (cv_bridge::Exception &e)
      {
        qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
        ui_.image_frame->setImage(QImage());
        return;
      }
    }

    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    ui_.image_frame->setImage(image);

    if (!ui_.zoom_1_push_button->isEnabled())
    {
      ui_.zoom_1_push_button->setEnabled(true);
    }
    // Need to update the zoom 1 every new image in case the image aspect ratio changed,
    // though could check and see if the aspect ratio changed or not.
    onZoom1(ui_.zoom_1_push_button->isChecked());
  }
}

PLUGINLIB_EXPORT_CLASS(rqt_slides::ImageView, rqt_gui_cpp::Plugin)
