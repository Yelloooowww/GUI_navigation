#ifndef SUB_THREAD_H
#define SUB_THREAD_H

#include <QObject>
#include <QThread>
#include <QImage>

#include <sstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace std;
using namespace ros;
using namespace cv;

class Mainwindow;

class sub_thread : public QThread
{
  Q_OBJECT

public:
  sub_thread(QObject *parent, NodeHandle *nh, Mainwindow *gui,string topic_name, int id);
  void run();

  void cb_image(const sensor_msgs::ImageConstPtr &msg );
  void subcribe_to_image(bool s);
  image_transport::Subscriber sub;



private:
  NodeHandle *nh;
  // Publisher pub = nh.advertise<std_msgs::String>("pixel_position", 1000);
  Mainwindow *gui;
  string topic_name;
  QImage *img;
  image_transport::ImageTransport *it;
  int my_id;


};

#endif // SUB_THREAD_H
