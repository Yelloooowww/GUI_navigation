#include "sub_thread.h"
#include "mainwindow.h"


sub_thread::sub_thread(QObject *parent, NodeHandle *nh, Mainwindow *gui, string topic_name, int id):
  QThread(parent)
{
  this->nh = nh;
  this->gui = gui;
  this->topic_name = topic_name;
  this->my_id = id;
  it = new image_transport::ImageTransport(*nh);

}

void sub_thread::run()
{

  subcribe_to_image(true);

  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  spinner.start();
  ros::waitForShutdown();

}

void sub_thread::cb_image(const sensor_msgs::ImageConstPtr &msg)
{
   cv_bridge::CvImagePtr cv_ptr;
   cv::Mat image;

   cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
   image = cv_ptr->image;

   cv::Mat rgb;
   if(image.channels()==3)
   {
       //cvt Mat BGR 2 QImage RGB
       cv::cvtColor(image, rgb, CV_BGR2RGB);
       img = new QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, rgb.cols*rgb.channels(), QImage::Format_RGB888);
   }
   else
   {
       img = new QImage((const unsigned char*)(image.data), image.cols, image.rows, image.cols*image.channels(), QImage::Format_RGB888);
   }

   gui->image_list[my_id] = img;
   emit gui->set_image_sig(my_id);
}

void sub_thread::subcribe_to_image(bool s)
{
  if(s){
    sub = it->subscribe(topic_name,1, &sub_thread::cb_image, this);
  }
  else{
    sub.shutdown();
  }
}
