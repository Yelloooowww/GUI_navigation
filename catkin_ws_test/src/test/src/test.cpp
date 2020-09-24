#include <ros/ros.h>
#include <QApplication>
#include "mainwindow.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"test_gui");
  QApplication app(argc, argv);
  Mainwindow gui(nullptr);
  gui.show();
  int stat = app.exec();

  return stat;
}
