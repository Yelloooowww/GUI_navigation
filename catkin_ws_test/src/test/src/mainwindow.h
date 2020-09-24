#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QDialog>
#include <QImage>
#include <QString>
#include <QPixmap>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPoint>
#include <QDebug>
#include <QThread>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "sub_thread.h"
#include "roscpp_tutorials/TwoInts.h"
#include <sstream>

using namespace std;
using namespace ros;

namespace Ui{
  class Mainwindow;
}

class Mainwindow : public QDialog
{
  Q_OBJECT

public:
  explicit Mainwindow(QWidget *parent = nullptr);
  ~Mainwindow();

  vector<QImage *> image_list;
  int position[2];

private:
  Ui::Mainwindow *ui;
  NodeHandle *nh;
  sub_thread *sub_cam;
  ServiceClient h1;
  roscpp_tutorials::TwoInts srv;

signals:
  void set_image_sig(int);

public slots:
  void set_image(int);

private slots:
  void on_pushButton_clicked();
  void on_pushButton_2_clicked();

protected:
void mouseMoveEvent(QMouseEvent *event);
void mouseDoubleClickEvent(QMouseEvent *event);

};

#endif // MAINWINDOW_H
