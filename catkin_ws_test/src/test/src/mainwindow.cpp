#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "sub_thread.h"
#include "ros/ros.h"
#include "roscpp_tutorials/TwoInts.h"
#include <sstream>


Mainwindow::Mainwindow(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Mainwindow)
{
  ui->setupUi(this);

  nh = new ros::NodeHandle;
  h1 = nh->serviceClient<roscpp_tutorials::TwoInts>("HI_service");
//  setWindowState(Qt::WindowMaximized);

  for (int i = 0; i < 6; i++)
  {
    image_list.push_back(new QImage);
  }

  // sub_cam = new sub_thread(parent, nh, this, "/husky2/camera_middle/color/image_raw", 0);
  sub_cam = new sub_thread(parent, nh, this, "/camera/color/image_raw", 0);
  // sub_cam = new sub_thread(parent, nh, this, "img_input", 0);
  sub_cam->start();

  connect(this, SIGNAL(set_image_sig(int)), SLOT(set_image(int)));
}

Mainwindow::~Mainwindow()
{
  delete ui;
}

void Mainwindow::set_image(int i)
{
  if (image_list[i])
  {
    switch (i)
    {
    case 0:
      ui->label->setPixmap(QPixmap::fromImage(*image_list[i]));
      break;
    }
  }
}

void Mainwindow::on_pushButton_clicked()
{
  QImage Image;//聲明QImage 對象
  Image.load("/home/louis/圖片/0.jpg");//加載圖片，前提是該圖片存在項目的資源文件中
  QPixmap pixmap = QPixmap::fromImage(Image);//QPixmap引入Image
  ui->label->setPixmap(pixmap);//lable_logo 添加image
  int pic_with = pixmap.width();
  int pic_height = pixmap.height();
//  int with = ui->label->width();//設置寬度
//  int height = ui->label->height();//設置高度
//  QPixmap fitpixmap = pixmap.scaled(with, height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation); // 飽滿填充
//  QPixmap fitpixmap = pixmap.scaled(with, height, Qt::KeepAspectRatio, Qt::SmoothTransformation); // 按比例縮放
//  int img_with, img_height;
//  img_with = fitpixmap.width();
//  img_height = fitpixmap.height();
//  ui->label_2->setText(QString("width: ")+QString::number(img_with)+QString("\nheight: ")+QString::number(img_height));
//  ui->label->setPixmap(fitpixmap);//lable_logo 添加image

  ui->label_2->setText(QString("width: ")+QString::number(pic_with)+QString("\nheight: ")+QString::number(pic_height));
}

void Mainwindow::mouseDoubleClickEvent(QMouseEvent *e)
{
  ui->textBrowser->setText(QString("pag_x : ")+QString::number(e->x())+QString("  pag_y : ")+QString::number(e->y())
                           +QString("\n\nimg_x : ")+QString::number(e->x() - 13)+QString("  img_y : ")+QString::number(e->y() - 13));
  // qDebug() << e->x() << ":" << e->y();
  // qDebug() << "雙擊";

  srv.request.a  = e->x()-13 ;
  srv.request.b  = e->y()-13 ;
  if (h1.call(srv)){
    // qDebug() << "call success";
    ROS_INFO("call success");
  }
  else{
    // qDebug() << "Failed to call service";
    ROS_ERROR("Failed to call service");
  }

}

void Mainwindow::mouseMoveEvent(QMouseEvent *e)
{
  // qDebug() << "move" << e->x() << ":" << e->y();
  ;
}

void Mainwindow::on_pushButton_2_clicked()
{
  int lab_x = ui->label->x();
  int lab_y = ui->label->y();
  int with = ui->label->width();//設置寬度
  int height = ui->label->height();//設置高度
  ui->label_3->setText(QString("x: ")+QString::number(lab_x)+QString("\ny: ")+QString::number(lab_y)
                       +QString("\nwidth: ")+QString::number(with)+QString("\nheight: ")+QString::number(height));
}
