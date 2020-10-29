#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QObject>
#include <QCheckBox>
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

#include <sstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include "sub_thread.h"
#include "sub_artimap.h"
#include "roscpp_tutorials/TwoInts.h"
#include "subt_msgs/artifact.h"
#include "subt_msgs/ArtifactPose.h"
#include "subt_msgs/ArtifactPoseArray.h"
#include "subt_msgs/int8.h"
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    vector<QImage *> image_list;
    roscpp_tutorials::TwoInts srv;
    subt_msgs::int8 srv_cmd_vel;


private slots:
    void on_pushButton_left_pressed();
    void on_pushButton_up_pressed();
    void on_pushButton_right_pressed();
    void on_pushButton_down_pressed();
    void on_radioButton_back_clicked();
    void on_radioButton_auto_clicked();
    void on_radioButton_manual_clicked();
    void on_pushButton_clicked();
    void on_pushButton_01_clicked();
    void on_pushButton_02_clicked();
    void on_pushButton_03_clicked();
    void on_pushButton_04_clicked();
    void on_pushButton_05_clicked();
    void on_pushButton_06_clicked();
    void on_pushButton_07_clicked();
    void on_pushButton_08_clicked();
    void on_pushButton_09_clicked();
    void on_pushButton_10_clicked();
    void on_pushButton_11_clicked();
    void on_pushButton_12_clicked();
    void on_pushButton_13_clicked();
    void on_pushButton_14_clicked();
    void on_pushButton_15_clicked();
    void on_pushButton_16_clicked();
    void on_pushButton_17_clicked();
    void on_pushButton_18_clicked();
    void on_pushButton_19_clicked();

private:
    Ui::MainWindow *ui;
    NodeHandle *nh;
    sub_thread *sub_cam;
    sub_artimap *sub_arti;
    sub_artimap *sub_arti_cmd_vel;
    Publisher h1_pose;
    Publisher h2_cmd_vel;
    Publisher h3_joy;
    Publisher h4_deal_with_goBack;
    Publisher h5_rotate;

signals:
  void set_image_sig(int);
  void add_to_pose_sig(QString);

public slots:
  void set_image(int);
  void add_to_pose(QString arti_string);
  void add_to_cmd_vel(QString cmd_vel_string);

protected:
  void mouseDoubleClickEvent(QMouseEvent *event);



};
#endif // MAINWINDOW_H
