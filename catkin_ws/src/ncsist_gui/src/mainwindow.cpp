#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/ros.h"
#include <sstream>


MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  nh =new ros::NodeHandle;
  // Publisher
  h1_pose=nh->advertise<std_msgs::Int16MultiArray>("ui_position", 1);
  h2_cmd_vel=nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  h3_joy=nh->advertise<sensor_msgs::Joy>("joy_teleop/joy", 1);
  h4_deal_with_goBack=nh->advertise<std_msgs::String>("deal_with_goBack", 1);
  h5_rotate=nh->advertise<std_msgs::Int32>("rotate_angle", 1);


  for (int i = 0; i < 6; i++)
  {
    image_list.push_back(new QImage);
  }

  // thread of Subscriber
  // sub_cam = new sub_thread(parent, nh, this, "/camera/color/image_raw", 0);
  sub_cam = new sub_thread(parent, nh, this, "/box_pred/img", 0);
  sub_cam->start();
  sub_arti = new sub_artimap(parent, nh, this, "artifact_pose", 0);
  sub_arti->start();
  sub_arti_cmd_vel = new sub_artimap(parent, nh, this, "cmd_vel", 1);
  sub_arti_cmd_vel->start();

  connect(this, SIGNAL(set_image_sig(int)), SLOT(set_image(int)));
  connect(this, SIGNAL(add_to_pose_sig(QString)), SLOT(add_to_pose(QString)));

  ROS_INFO("MainWin init done");
}

void MainWindow::set_image(int i)
{
  if (image_list[i])
  {
    switch (i)
    {
    case 0:
      ui->label_img->setPixmap(QPixmap::fromImage(*image_list[i]));
      break;
    }
  }
}

// add text_items to the display area
void MainWindow::add_to_pose(QString arti_string)
{
  ROS_INFO("MainWindow::add_to_pose");
  ui->listWidget->clear();
  ui->listWidget->addItem(arti_string);
}
void MainWindow::add_to_cmd_vel(QString cmd_vel_string)
{
  ui->listWidget_2->clear();
  ui->listWidget_2->addItem(cmd_vel_string);
}


void MainWindow::mouseDoubleClickEvent(QMouseEvent *e){
  // if the click point in the image range
  if( (e->x()-10)>=0 && (e->x()-10)<=640 && (e->y()-10)>=0 && (e->y()-10)<=480 ){
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(e->x()-10);
    msg.data.push_back(e->y()-10);
    h1_pose.publish(msg);
  }
}

void MainWindow::on_pushButton_left_pressed()
{
  ROS_INFO("on_pushButton_left_pressed");
  ui->listWidget_2->addItem("left\n");

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x=0;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0.2;
  h2_cmd_vel.publish(vel_msg);
}

void MainWindow::on_pushButton_up_pressed()
{
  ROS_INFO("on_pushButton_up_pressed");
  ui->listWidget_2->addItem("up\n");

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x=0.1;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  h2_cmd_vel.publish(vel_msg);
}

void MainWindow::on_pushButton_right_pressed()
{
  ROS_INFO("on_pushButton_right_pressed");
  ui->listWidget_2->addItem("right\n");

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x=0;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = -0.2;
  h2_cmd_vel.publish(vel_msg);
}

void MainWindow::on_pushButton_down_pressed()
{
  ROS_INFO("on_pushButton_down_pressed");
  ui->listWidget_2->addItem("down\n");

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x= -0.1;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  h2_cmd_vel.publish(vel_msg);
}


void MainWindow::on_radioButton_auto_clicked()
{
  sensor_msgs::Joy msg;
  msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  h3_joy.publish(msg);
}


void MainWindow::on_radioButton_manual_clicked()
{
  sensor_msgs::Joy msg;
  msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.buttons = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  h3_joy.publish(msg);
}


void MainWindow::on_radioButton_back_clicked()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "auto_move_back" ;
  msg.data = ss.str();
  h4_deal_with_goBack.publish(msg);


  sensor_msgs::Joy jmsg;
  jmsg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  jmsg.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  h3_joy.publish(jmsg);
}


void MainWindow::on_pushButton_clicked()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "record position" ;
  msg.data = ss.str();
  h4_deal_with_goBack.publish(msg);
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_01_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_02_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*2;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_03_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*3;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_04_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*4;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_05_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*5;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_06_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*6;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_07_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*7;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_08_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*8;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_09_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*9;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_10_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*10;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_11_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*11;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_12_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*12;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_13_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*13;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_14_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*14;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_15_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*15;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_16_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*16;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_17_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*17;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_18_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*18;
  h5_rotate.publish(msg);
}

void MainWindow::on_pushButton_19_clicked()
{
  std_msgs::Int32 msg;
  msg.data = 18*19;
  h5_rotate.publish(msg);
}
