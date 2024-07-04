/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rescue_vision_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rescue_vision_ui
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(update()));
  QObject::connect(&qnode, SIGNAL(recvImg_thermal()), this, SLOT(update_thermal()));

  QObject::connect(ui.b, SIGNAL(clicked()), this, SLOT(button()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::update()
{
  cv::Mat qr_clone_mat = qnode.original->clone();

  ui.label->setPixmap(QPixmap::fromImage(
      QImage((const unsigned char*)(qr_clone_mat.data), qr_clone_mat.cols, qr_clone_mat.rows, QImage::Format_RGB888)));
  delete qnode.original;
  qnode.isRecv = false;
}

void MainWindow::update_thermal()
{
  cv::Mat qr_thermal_clone_mat = qnode.original_thermal->clone();
  ui.label_2->setPixmap(
      QPixmap::fromImage(QImage((const unsigned char*)(qr_thermal_clone_mat.data), qr_thermal_clone_mat.cols,
                                qr_thermal_clone_mat.rows, QImage::Format_RGB888)));
  delete qnode.original_thermal;
  qnode.isRecv_thermal = false;
}

void MainWindow::button()
{ 
  start_flag = "start";
  std_msgs::String msg;
  msg.data = start_flag;
  qnode.victim_start.publish(msg);
}
}  // namespace rescue_vision_ui
