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

  QObject::connect(&qnode, SIGNAL(recvImg_qr()), this, SLOT(update_qr()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::update_qr()
{
  cv::Mat qr_clone_mat = qnode.qr_qnode->clone();

  if (qnode.isRecv_msg1)
  {
    cv::rectangle(qr_clone_mat, cv::Rect(0, 0, 200, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
    switch (qnode.result_maxAngle)
    {
      case -3:
        cv::putText(qr_clone_mat, "left_down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case -2:
        cv::putText(qr_clone_mat, "down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case -1:
        cv::putText(qr_clone_mat, "right_down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 0:
        cv::putText(qr_clone_mat, "right", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 1:
        cv::putText(qr_clone_mat, "right_up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 2:
        cv::putText(qr_clone_mat, "up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 3:
        cv::putText(qr_clone_mat, "left_up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 4:
        cv::putText(qr_clone_mat, "left", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
    }
  }
  if (qnode.isRecv_msg2)
  {
    cv::rectangle(qr_clone_mat, cv::Rect(640 - 200, 0, 640, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
    switch (qnode.direction_i)
    {
      case 1:
        cv::putText(qr_clone_mat, "CW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 0:
        cv::putText(qr_clone_mat, "CCW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
    }
  }

  ui.label->setPixmap(QPixmap::fromImage(
      QImage((const unsigned char*)(qr_clone_mat.data), qr_clone_mat.cols, qr_clone_mat.rows, QImage::Format_RGB888)));
  delete qnode.qr_qnode;
  qnode.isRecv_qr = false;
}

}  // namespace rescue_vision_ui
