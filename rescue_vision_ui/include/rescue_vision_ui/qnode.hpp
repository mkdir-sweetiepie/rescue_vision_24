/**
 * @file /include/rescue_vision_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rescue_vision_ui_QNODE_HPP_
#define rescue_vision_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Int32.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rescue_vision_ui
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  cv::Mat* qr_qnode = NULL;
  bool isRecv_qr = false;
  bool isRecv_msg1 = false;
  bool isRecv_msg2 = false;
  int result_maxAngle;
  int direction_i;
  void result1_Callback(const std_msgs::Int32::ConstPtr& msg1);
  void result2_Callback(const std_msgs::Int32::ConstPtr& msg2);

Q_SIGNALS:
  void rosShutdown();

  void recvImg_qr();
  

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber qr_sub;
  ros::Subscriber c_result1_sub;
  ros::Subscriber c_result2_sub;
  void qr_Callback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace rescue_vision_ui

#endif /* rescue_vision_ui_QNODE_HPP_ */
