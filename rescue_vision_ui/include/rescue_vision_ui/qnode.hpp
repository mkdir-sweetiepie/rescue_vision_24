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

  cv::Mat* original = NULL;
  cv::Mat* original_thermal = NULL;
  bool isRecv = false;
  bool isRecv_thermal = false;
  ros::Publisher victim_start;
Q_SIGNALS:
  void rosShutdown();

  void recvImg();
  void recvImg_thermal();

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber image_sub;
  image_transport::Subscriber thermal_sub;

  void victim_Callback(const sensor_msgs::ImageConstPtr& msg_img);
  void thermal_Callback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace rescue_vision_ui

#endif /* rescue_vision_ui_QNODE_HPP_ */
