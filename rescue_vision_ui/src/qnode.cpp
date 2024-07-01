/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/rescue_vision_ui/qnode.hpp"
#include "std_msgs/Int32.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rescue_vision_ui
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "rescue_vision_ui");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.

  image_transport::ImageTransport image(n);
  image_sub = image.subscribe("/victim_image", 1, &QNode::victim_Callback, this);
  thermal_sub = image.subscribe("/img_result_thermal", 1, &QNode::thermal_Callback, this);
  victim_start = n.advertise<std_msgs::Int32>("/victim_start", 1);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::victim_Callback(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (!isRecv)  // imgRaw -> NULL, isreceived -> false
  {
    // ROS 이미지 메시지를 OpenCV Mat 형식으로 변환, 이미지 객체에 할당
    original = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);

    if (original != NULL)  // imgRaw 변환 성공
    {
      isRecv = true;
      Q_EMIT recvImg();  // 이미지 수신을 알리는 시그널 발생
    }
  }
}

void QNode::thermal_Callback(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (!isRecv_thermal)  // imgRaw -> NULL, isreceived -> false
  {
    // ROS 이미지 메시지를 OpenCV Mat 형식으로 변환, 이미지 객체에 할당
    original_thermal = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);

    if (original != NULL)  // imgRaw 변환 성공
    {
      isRecv_thermal = true;
      Q_EMIT recvImg_thermal();  // 이미지 수신을 알리는 시그널 발생
    }
  }
}

}  // namespace rescue_vision_ui
