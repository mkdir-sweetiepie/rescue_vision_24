#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include "../include/rescue_vision_24/qr.hpp"

int main(int argc, char** argv)
{
  vision_rescue_24::QR start(argc, argv);
  start.run();
}

namespace vision_rescue_24
{
using namespace cv;
using namespace std;
using namespace ros;

QR::QR(int argc, char** argv) : init_argc(argc), init_argv(argv), isRecv(false)
{
  init();
}

QR::~QR()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

bool QR::init()
{
  ros::init(init_argc, init_argv, "qr");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  image_transport::ImageTransport img(n);
  img_result = img.advertise("/qr_image", 100);
  n.getParam("/usb_cam/image_raw", param);
  ROS_INFO("Starting Rescue Vision With Camera : %s", param.c_str());
  img_sub = img.subscribe("/usb_cam/image_raw", 100, &QR::imageCallBack, this);  /// camera/color/image_raw
  // Add your ros communications here.
  return true;
}

void QR::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (isRecv == true)
    {
      update();
      img_result.publish(
          cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, output_qr).toImageMsg());
    }
    loop_rate.sleep();
  }
}

void QR::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (!isRecv)
  {
    original = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
    if (original != NULL)
    {
      isRecv = true;
    }
  }
}

void QR::update()
{
  clone_mat = original->clone();
  resize(clone_mat, clone_mat, Size(640, 480), 0, 0, INTER_CUBIC);
  cvtColor(clone_mat, gray_clone, COLOR_BGR2GRAY);

  output_qr = clone_mat.clone();

  if (detector.detect(gray_clone, points))
  {
    info = detector.decode(gray_clone, points);  // QR 코드 해독하여 정보 추출

    if (!info.empty())  // 정보가 비어 있지 않은 경우
    {
      cv::Size text_size = cv::getTextSize(info, cv::FONT_HERSHEY_SIMPLEX, 1, 2, nullptr);
      cv::Point2i bg_top_left(points[0].x, points[0].y - text_size.height - 20);
      cv::Point2i bg_bottom_right(bg_top_left.x + text_size.width, bg_top_left.y + text_size.height + 5);

      cv::rectangle(output_qr, bg_top_left, bg_bottom_right, cv::Scalar(0, 0, 0), -1);  // -1은 사각형을 채우라는 의미
      cv::putText(output_qr, info, cv::Point(points[0].x, points[0].y - 20), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar(255, 255, 255), 2);
      cv::polylines(output_qr, points, true, cv::Scalar(0, 0, 0), 5);
      cout << info << endl;
      // cv::putText(clone_mat, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 3);
      // std::cout << info << std::endl;  // 콘솔에 QR 코드 정보 출력
    }
  }

  delete original;
  isRecv = false;
}
}  // namespace vision_rescue_24