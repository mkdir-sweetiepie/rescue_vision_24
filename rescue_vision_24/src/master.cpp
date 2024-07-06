#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <ros/package.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <map>
#include "std_msgs/Int32.h"

#include "../include/rescue_vision_24/master.hpp"

constexpr float CONFIDENCE_THRESHOLD = 0.52;  // 확률 경계값
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 15;
const cv::Scalar colors[] = { { 0, 255, 255 }, { 255, 255, 0 }, { 0, 255, 0 }, { 255, 0, 0 } };
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

#define PI 3.141592

int main(int argc, char** argv)
{
  vision_rescue_24::MASTER start(argc, argv);
  start.run();
}

namespace vision_rescue_24
{
using namespace cv;
using namespace std;
using namespace ros;

MASTER::MASTER(int argc, char** argv) : init_argc(argc), init_argv(argv), isRecv(false), isRecv_thermal(false)
{
  std::string packagePath = ros::package::getPath("rescue_vision_24");
  cout << packagePath << endl;
  std::string dir = packagePath + "/yolo/";
  {
    std::ifstream class_file(dir + "classes.txt");
    if (!class_file)
    {
      std::cerr << "failed to open classes.txt\n";
    }

    std::string line;
    while (std::getline(class_file, line))
      class_names.push_back(line);
  }

  std::string modelConfiguration = dir + "yolov7_tiny_hazmat.cfg";
  std::string modelWeights = dir + "2023_06_29.weights";

  net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
  //     net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
  //     net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  init();
}

MASTER::~MASTER()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

bool MASTER::init()
{
  ros::init(init_argc, init_argv, "master");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  image_transport::ImageTransport img(n);

  // Add your ros communications here.
  img_result = img.advertise("/victim_image", 1);
  img_result_thermal = img.advertise("/img_result_thermal", 1);

  // cam 정보
  n.param<std::string>("cam1_topic", cam1_topic_name, "/camera/color/image_raw");  // realsense
  n.param<std::string>("cam2_topic", cam2_topic_name, "/usb_cam/image_raw");       // usbcam
  ROS_INFO("Starting Rescue Vision With Camera : %s", cam1_topic_name.c_str());

  img_sub = img.subscribe(cam1_topic_name, 1, &MASTER::imageCallBack, this);  // camera/color/image_raw
  img_sub_thermal = img.subscribe("/thermal_camera/image_colored", 1, &MASTER::imageCallBack_thermal, this);
  ROS_INFO("Starting Rescue Vision With Camera : %s", "/thermal_camera/image_colored");
  return true;
}

void MASTER::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (isRecv == true)
    {
      if (isRecv_thermal == true)
      {
        set_thermal();
        img_result_thermal.publish(
            cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, thermal_mat).toImageMsg());
      }
      // victim_start = "start";
      update();
    }
  }
}

void MASTER::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img)
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

void MASTER::imageCallBack_thermal(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (!isRecv_thermal)
  {
    original_thermal = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
    if (original_thermal != NULL)
    {
      isRecv_thermal = true;
    }
  }
}

void MASTER::set_thermal()
{
  thermal_mat = original_thermal->clone();
  cv::resize(thermal_mat, thermal_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
  delete original_thermal;
  isRecv_thermal = false;
}

void MASTER::update()
{
  clone_mat = original->clone();
  cv::resize(clone_mat, clone_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
  cv::rectangle(clone_mat, cv::Rect(0, 0, 150, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  cv::rectangle(clone_mat, cv::Rect(0, 51, 150, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  cv::rectangle(clone_mat, cv::Rect(0, 101, 150, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  cv::rectangle(clone_mat, cv::Rect(640 - 200, 0, 640, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  switch (result_maxAngle1)
  {
    case -4:
      cv::putText(clone_mat, "left", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -3:
      cv::putText(clone_mat, "left_down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -2:
      cv::putText(clone_mat, "down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -1:
      cv::putText(clone_mat, "right_down", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 0:
      cv::putText(clone_mat, "right", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 1:
      cv::putText(clone_mat, "right_up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 2:
      cv::putText(clone_mat, "up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 3:
      cv::putText(clone_mat, "left_up", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 4:
      cv::putText(clone_mat, "left", cv::Point(0, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    default:
      break;
  }
  switch (result_maxAngle2)
  {
    case -4:
      cv::putText(clone_mat, "left", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -3:
      cv::putText(clone_mat, "left_down", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -2:
      cv::putText(clone_mat, "down", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -1:
      cv::putText(clone_mat, "right_down", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 0:
      cv::putText(clone_mat, "right", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 1:
      cv::putText(clone_mat, "right_up", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 2:
      cv::putText(clone_mat, "up", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 3:
      cv::putText(clone_mat, "left_up", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 4:
      cv::putText(clone_mat, "left", cv::Point(0, 81), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    default:
      break;
  }
  switch (result_maxAngle3)
  {
    case -4:
      cv::putText(clone_mat, "left", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -3:
      cv::putText(clone_mat, "left_down", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -2:
      cv::putText(clone_mat, "down", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case -1:
      cv::putText(clone_mat, "right_down", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 0:
      cv::putText(clone_mat, "right", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 1:
      cv::putText(clone_mat, "right_up", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 2:
      cv::putText(clone_mat, "up", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 3:
      cv::putText(clone_mat, "left_up", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 4:
      cv::putText(clone_mat, "left", cv::Point(0, 131), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    default:
      break;
  }
  switch (direction_i)
  {
    case 1:
      cv::putText(clone_mat, "CW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 0:
      cv::putText(clone_mat, "CCW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    default:
      break;
  }

  set_qr();
  set_hazmat();
  set_c();

  delete original;
  isRecv = false;
}
// =========================================================================================
void MASTER::set_hazmat()
{
  auto output_names = net.getUnconnectedOutLayersNames();
  clone_mat = clone_mat.clone();
  std::vector<cv::Mat> detections;

  auto total_start = std::chrono::steady_clock::now();
  cv::dnn::blobFromImage(clone_mat, blob, 0.00392, cv::Size(416, 416), cv::Scalar(), true, false, CV_32F);
  net.setInput(blob);

  auto dnn_start = std::chrono::steady_clock::now();
  net.forward(detections, output_names);
  auto dnn_end = std::chrono::steady_clock::now();

  std::vector<int> indices[NUM_CLASSES];
  std::vector<cv::Rect> boxes[NUM_CLASSES];
  std::vector<float> scores[NUM_CLASSES];

  for (auto& output : detections)
  {
    const auto num_boxes = output.rows;
    for (int i = 0; i < num_boxes; i++)
    {
      auto x = output.at<float>(i, 0) * clone_mat.cols;  // 중심 x
      auto y = output.at<float>(i, 1) * clone_mat.rows;  // 중심 y
      auto width = output.at<float>(i, 2) * clone_mat.cols;
      auto height = output.at<float>(i, 3) * clone_mat.rows;
      cv::Rect rect(x - width / 2, y - height / 2, width, height);

      for (int c = 0; c < NUM_CLASSES; c++)
      {
        auto confidence = *output.ptr<float>(i, 5 + c);
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
          boxes[c].push_back(rect);
          scores[c].push_back(confidence);
        }
      }
    }
  }

  for (int c = 0; c < NUM_CLASSES; c++)
    cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);

  for (int c = 0; c < NUM_CLASSES; c++)
  {
    for (size_t i = 0; i < indices[c].size(); ++i)
    {
      const auto color = colors[c % NUM_COLORS];
      auto idx = indices[c][i];
      const auto& rect = boxes[c][idx];

      isOverlapping = false;
      if (indices[c].size() != 0)
      {
        for (size_t j = 0; j < indices[c].size(); ++j)
        {
          if (j != i)
          {
            auto idx2 = indices[c][j];
            const auto& rect2 = boxes[c][idx2];
            if (isRectOverlapping(rect, rect2))
            {
              if (rect2.area() < rect.area())
              {
                isOverlapping = true;
                break;
              }
              else
              {
                continue;
              }
            }
          }
        }
      }

      // QR 박스와 겹치는지 확인
      for (const auto& qr_rect : qr_boxes)
      {
        if (isRectOverlapping(rect, qr_rect))
        {
          isOverlapping = true;
          break;
        }
      }

      if (!isOverlapping)
      {
        cv::rectangle(clone_mat, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color,
                      3);

        std::ostringstream label_ss;
        label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
        auto label = label_ss.str();

        int baseline;
        auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);

        int label_x = rect.x;
        if (label_x + label_bg_sz.width > clone_mat.cols)
        {
          label_x = clone_mat.cols - label_bg_sz.width - 10;
        }
        if (label_x < 0)
        {
          label_x = 10;
        }

        if ((rect.y - label_bg_sz.height - baseline - 10) >= 0)
        {
          cv::rectangle(clone_mat, cv::Point(label_x, rect.y - label_bg_sz.height - baseline - 10),
                        cv::Point(label_x + label_bg_sz.width, rect.y), cv::Scalar(255, 255, 255), cv::FILLED);
          cv::putText(clone_mat, label.c_str(), cv::Point(label_x, rect.y - baseline - 5),
                      cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
        }
        else
        {
          cv::rectangle(
              clone_mat, cv::Point(label_x, rect.y + rect.height),
              cv::Point(label_x + label_bg_sz.width, rect.y + rect.height + label_bg_sz.height + baseline + 10),
              cv::Scalar(255, 255, 255), cv::FILLED);
          cv::putText(clone_mat, label.c_str(), cv::Point(label_x, rect.y + rect.height + baseline + 5),
                      cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
        }
        if (qr_flag == false)
          img_result.publish(
              cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, clone_mat).toImageMsg());
      }
    }
  }
}

bool MASTER::isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2)
{
  int x1 = std::max(rect1.x, rect2.x);
  int y1 = std::max(rect1.y, rect2.y);
  int x2 = std::min(rect1.x + rect1.width, rect2.x + rect2.width);
  int y2 = std::min(rect1.y + rect1.height, rect2.y + rect2.height);

  // If the intersection area is positive, the rectangles overlap
  if (x1 < x2 && y1 < y2)
    return true;
  else
    return false;
}
// ===========================================================================================

void MASTER::set_qr()
{
  output_qr = clone_mat.clone();
  cv::cvtColor(clone_mat, gray_clone, COLOR_BGR2GRAY);

  if (detector.detect(gray_clone, points))
  {
    info = detector.decode(gray_clone, points);  // QR 코드 해독하여 정보 추출
    qr_flag = true;
    if (!info.empty())  // 정보가 비어 있지 않은 경우
    {
      cv::Size text_size = cv::getTextSize(info, cv::FONT_HERSHEY_SIMPLEX, 1, 2, nullptr);
      cv::Point2i bg_top_left(points[0].x / 2, points[0].y - text_size.height - 20);
      cv::Point2i bg_bottom_right(bg_top_left.x + text_size.width, bg_top_left.y + text_size.height + 5);

      cv::Rect qr_rect(bg_top_left, bg_bottom_right);
      qr_boxes.push_back(qr_rect);

      cv::rectangle(output_qr, bg_top_left, bg_bottom_right, cv::Scalar(0, 0, 0), -1);  // -1은 사각형을 채우라는 의미
      cv::putText(output_qr, info, cv::Point(points[0].x / 2, points[0].y - 20), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar(255, 255, 255), 2);
      cv::polylines(output_qr, points, true, cv::Scalar(0, 0, 0), 5);
      // cout << info << endl;
      // cv::putText(clone_mat, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 3);
      // std::cout << info << std::endl;  // 콘솔에 QR 코드 정보 출력
    }
    if (qr_flag == true)
      img_result.publish(
          cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, output_qr).toImageMsg());
  }
  else
  {
    qr_flag = false;
  }
}
// ===========================================================================================

void MASTER::set_c()
{
  output_c = clone_mat.clone();
  cv::cvtColor(output_c, gray_clone, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray_clone, gray_clone, cv::Size(5, 5), 0);
  cv::Canny(gray_clone, gray_clone, 100, 300);
  cv::HoughCircles(gray_clone, circles, cv::HOUGH_GRADIENT, 1, gray_clone.rows / 8, 200, 50, 10,
                   120);  // 1: 입력 이미지와 같은 해상도

  for (size_t i = 0; i < circles.size(); i++)
  {
    // cvRound 함수는 부동 소수점 숫자를 가장 가까운 정수로 반올림 -> cv::circle 함수가 정수형 좌표를 요구
    c = circles[i];
    cv::Point center(cvRound(c[0]), cvRound(c[1]));
    two_radius = cvRound(c[2]);

    // 작은 원 반지름 : 큰 원 반지름 -> 1.2 : 2.8 -> 2.8 / 1.2 = 2.3
    /* 큰 원의 범위 내에 위치한 작은 원들을 처리하는 역할(choose_circle에서 작은원 처리)
       작은 원이 큰 원의 범위를 벗어나는 경우에는 처리를 건너뜁니다. */
    if (!(((c[1] - 2.3 * two_radius) < 0) || ((c[1] + 2.3 * two_radius) > 480) || ((c[0] - 2.3 * two_radius) < 0) ||
          ((c[0] + 2.3 * two_radius) > 640)))
    {  // 300, 400
      choose_circle(center, two_radius);
      if (find_two == true)
      {
        catch_c(center, two_radius);

        cv::circle(output_c, center, two_radius, cv::Scalar(0, 255, 0), 2);  // 바깥원
        // cv::circle(clone_mat, center, 1, cv::Scalar(255, 0, 0), 1);           // 중심원
      }
    }

    if (qr_flag == false)
      img_result.publish(
          cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, output_c).toImageMsg());
  }
  circles.clear();
}

// 원 부부만 자르기 ->  c_find 미션인지 구별하는 코드
void MASTER::choose_circle(cv::Point center, int two_radius)
{
  two_mat = output_c(Range(c[1] - two_radius, c[1] + two_radius), Range(c[0] - two_radius, c[0] + two_radius));
  cv::resize(two_mat, two_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(two_mat, two_gray, cv::COLOR_BGR2GRAY);
  threshold(two_gray, two_binary, 80, 255, cv::THRESH_BINARY);
  two_binary = ~two_binary;
  find_two = check_black(two_binary);
}

// 첫번째 C 부터 순서대로 챚게 하는 코드
void MASTER::catch_c(cv::Point center, int two_radius)
{
  int one_radius = 2.3 * two_radius;
  one_mat = output_c(Range(c[1] - one_radius, c[1] + one_radius), Range(c[0] - one_radius, c[0] + one_radius));
  // cv::rectangle(output_c, cv::Point(c[0] - one_radius, c[1] - one_radius),
  //               cv::Point(c[0] + one_radius, c[1] + one_radius), cv::Scalar(0, 255, 0), 2);
  cv::resize(one_mat, one_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(one_mat, one_gray, cv::COLOR_BGR2GRAY);
  threshold(one_gray, one_binary, 80, 255, cv::THRESH_BINARY);
  one_binary = ~one_binary;
  find_one = check_black(one_binary);

  int three_radius = 0.4167 * two_radius;
  three_mat =
      output_c(Range(c[1] - three_radius, c[1] + three_radius), Range(c[0] - three_radius, c[0] + three_radius));
  cv::resize(three_mat, three_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(three_mat, three_gray, cv::COLOR_BGR2GRAY);
  threshold(three_gray, three_binary, 80, 255, cv::THRESH_BINARY);
  three_binary = ~three_binary;

  if (find_one)
  {
    detect_way();

    find_one = false;
  }
}

void MASTER::detect_way()
{
  //--------------------------------------------------first circle check-----------------------------------------------
  double sumAngles1 = 0.0;
  int count1 = 0;
  int radius1 = 125;  //원의 반지름
  double temp_radian1 = 0;
  double angleRadians1 = 0;

  for (int y = 0; y < one_binary.rows; y++)
  {
    for (int x = 0; x < one_binary.cols; x++)
    {
      if (one_binary.at<uchar>(y, x) == 0)
      {
        // 중심 좌표로부터의 거리 계산
        double distance1 = std::sqrt(std::pow(x - 150, 2) + std::pow(y - 150, 2));
        if (std::abs(distance1 - radius1) < 1.0)  // 거리가 125인 지점 판단
        {
          angleRadians1 = std::atan2(y - 150, x - 150) * 180.0 / CV_PI;
          if (abs(sumAngles1 / count1 - angleRadians1) > 180)
          {
            angleRadians1 -= 360;
          }
          sumAngles1 += angleRadians1;
          count1++;
        }
      }
    }
    if (abs(temp_radian1 - angleRadians1) > 180)
      break;
  }

  double averageAngle1 = 0;
  if (count1 > 0)
  {
    averageAngle1 = -(sumAngles1 / count1);
    if (averageAngle1 > 180)
    {
      averageAngle1 -= 360;
    }
    else if (averageAngle1 < -180)
    {
      averageAngle1 += 360;
    }
  }
  // cout << "1: " << averageAngle1 << endl;

  //-------------------------------------------------------one----------------------------------------------------
  // (badly handpicked coords):
  cv::Point cen(150 + radius1 * cos(averageAngle1 / 180.0 * PI), 150 + radius1 * sin(-averageAngle1 / 180.0 * PI));
  int radius_roi = 20;

  // get the Rect containing the circle:
  cv::Rect r(cen.x - radius_roi, cen.y - radius_roi, radius_roi * 2, radius_roi * 2);

  // obtain the image ROI:
  cv::Mat roi(one_binary, r);

  // make a black mask, same size:
  cv::Mat mask(roi.size(), roi.type(), Scalar::all(0));
  // with a white, filled circle in it:
  cv::circle(mask, Point(radius_roi, radius_roi), radius_roi, Scalar::all(255), -1);

  // combine roi & mask:
  Mat eye_cropped = roi & mask;
  Scalar aver = mean(eye_cropped);
  // roi_img.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8,
  // eye_cropped).toImageMsg());
  if (aver[0] < 5 || 200 < aver[0])
  {
    return;
  }

  //--------------------------------------------------second circle check-----------------------------------------------
  double sumAngles2 = 0.0;
  int count2 = 0;
  int radius2 = 125;  // 원의 반지름
  double temp_radian2 = 0;
  double angleRadians2 = 0;

  for (int y = 0; y < two_binary.rows; y++)
  {
    for (int x = 0; x < two_binary.cols; x++)
    {
      if (two_binary.at<uchar>(y, x) == 0)
      {
        // 중심 좌표로부터의 거리 계산
        double distance2 = std::sqrt(std::pow(x - 150, 2) + std::pow(y - 150, 2));

        if (std::abs(distance2 - radius2) < 1.0)  // 거리가 125인 지점 판단
        {
          angleRadians2 = std::atan2(y - 150, x - 150) * 180.0 / CV_PI;
          if (abs(sumAngles2 / count2 - angleRadians2) > 180)
          {
            angleRadians2 -= 360;
          }
          sumAngles2 += angleRadians2;
          count2++;
        }
      }
    }
    if (abs(temp_radian2 - angleRadians2) > 180)
      break;
  }

  double averageAngle2 = 0;
  if (count2 > 0)
  {
    averageAngle2 = -(sumAngles2 / count2);
    if (averageAngle2 > 180)
    {
      averageAngle2 -= 360;
    }
    else if (averageAngle2 < -180)
    {
      averageAngle2 += 360;
    }
  }

  // cout << "2: " << averageAngle2 << endl;

  //-------------------------------------------------------two----------------------------------------------------
  // (badly handpicked coords):
  cv::Point cen2(150 + radius2 * cos(averageAngle2 / 180.0 * PI), 150 + radius2 * sin(-averageAngle2 / 180.0 * PI));
  int radius_roi2 = 10;
  // out << "(" << cen2.x << "," << cen2.y << ")" << endl;
  // get the Rect containing the circle:
  cv::Rect r2(cen2.x - radius_roi2, cen2.y - radius_roi2, radius_roi2 * 2, radius_roi2 * 2);

  // obtain the image ROI:
  cv::Mat roi2(two_binary, r2);

  // make a black mask, same size:
  cv::Mat mask2(roi2.size(), roi2.type(), Scalar::all(0));
  // with a white, filled circle in it:
  cv::circle(mask2, Point(radius_roi2, radius_roi2), radius_roi2, Scalar::all(255), -1);

  // combine roi & mask:
  Mat eye_cropped2 = roi2 & mask2;
  Scalar aver2 = mean(eye_cropped2);
  // roi_img.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8,
  // eye_cropped).toImageMsg());
  // if (aver2[0] < 5 || 200 < aver2[0])
  // {
  //   return;
  // }
  // cout << aver2[0] << endl;

  //--------------------------------------------------third circle check-----------------------------------------------
  double sumAngles3 = 0.0;
  int count3 = 0;
  int radius3 = 125;  //원의 반지름
  double temp_radian3 = 0;
  double angleRadians3 = 0;

  for (int y = 0; y < three_binary.rows; y++)
  {
    for (int x = 0; x < three_binary.cols; x++)
    {
      if (three_binary.at<uchar>(y, x) == 0)
      {
        // 중심 좌표로부터의 거리 계산
        double distance3 = std::sqrt(std::pow(x - 150, 2) + std::pow(y - 150, 2));
        if (std::abs(distance3 - radius3) < 1.0)  // 거리가 125인 지점 판단
        {
          angleRadians3 = std::atan2(y - 150, x - 150) * 180.0 / CV_PI;
          if (abs(sumAngles3 / count3 - angleRadians3) > 180)
          {
            angleRadians3 -= 360;
          }
          sumAngles3 += angleRadians3;
          count3++;
        }
      }
    }
    if (abs(temp_radian3 - angleRadians3) > 180)
      break;
  }

  double averageAngle3 = 0;
  if (count3 > 0)
  {
    averageAngle3 = -(sumAngles3 / count3);
    if (averageAngle3 > 180)
    {
      averageAngle3 -= 360;
    }
    else if (averageAngle3 < -180)
    {
      averageAngle3 += 360;
    }
  }
  int averageAngle1_i = averageAngle1;
  int averageAngle2_i = averageAngle2;
  int averageAngle3_i = averageAngle3;
  // averageAngle1_i = (90 - averageAngle2) + averageAngle1;
  // if (averageAngle1_i > 180)
  //   averageAngle1_i -= 360;
  // averageAngle2_i = (90 - averageAngle3) + averageAngle1;
  // if (averageAngle2_i > 180)
  //   averageAngle2_i -= 360;

  averageAngle_1 = (averageAngle1_i + 22.5 * ((averageAngle1_i > 0) ? 1 : -1)) / 45.0;
  averageAngle_2 = (averageAngle2_i + 22.5 * ((averageAngle2_i > 0) ? 1 : -1)) / 45.0;
  averageAngle_3 = (averageAngle3_i + 22.5 * ((averageAngle3_i > 0) ? 1 : -1)) / 45.0;

  if (averageAngle_1 == -4)
    averageAngle_1 = 4;
  if (averageAngle_2 == -4)
    averageAngle_2 = 4;
  if (averageAngle_3 == -4)
    averageAngle_3 = 4;

  // 키면 거의 바로 인식해서 세리는 스타트 플래그 필요함

  if (movement_count <= 21)
    movement_count += 1;

  if (movement_count < 20)
  {
    angleFrequency1[averageAngle_1]++;
    if (angleFrequency1[averageAngle_1] > maxFrequency1)
    {
      maxFrequency1 = angleFrequency1[averageAngle_1];
      maxFrequencyAngle1 = averageAngle_1;
    }
    angleFrequency2[averageAngle_2]++;
    if (angleFrequency2[averageAngle_2] > maxFrequency2)
    {
      maxFrequency2 = angleFrequency2[averageAngle_2];
      maxFrequencyAngle2 = averageAngle_2;
    }
    angleFrequency3[averageAngle_3]++;
    if (angleFrequency3[averageAngle_3] > maxFrequency3)
    {
      maxFrequency3 = angleFrequency3[averageAngle_3];
      maxFrequencyAngle3 = averageAngle_3;
    }
  }
  cout << movement_count << endl;
  // cout << maxFrequencyAngle << endl;

  if (movement_count == 20)
  {
    result_maxAngle1 = maxFrequencyAngle1;
    result_maxAngle2 = maxFrequencyAngle2;
    result_maxAngle3 = maxFrequencyAngle3;
    first_i = averageAngle_1;
    if (first_i == 4 || first_i == -3 || first_i == maxFrequencyAngle1)
    {
      movement_count -= 5;
    }

    cout << first_i << endl;
  }

  if (movement_count > 20 && c_flag == false)
  {
    last_i = averageAngle_1;
    if (last_i == maxFrequencyAngle1)
    {
      movement_count++;
    }
    else
    {
      cout << last_i << endl;
      direction = first_i - last_i;
      // cout << direction << endl;
      if (direction > 0)
      {
        direction_i = 1;
        c_flag = true;
      }

      else if (direction < 0)
      {
        direction_i = 0;
        c_flag = true;
      }
      else if (direction == 0)
      {
        cout << "stop" << endl;
        movement_count++;
      }
    }

    // cout << "3: " << averageAngle3 << endl;
    // cout << endl;
  }
}

// c 라는걸 판별 해줌 예를 들어 c 모양이라고 한다면 위 아래 왼쪽은 cnt++ 3번이 되기 때문에 c리는걸 알수 있음
bool MASTER::check_black(const Mat& binary_mat)
{
  int cnt = 0;

  bool up = binary_mat.at<uchar>(20, 150);
  bool left = binary_mat.at<uchar>(150, 20);
  bool down = binary_mat.at<uchar>(280, 150);
  bool right = binary_mat.at<uchar>(150, 280);

  if (up == 1)
    cnt++;
  if (left == 1)
    cnt++;
  if (right == 1)
    cnt++;
  if (down == 1)
    cnt++;
  // cout << up << " " << left << " " << down << " " << right << endl;

  if (cnt >= 3)
    return true;
  else
    return false;
}

}  // namespace vision_rescue_24
