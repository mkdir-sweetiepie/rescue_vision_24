#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include "../include/rescue_vision_24/findc_motion.hpp"

int main(int argc, char** argv)
{
  vision_rescue_24::FINDC_MOTION start(argc, argv);
  start.run();
}

namespace vision_rescue_24
{
using namespace cv;
using namespace std;
using namespace ros;

FINDC_MOTION::FINDC_MOTION(int argc, char** argv) : init_argc(argc), init_argv(argv), isRecv(false)
{
  init();
}

FINDC_MOTION::~FINDC_MOTION()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

bool FINDC_MOTION::init()
{
  ros::init(init_argc, init_argv, "qr");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  image_transport::ImageTransport img(n);
  img_result = img.advertise("/findc_motion_image", 100);
  n.getParam("/usb_cam/image_raw", param);
  ROS_INFO("Starting Rescue Vision With Camera : %s", param.c_str());
  img_sub = img.subscribe("/usb_cam/image_raw", 100, &FINDC_MOTION::imageCallBack, this);  /// camera/color/image_raw
  // Add your ros communications here.
  return true;
}

void FINDC_MOTION::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (isRecv == true)
    {
      update();
      img_result.publish(
          cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, clone_mat).toImageMsg());
    }
    loop_rate.sleep();
  }
}

void FINDC_MOTION::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img)
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

void FINDC_MOTION::update()
{
  clone_mat = original->clone();
  cv::resize(clone_mat, clone_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
  cv::cvtColor(clone_mat, gray_clone, cv::COLOR_BGR2GRAY);

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
        catch_c(center, two_radius);

      //cv::circle(clone_mat, center, two_radius, cv::Scalar(0, 255, 0), 2);  // 바깥원
      // cv::circle(clone_mat, center, 1, cv::Scalar(255, 0, 0), 1);           // 중심원
    }
  }

  cv::rectangle(clone_mat, cv::Rect(0, 0, 200, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  switch (averageAngle_i)
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
  }
  
  cv::rectangle(clone_mat, cv::Rect(640 - 200, 0, 640, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
  switch (direction_i)
  {
    case 1:
      cv::putText(clone_mat, "CW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
    case 0:
      cv::putText(clone_mat, "CCW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
      break;
  }

  circles.clear();
  delete original;  // 동적 할당된 원본 이미지 메모리 해제
  original = NULL;
  isRecv = false;  // 이미지 수신 플래그 재설정
}

// 원 부부만 자르기 ->  c_find 미션인지 구별하는 코드
void FINDC_MOTION::choose_circle(cv::Point center, int two_radius)
{
  two_mat = clone_mat(Range(c[1] - two_radius, c[1] + two_radius), Range(c[0] - two_radius, c[0] + two_radius));
  cv::resize(two_mat, two_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(two_mat, two_gray, cv::COLOR_BGR2GRAY);
  threshold(two_gray, two_binary, 80, 255, cv::THRESH_BINARY);
  two_binary = ~two_binary;
  find_two = check_black(two_binary);
}

// 첫번째 C 부터 순서대로 챚게 하는 코드
void FINDC_MOTION::catch_c(cv::Point center, int two_radius)
{
  int one_radius = 2.3 * two_radius;
  one_mat = clone_mat(Range(c[1] - one_radius, c[1] + one_radius), Range(c[0] - one_radius, c[0] + one_radius));
  cv::resize(one_mat, one_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(one_mat, one_gray, cv::COLOR_BGR2GRAY);
  threshold(one_gray, one_binary, 80, 255, cv::THRESH_BINARY);
  one_binary = ~one_binary;
  find_one = check_black(one_binary);

  int three_radius = 0.4167 * two_radius;
  three_mat =
      clone_mat(Range(c[1] - three_radius, c[1] + three_radius), Range(c[0] - three_radius, c[0] + three_radius));
  cv::resize(three_mat, three_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  cvtColor(three_mat, three_gray, cv::COLOR_BGR2GRAY);
  threshold(three_gray, three_binary, 80, 255, cv::THRESH_BINARY);
  three_binary = ~three_binary;

  // int four_radius = 0.1667 * two_radius;
  // four_mat = clone_mat(Range(c[1] - four_radius, c[1] + four_radius), Range(c[0] - four_radius, c[0] + four_radius));
  // cv::resize(four_mat, four_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  // cvtColor(four_mat, four_gray, cv::COLOR_BGR2GRAY);
  // threshold(four_gray, four_binary, 80, 255, cv::THRESH_BINARY);
  // four_binary = ~four_binary;

  // int five_radius = 0.05 * two_radius;
  // five_mat =
  //     clone_mat(Range(c[1] - five_radius, c[1] + five_radius), Range(c[0] - five_radius, c[0] + five_radius));
  // cv::resize(five_mat, five_mat, cv::Size(300, 300), 0, 0, cv::INTER_CUBIC);
  // cvtColor(five_mat, five_gray, cv::COLOR_BGR2GRAY);
  // threshold(five_gray, five_binary, 80, 255, cv::THRESH_BINARY);
  // five_binary = ~five_binary;

  if (find_one == true)
  {
    // 첫번째 원
    detect_way();
  }
}

void FINDC_MOTION::detect_way()
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

  // cout << "3: " << averageAngle3 << endl;
  // cout << endl;

  /* -----------------------------------------------------------------------------------*/
  int averageAngle1_i = averageAngle1;
  averageAngle_i = (averageAngle1_i + 22.5 * ((averageAngle1_i > 0) ? 1 : -1)) / 45.0;
  if(averageAngle_i == -4)
    averageAngle_i = 4;

  movement_count += 1;

  if (movement_count == 10)
  {
    first_i = averageAngle_i;
  }
  else if (movement_count == 50)
  {
    last_i = averageAngle_i;

    direction = first_i - last_i;
    cout << direction << endl;
    if (direction > 0)
      direction_i = 1;
    else if (direction < 0)
      direction_i = 0;
    else if (direction == 0)
      movement_count = 0;

    cv::rectangle(clone_mat, cv::Rect(640 - 200, 0, 640, 50), cv::Scalar(255, 255, 255), cv::FILLED, 8);
    switch (direction_i)
    {
      case 1:
        cv::putText(clone_mat, "CW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
      case 0:
        cv::putText(clone_mat, "CCW", cv::Point(640 - 200, 30), 0.5, 1, cv::Scalar(0, 0, 0), 2, 8);
        break;
    }
  }
  // namespace rescue_vision_cfind
  //-------------------------------------------------------three----------------------------------------------------
  // // (badly handpicked coords):
  // cv::Point cen3(150 + radius3 * cos(averageAngle3 / 180.0 * PI), 150 + radius3 * sin(-averageAngle3 / 180.0 *
  // PI)); int radius_roi3 = 5;
  // // out << "(" << cen2.x << "," << cen2.y << ")" << endl;
  // // get the Rect containing the circle:
  // cv::Rect r3(cen3.x - radius_roi3, cen3.y - radius_roi3, radius_roi3 * 2, radius_roi3 * 2);

  // // obtain the image ROI:
  // cv::Mat roi3(three_binary, r3);

  // // make a black mask, same size:
  // cv::Mat mask3(roi3.size(), roi3.type(), Scalar::all(0));
  // // with a white, filled circle in it:
  // cv::circle(mask3, Point(radius_roi3, radius_roi3), radius_roi3, Scalar::all(255), -1);

  // // combine roi & mask:
  // Mat eye_cropped3 = roi3 & mask3;
  // Scalar aver3 = mean(eye_cropped3);
  // // roi_img.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8,
  // // eye_cropped).toImageMsg());
  // // if (aver2[0] < 5 || 200 < aver2[0])
  // // {
  // //   return;
  // // }
  // // cout << aver2[0] << endl;

  //--------------------------------------------------fourth circle
  // check-----------------------------------------------
  // double sumAngles4 = 0.0;
  // int count4 = 0;
  // int radius4 = 125;  // 원의 반지름
  // double temp_radian4 = 0; double angleRadians4 = 0;

  // for (int y = 0; y < four_binary.rows; y++)
  // {
  //   for (int x = 0; x < four_binary.cols; x++)
  //   {
  //     if (four_binary.at<uchar>(y, x) == 0)
  //     {
  //       // 중심 좌표로부터의 거리 계산
  //       double distance4 = std::sqrt(std::pow(x - 150, 2) + std::pow(y - 150, 2));
  //       if (std::abs(distance4 - radius4) < 1.0)  // 거리가 125인 지점 판단
  //       {
  //         angleRadians4 = std::atan2(y - 150, x - 150) * 180.0 / CV_PI;
  //         if (abs(sumAngles4 / count4 - angleRadians4) > 180)
  //         {
  //           angleRadians4 -= 360;
  //         }
  //         sumAngles4 += angleRadians4;
  //         count4++;
  //       }
  //     }
  //   }
  //   if (abs(temp_radian4 - angleRadians4) > 180)
  //     break;
  // }

  // double averageAngle4 = 0;
  // if (count4 > 0)
  // {
  //   averageAngle4 = -(sumAngles4 / count4);
  //   if (averageAngle4 > 180)
  //   {
  //     averageAngle4 -= 360;
  //   }
  //   else if (averageAngle4 < -180)
  //   {
  //     averageAngle4 += 360;
  //   }
  // }

  // cout << "4: " << averageAngle4 << endl;

  //-------------------------------------------------------four----------------------------------------------------
  // // (badly handpicked coords):
  // cv::Point cen4(150 + radius4 * cos(averageAngle4 / 180.0 * PI), 150 + radius4 * sin(-averageAngle4 / 180.0 *
  // PI)); int radius_roi4 = 5;
  // // out << "(" << cen2.x << "," << cen2.y << ")" << endl;
  // // get the Rect containing the circle:
  // cv::Rect r4(cen4.x - radius_roi4, cen4.y - radius_roi4, radius_roi4 * 2, radius_roi4 * 2);

  // // obtain the image ROI:
  // cv::Mat roi4(four_binary, r4);

  // // make a black mask, same size:
  // cv::Mat mask4(roi4.size(), roi4.type(), Scalar::all(0));
  // // with a white, filled circle in it:
  // cv::circle(mask4, Point(radius_roi4, radius_roi4), radius_roi4, Scalar::all(255), -1);

  // // combine roi & mask:
  // Mat eye_cropped4 = roi4 & mask4;
  // Scalar aver4 = mean(eye_cropped4);
  // // roi_img.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8,
  // // eye_cropped).toImageMsg());
  // // if (aver2[0] < 5 || 200 < aver2[0])
  // // {
  // //   return;
  // // }
  // // cout << aver2[0] << endl;

  // //--------------------------------------------------fifth circle
  // check----------------------------------------------- double sumAngles5 = 0.0; int count5 = 0; int radius5 = 125;
  // //원의 반지름 double temp_radian5 = 0; double angleRadians5 = 0;

  // for (int y = 0; y < five_binary.rows; y++)
  // {
  //   for (int x = 0; x < five_binary.cols; x++)
  //   {
  //     if (five_binary.at<uchar>(y, x) == 0)
  //     {
  //       // 중심 좌표로부터의 거리 계산
  //       double distance5 = std::sqrt(std::pow(x - 150, 2) + std::pow(y - 150, 2));
  //       if (std::abs(distance5 - radius5) < 1.0)  // 거리가 125인 지점 판단
  //       {
  //         angleRadians5 = std::atan2(y - 150, x - 150) * 180.0 / CV_PI;
  //         if (abs(sumAngles5 / count5 - angleRadians5) > 180)
  //         {
  //           angleRadians5 -= 360;
  //         }
  //         sumAngles5 += angleRadians5;
  //         count5++;
  //       }
  //     }
  //   }
  //   if (abs(temp_radian5 - angleRadians5) > 180)
  //     break;
  // }

  // double averageAngle5 = 0;
  // if (count5 > 0)
  // {
  //   averageAngle5 = -(sumAngles5 / count5);
  //   if (averageAngle5 > 180)
  //   {
  //     averageAngle5 -= 360;
  //   }
  //   else if (averageAngle5 < -180)
  //   {
  //     averageAngle5 += 360;
  //   }
  // }

  // cout << "5: " << averageAngle5 << endl;
}

// c 라는걸 판별 해줌 예를 들어 c 모양이라고 한다면 위 아래 왼쪽은 cnt++ 3번이 되기 때문에 c리는걸 알수 있음
bool FINDC_MOTION::check_black(const Mat& binary_mat)
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