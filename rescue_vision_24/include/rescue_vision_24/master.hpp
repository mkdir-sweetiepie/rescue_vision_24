#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <map>

using namespace cv;
using namespace std;

namespace vision_rescue_24
{
class MASTER
{
public:
  MASTER(int argc, char** argv);
  ~MASTER();

  bool init();
  void run();
  void update();
  bool isRecv;

  Mat* original;
  Mat clone_mat;
  Mat gray_clone;

  Mat frame;
  ;  // hazmat
  Mat blob;
  cv::dnn::Net net;
  std::vector<std::string> class_names;
  bool isOverlapping;  // 겹침 여부 플래그
  bool isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2);
  void set_hazmat();
  //================================================

  Mat output_qr;  // qr
  vector<Point> points;
  QRCodeDetector detector;
  void set_qr();
  //================================================

  bool exist_moving = false;  // c

  Mat output_c;
  Vec3i c;
  vector<Vec3f> circles;  //(중심좌표x, 중심좌표y, 반지름r)
  int two_radius;

  Mat two_mat;
  Mat two_gray;
  Mat two_binary;
  bool find_two = false;

  Mat one_mat;
  Mat one_gray;
  Mat one_binary;
  bool find_one = false;

  Mat three_mat;
  Mat three_gray;
  Mat three_binary;

  Mat four_mat;
  Mat four_gray;
  Mat four_binary;

  Mat five_mat;
  Mat five_gray;
  Mat five_binary;

  int averageAngle_i;
  int movement_count = 0;
  int previous_value = 0;
  int count_same_value = 0;
  int first_i = 0;
  int last_i = 0;
  int direction = 0;
  int direction_i;
  std::map<int, int> angleFrequency;
  int maxFrequencyAngle = 0;  // 최다 빈도 각도
  int maxFrequency = 0;       // 최다 빈도 값
  int result_maxAngle;

  void choose_circle(Point center, int two_radius);
  void catch_c(Point center, int two_radius);
  void detect_way();
  bool check_black(const Mat& binary_mat);
  void check_moving(double averageAngle1);
  void set_c();
  //========================================
  bool victim_start = true;
  bool qr_flag = false;
  bool c_flag = false;

  std::string param;
  String info;

  std_msgs::Int32 msg1, msg2;

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber img_sub;
  image_transport::Publisher img_result;
  ros::Publisher c_result1_pub;
  ros::Publisher c_result2_pub;
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace vision_rescue_24