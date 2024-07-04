#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
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

  // 이미지 정보 ================================================
  Mat* original;
  Mat* original_thermal;
  Mat clone_mat;
  Mat thermal_mat;
  Mat gray_clone;
  bool isRecv;
  bool isRecv_thermal;
  void update();
  void set_thermal();

  // hazmat 정보 ================================================
  Mat frame;
  Mat blob;
  cv::dnn::Net net;
  std::vector<std::string> class_names;
  bool isOverlapping;  // 겹침 여부 플래그
  bool isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2);
  void set_hazmat();

  // qr 정보 ================================================
  Mat output_qr;  // qr
  vector<Point> points;
  vector<cv::Rect> qr_boxes;
  String info;
  QRCodeDetector detector;
  void set_qr();

  // c 정보 ================================================
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

  int averageAngle_i;
  int movement_count = 0;
  int previous_value = 0;
  int count_same_value = 0;
  int first_i = 0;
  int last_i = 0;
  int direction = 0;
  int direction_i = 2;
  std::map<int, int> angleFrequency1;
  std::map<int, int> angleFrequency2;
  std::map<int, int> angleFrequency3;
  int maxFrequencyAngle1 = 0;  // 최다 빈도 각도
  int maxFrequencyAngle2 = 0;
  int maxFrequencyAngle3 = 0;
  int maxFrequency1= 0;       // 최다 빈도 값
  int maxFrequency2= 0;
  int maxFrequency3= 0;
  int result_maxAngle1;
  int result_maxAngle2;
  int result_maxAngle3;

  void choose_circle(Point center, int two_radius);
  void catch_c(Point center, int two_radius);
  void detect_way();
  bool check_black(const Mat& binary_mat);
  void set_c();
  //========================================
  std::string victim_start = "";
  bool qr_flag = false;
  bool c_flag = false;

  int averageAngle_1;
  int averageAngle_2;
  int averageAngle_3;

  // cam 정보 (1)realsense (2)usbcam ========================
  std::string cam1_topic_name;
  std::string cam2_topic_name;

  std_msgs::Int32 msg1, msg2;

private:
  int init_argc;
  char** init_argv;

  image_transport::Publisher img_result;
  image_transport::Publisher img_result_thermal;

  image_transport::Subscriber img_sub;
  image_transport::Subscriber img_sub_thermal;
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img);
  void imageCallBack_thermal(const sensor_msgs::ImageConstPtr& msg_img);

};

}  // namespace vision_rescue_24