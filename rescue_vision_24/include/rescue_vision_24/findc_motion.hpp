#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
#define PI 3.141592  // pi

namespace vision_rescue_24
{
class FINDC_MOTION
{
public:
  FINDC_MOTION(int argc, char** argv);
  ~FINDC_MOTION();

  bool init();
  void run();
  void update();
  bool isRecv;

  Mat* original;
  Mat clone_mat;  // output_findc_motion
  Mat gray_clone;

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

  int averageAngle_i = 0;
  int movement_count = 0;
  int previous_value = 0;
  int count_same_value = 0;
  int first_i = 0;
  int last_i = 0;
  int direction = 0;
  int direction_i;

  void choose_circle(Point center, int two_radius);
  void catch_c(Point center, int two_radius);
  void detect_way();
  bool check_black(const Mat& binary_mat);

  std::string param;
  String info;

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber img_sub;
  image_transport::Publisher img_result;
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace vision_rescue_24