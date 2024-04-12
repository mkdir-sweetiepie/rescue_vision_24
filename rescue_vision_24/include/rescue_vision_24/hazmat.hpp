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

using namespace cv;
using namespace std;

namespace vision_rescue_24
{
class HAZMAT
{
public:
  HAZMAT(int argc, char** argv);
  ~HAZMAT();

  bool init();
  void run();
  void update();
  bool isRecv;

  Mat* original;
  Mat clone_mat;
  Mat frame; //output_hazmat
  Mat blob;

  cv::dnn::Net net;
  std::vector<std::string> class_names;
  bool isOverlapping;  // 겹침 여부 플래그
  bool isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2);
  void set_yolo();

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