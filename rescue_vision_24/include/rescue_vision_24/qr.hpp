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

namespace vision_rescue_24
{
class QR
{
public:
  QR(int argc, char** argv);
  ~QR();

  bool init();
  void run();
  void update();
  bool isRecv;

  Mat* original;
  Mat clone_mat;
  Mat gray_clone;
  Mat output_qr;

  vector<Point> points;
  QRCodeDetector detector;

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