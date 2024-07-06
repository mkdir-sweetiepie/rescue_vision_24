#ifndef VICTIM_REQUEST_MANAGER_H
#define VICTIM_REQUEST_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdlib>

class VICTIM_REQUEST_MANAGER
{
public:
  VICTIM_REQUEST_MANAGER();

private:
  bool runRequest(const std::string& command, const std::string& type);

  // ui로부터 /victim_start 받음
  ros::Subscriber request_sub_;
  void requestCallback(const std_msgs::String::ConstPtr& msg);

  // /victim_end를 ui에게 보냄
  ros::Publisher response_pub_;
};

#endif  // VICTIM_REQUEST_MANAGER_H