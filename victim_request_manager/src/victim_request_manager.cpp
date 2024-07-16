#include "../include/victim_request_manager/victim_request_manager.hpp"

VICTIM_REQUEST_MANAGER::VICTIM_REQUEST_MANAGER()
{
  ros::NodeHandle n;
  request_sub_ = n.subscribe("/victim_start", 100, &VICTIM_REQUEST_MANAGER::requestCallback, this);
  // response_pub_ = n.advertise<std_msgs::String>("/victim_end", 100);
}

void VICTIM_REQUEST_MANAGER::requestCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string request = msg->data;
  bool received_start = false;

  if (!received_start)
  {
    received_start = true;
    if (request == "start")
    {
      std::string operate_command = "rosrun rescue_vision_24 master &";
      ROS_INFO("Received: victim %s -> operate", request.c_str());
      system(operate_command.c_str());
    }
    else if (request == "end")
    {
      std::string master_node_kill = "rosnode kill /master &";
      ROS_INFO("Received: victim %s -> done", request.c_str());
      system(master_node_kill.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "victim_request_manager");
  VICTIM_REQUEST_MANAGER manager;
  ros::spin();

  return 0;
}
