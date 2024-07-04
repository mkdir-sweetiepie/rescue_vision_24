#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

pid_t launch_pid = -1;  // 실행된 프로세스의 PID를 저장할 변수

void start_CallBack(const std_msgs::String::ConstPtr& msg)
{
  std::string str_onoff = msg->data;
  ROS_INFO("Received: victim [%s]", str_onoff.c_str());

  if (str_onoff == "start")
  {
    if (launch_pid == -1) // 프로세스가 실행 중이 아닐 때만 실행
    {
      launch_pid = fork(); // 새로운 프로세스를 생성
      if (launch_pid == 0) // 자식 프로세스
      {
        // 새로운 세션과 프로세스 그룹을 생성
        if (setsid() < 0)
        {
          ROS_ERROR("Failed to create new session");
          _exit(EXIT_FAILURE);
        }

        execl("/bin/sh", "sh", "-c", "roslaunch rescue_vision_24 rescue_vision_24.launch", (char *) 0);
        _exit(EXIT_FAILURE); // execl이 실패할 경우
      }
      else if (launch_pid < 0)
      {
        ROS_ERROR("Process creation failed");
      }
      else
      {
        ROS_INFO("rescue_vision_24 started, PID: %d", launch_pid);
      }
    }
  }
  else if (str_onoff == "end")
  {
    if (launch_pid != -1)
    {
      // 프로세스 그룹을 종료
      if (killpg (launch_pid, SIGTERM) == 0)
      {
        ROS_INFO("rescue_vision_24 stopped, PID: %d", launch_pid);
      }
      else
      {
        ROS_ERROR("Failed to stop process group, PID: %d", launch_pid);
      }
      launch_pid = -1; // PID를 초기화
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "system_test");
  ros::NodeHandle n;
  ros::Subscriber victim_onoff_sub = n.subscribe("/victim_start", 1, start_CallBack);

  ros::spin();

  return 0;
}