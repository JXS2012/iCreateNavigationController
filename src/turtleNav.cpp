#include <ros/ros.h>
#include "TurtleDriver.h"

TurtleDriver *turtle;
int fileNo;

void timerCallback(const ros::TimerEvent&)
{
  turtle->drive();
}

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"turtle");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~"); //To obtain private parameters

  if (!nh_private.getParam("freq",freq))
    freq = 30.0;
  ROS_INFO("call back freq %.3f",freq);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  ROS_INFO("create timer");

  ROS_INFO("Start from test No?");
  std::cin >> fileNo;
  ROS_INFO("Start from test No.%d.", fileNo);

  while (nh.ok() )
    {
      char flag;
      turtle = new TurtleDriver(nh,nh_private);
      ROS_INFO("create turtle");
      
      while (!turtle->finish())
	ros::spinOnce();
      
      ROS_INFO("Writing to No.%d", fileNo);
      turtle->writeLog(fileNo);
      delete turtle;

      ROS_INFO("Continue? (y/n)");
      std::cin >> flag;
      if (flag == 'n')
	break;

      fileNo ++;
    }


  return 0;

}
