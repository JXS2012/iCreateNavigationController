#include <ros/ros.h>
#include "TurtleEye.h"
#include "DummyTurtle.h"
#include "Potential.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "VectorComputation.h"
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>

#include "math.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>

#define _USE_MATH_DEFINES

class TurtleDriver
{
 private:
  DummyTurtle *dummy;
  TurtleEye *eye;
  Potential *nav;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber odom_sub;
  ros::Subscriber target_sub;
  //constants
  double total_time;
  double freq;

  bool drive_mode; //turn motor on or not. If false, quadrotor won't fly. For testing purpose

  //variables
  pcl::PointXYZ shiftOrigin, turtleOdom, turtleRPY;
  bool TurtleDriver_finish;

  int counter; //used in main loop for tracking time, current time = counter/freq
  std::string sensor;
  std::vector<pcl::PointXYZ> targets, turtlePosLog, turtleRPYLog, visualBuffer;
  std::vector<double> timeTable,timeLog;
  double sensorDistance;

  //functions
  //initiation
  void initiation();
  void initParameters();
  void readTargets();

  //high level controllers
  void invokeController(); //deciding which controller to use
  void stop_drive();

  void odomCallback(const nav_msgs::Odometry);
  void targetCallback(const geometry_msgs::PointStamped msg);

 public:
  TurtleDriver(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~TurtleDriver();

  //drive quadrotor to achieve goals
  void drive();
  //store flight data(x,y,z,roll,pitch,yaw,control inputs)
  void writeLog(int);
  //return true when finished flying
  bool finish();
};
