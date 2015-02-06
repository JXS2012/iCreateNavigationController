#include <ros/ros.h>

#include "geometry_msgs/Twist.h"

#include <pcl/point_cloud.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>


class DummyTurtle
{
 private:
  //direct control related
  //ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //control cmds publisher
  ros::Publisher pub_vel;

  //control msgs
  geometry_msgs::Twist msg_turtleInput;

  //variables
  std::vector<float> linearVel_log,omega_log;

  //constants
  std::string strThrustCmd,strActThrust,quadrotorType,sensor;
  std::vector<double> thrustCmd,actualThrust;
  double sensorDistance;

  //functions
  void initParameters();
  //take in desired acc and current psi angle, calculate control output to quadrotor
  void vel_des2control(double,double,double);
  //double interpolateThrust(double);
  std::vector<double> readTable(std::string);

 public:

  DummyTurtle(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~DummyTurtle();

  void on();
  void off();
  
  void VelDrive(double,double,double);

  inline float getLogVel(int i)
  {
    return linearVel_log[i];
  }

  inline float getLogOmega(int i)
  {
    return omega_log[i];
  }

};
