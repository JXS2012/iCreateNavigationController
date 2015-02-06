#include "TurtleEye.h"

TurtleEye::TurtleEye(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"birdeye");
  listener = new(tf::TransformListener);
  transform = new(tf::StampedTransform);

  //target_sub = node.subscribe("/target_states",1,&TurtleEye::targetCallback,this);  
  target_sub = node.subscribe("/VisualBird/targetPoint",1,&TurtleEye::targetCallback,this);  
  //initiate variables
  initiation();


}

/*
void TurtleEye::targetCallback(const std_msgs::Float32MultiArray msg)
{

//	targetPos.x = msg.data[0];
//	targetPos.y = msg.data[1];
//	targetPos.z = msg.data[2];
//	targetVel.x = msg.data[3];
//	targetVel.y = msg.data[4];
//	targetVel.z = msg.data[5];

  targetPos = zeroVector;
  targetVel = zeroVector;
}
*/

void TurtleEye::targetCallback(const geometry_msgs::PointStamped msg)
{
  targetPos.x = msg.point.x - shiftedOrigin.x/1000.;
  targetPos.y = msg.point.y - shiftedOrigin.y/1000.;
  targetPos.z = 0;
  targetVel = zeroVector;
}


TurtleEye::~TurtleEye()
{
  delete listener;
  delete transform;
  ROS_INFO("Destroying birdeye "); 
}

void TurtleEye::initiation()
{
  initPosition();
  initParameters();
  flightOrigin();

  //defining pre points for later velocity calculation
  prePoint = originPoint;
  ROS_INFO("origin at %.3f %.3f %.3f",originPoint.x,originPoint.y,originPoint.z);

}

void TurtleEye::initParameters()
{
  if (!nh_private_.getParam("turtle_freq",freq))
    freq = 30;
  if (!nh_private_.getParam("turtle_vicon",vicon))
    vicon = "/vicon/turtle/turtle";
  if (!nh_private_.getParam("turtle_averageStep",averageStep))
    averageStep = 10;
  double tempx, tempy;
  if (!nh_private_.getParam("originX", tempx))
    tempx = 1169;
  if (!nh_private_.getParam("originY", tempy))
    tempy = 954;
 
  shiftedOrigin = zeroVector;
  shiftedOrigin.x = tempx;
  shiftedOrigin.y = tempy;


  ROS_INFO("Frequency initialized %.3f",freq);
  ROS_INFO("vicon %s",vicon.c_str());
  ROS_INFO("Origin at %.3f %.3f", shiftedOrigin.x, shiftedOrigin.y);
  lostVicon = false;
  freezeCounter = 0;
  currentIndex = 0;

  zeroVector.x = 0;
  zeroVector.y = 0;
  zeroVector.z = 0;
}

void TurtleEye::initPosition()
{
  currentPoint = zeroVector;
  prePoint = zeroVector;
  originPoint = zeroVector;
  currentVel = zeroVector;
  intError = zeroVector;
  targetPos = zeroVector;
  targetVel = zeroVector;

  psi = 0;
  phi = 0;
  theta = 0;
}

void TurtleEye::flightOrigin()
{
  //finding start point of flight
  pcl::PointXYZ originAngle;
  while (true)
    {
      try{
	listener->lookupTransform("/world", vicon,  
				 ros::Time(0), *transform);
      }
      catch (tf::TransformException ex){
	ROS_DEBUG("%s",ex.what());
	usleep(100);
	continue;
      }
      originPoint.x = transform->getOrigin().x()*1000;
      originPoint.y = transform->getOrigin().y()*1000;
      originPoint.z = transform->getOrigin().z()*1000;

      double yaw,pitch,roll;
      tf::Matrix3x3(transform->getRotation()).getEulerYPR(yaw,pitch,roll);
      transformZYXtoZXY(yaw,pitch,roll);
      
      originAngle.x = phi;
      originAngle.y = theta;
      originAngle.z = psi;
      break;
    }
  pointLog.push_back(originPoint);
  angleLog.push_back(originAngle);
  velLog.push_back(zeroVector);
  accLog.push_back(zeroVector);
}

void TurtleEye::transformZYXtoZXY(double yaw, double pitch, double roll)
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

void TurtleEye::updateFlightStatus()
{
  double roll,pitch,yaw;

  try{
    listener->lookupTransform("/world", vicon,  
			     ros::Time(0), *transform);
  }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s",ex.what());
  }
  
  //get angles in ZYX
  tf::Matrix3x3(transform->getRotation()).getEulerYPR(yaw,pitch,roll);
  
  //transform from euler zyx to euler zxy
  transformZYXtoZXY(yaw,pitch,roll);
  
  //getting positions x,y,z
  currentPoint.x = transform->getOrigin().x()*1000;
  currentPoint.y = transform->getOrigin().y()*1000;
  currentPoint.z = transform->getOrigin().z()*1000;

  flightLog();
  
  //calculate velocity using one step differentiate, can be improved by moving average method.

  viconLostHandle();
  currentVel = differentiate(pointLog);
  velLog.push_back(currentVel);
  currentAcc = differentiate(velLog);
  accLog.push_back(currentAcc);

  //log target position
  ROS_DEBUG("eye target %.3f %.3f %.3f", targetVel.x, targetVel.y, targetVel.z);
  targetLog.push_back(targetPos);
  
}

pcl::PointXYZ TurtleEye::differentiate(const std::vector<pcl::PointXYZ>& x)
{
  pcl::PointXYZ dx;
  if (lostVicon)
    {
      dx = zeroVector;
    }
  else
    {
      if (freezeCounter!=0)
	{
	  if (freezeCounter>averageStep) 
	    dx = scalarProduct(freq/(freezeCounter+1), vectorMinus(x.back(),x[x.size()-1-freezeCounter-1]));
	  else
	    dx = scalarProduct(freq/averageStep, vectorMinus(x.back(),x[x.size()-1-averageStep]));
	  freezeCounter = 0;
	}
      else
	{
	  if (x.size()>(unsigned)(1+averageStep))
	    {
	      if (!checkZeroVector(x[x.size()-1-averageStep]))
		dx = scalarProduct(freq/averageStep, vectorMinus(x.back(),x[x.size()-1-averageStep]));
	      else
		{
		  int tempCounter = 1;
		  while (checkZeroVector(x[x.size()-1-averageStep-tempCounter]))
		    tempCounter++;
		  dx = scalarProduct(freq/(averageStep+tempCounter), vectorMinus(x.back(),x[x.size()-1-averageStep-tempCounter]));
		}
	    }
	  else
	    {
	      dx = scalarProduct(freq/x.size(),vectorMinus(x.back(),x.front()));
	    }
	}
    }
  return dx;
}

void TurtleEye::viconLostHandle()
{
  if (pointLog.size()<(unsigned)averageStep)
    {
      lostVicon = false;
      return;
    }
  if (checkZeroVector(vectorMinus(pointLog.back(),pointLog[pointLog.size()-2-freezeCounter])))
    {
      lostVicon = true;
      freezeCounter++;
      //pointLog.pop_back();
      //pointLog.push_back(zeroVector);
    }
  else
    {
      lostVicon = false;
      freezeCounter = 0;
    }
}

bool TurtleEye::checkZeroVector(pcl::PointXYZ x)
{
  if (x.x == 0 && x.y == 0 && x.z==0)
    return true;
  else
    return false;
}

void TurtleEye::updateIntError(pcl::PointXYZ targetPoint)
{
  intError = vectorPlus(intError, vectorMinus(currentPoint,targetPoint) );
}

void TurtleEye::resetIntError()
{
  intError = zeroVector;
}

void TurtleEye::flightLog()
{
  currentIndex++;
  pointLog.push_back(currentPoint);

  pcl::PointXYZ tempAngle;
  tempAngle.x = phi;
  tempAngle.y = theta;
  tempAngle.z = psi;
  angleLog.push_back(tempAngle);
}




