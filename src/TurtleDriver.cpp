#include "TurtleDriver.h"

TurtleDriver::TurtleDriver(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"TurtleDriver");

  dummy = new DummyTurtle(nh_,nh_private_);
  ROS_INFO("Create Dummy");
  eye = new TurtleEye(nh_,nh_private_);
  ROS_INFO("Create Eye");
  nav = new Potential(nh_,nh_private_);
  ROS_INFO("Create Navigation Function");


  odom_sub = node.subscribe("/odom",1,&TurtleDriver::odomCallback, this);
  target_sub = node.subscribe("/VisualBird/targetPoint",1,&TurtleDriver::targetCallback, this);
  initiation();
}

TurtleDriver::~TurtleDriver()
{

  delete dummy;
  delete eye;
  delete nav;
  ROS_INFO("Destroying TurtleDriver "); 

}

void TurtleDriver::odomCallback(const nav_msgs::Odometry msg)
{
  turtleOdom.x = msg.pose.pose.position.x;
  turtleOdom.y = msg.pose.pose.position.y;
  turtleOdom.z = msg.pose.pose.position.z;

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(msg.pose.pose.orientation.x,
			       msg.pose.pose.orientation.y,
			       msg.pose.pose.orientation.z,
			       msg.pose.pose.orientation.w)).getEulerYPR(yaw,pitch,roll);
  turtleRPY.z = yaw;
  turtleRPY.y = pitch;
  turtleRPY.x = roll;
}

void TurtleDriver::targetCallback(const geometry_msgs::PointStamped msg)
{
}

void TurtleDriver::initiation()
{
  initParameters();

  //shiftOrigin.x = 1515.7;
  //shiftOrigin.y = 1773.6;
  shiftOrigin.z = 0;

  if (drive_mode)
  {
    double secs =ros::Time::now().toSec();
    timeTable.push_back(secs);
  }
}

void TurtleDriver::initParameters()
{
  double tempx,tempy;
  if (!nh_private_.getParam("originX", tempx))
    tempx = 1169; 

  if (!nh_private_.getParam("originY", tempy))
    tempy = 954; 

  shiftOrigin.x = tempx;
  shiftOrigin.y = tempy;

  if (!nh_private_.getParam("turtle_drive", drive_mode))
    drive_mode = false; //turn motor on or not. 

  if (!nh_private_.getParam("turtle_freq",freq))
    freq = 30;

  if (!nh_private_.getParam("turtle_total_time",total_time))
    total_time = 30;

  if (!nh_private_.getParam("turtle_sensor", sensor))
    sensor = "vicon"; //unit mm

  if (!nh_private_.getParam("turtle_sensor_distance", sensorDistance))
    sensorDistance = 0.1; //unit m

  ROS_INFO("Drive mode initialized %d",drive_mode);
  ROS_INFO("Drive duration %.3f",total_time);

  counter = 0; //used in main loop for tracking time

  TurtleDriver_finish = false;
}

void TurtleDriver::invokeController()
{
  //in case vicon lost track of model
  if (sensor == "vicon")
    {
      if (eye->getCurrentVel().x == 0 && eye->getCurrentVel().y == 0 && eye->getCurrentVel().z == 0)
	{
	  dummy->VelDrive(0,0,eye->getPsi());
	}
      else
	{
	  pcl::PointXYZ vel_des = nav->getVelDes();
	  ROS_INFO("desired vel: %.3f %.3f current yaw %.3f",vel_des.x,vel_des.y,eye->getPsi());
	  if (sqrt(vel_des.x*vel_des.x+vel_des.y*vel_des.y)>0.05)
	    dummy->VelDrive(vel_des.x,vel_des.y,eye->getPsi());
	  else
	    dummy->VelDrive(0,0,eye->getPsi());
	}
    }
}

void TurtleDriver::drive()
{
  eye->updateFlightStatus();
  pcl::PointXYZ sensorPoint;
  sensorPoint = eye->getCurrentPoint();
  sensorPoint.x += sensorDistance * cos(eye->getPsi()) * 1000;
  sensorPoint.y += sensorDistance * sin(eye->getPsi()) * 1000;
  ROS_INFO("target at: %.3f %.3f",eye->getTargetPos().x, eye->getTargetPos().y);
  nav->updatePVA(scalarProduct(0.001,vectorMinus(sensorPoint,shiftOrigin)),eye->getTargetPos(),scalarProduct(0.001,eye->getCurrentVel()),eye->getTargetVel(),0);

  invokeController();

  double secs =ros::Time::now().toSec();
  timeLog.push_back(secs); 
  
  turtlePosLog.push_back(turtleOdom);
  turtleRPYLog.push_back(turtleRPY);

  stop_drive();
  
  counter ++;
}

void TurtleDriver::stop_drive()
{
  if (counter >= freq * total_time)
    {
      TurtleDriver_finish = true;
      double secs =ros::Time::now().toSec();
      timeTable.push_back(secs);
    }
}

bool TurtleDriver::finish()
{
  return TurtleDriver_finish;
}

void TurtleDriver::writeLog(int fileNo)
{
  std::FILE * pFile;
  char buff[100];
  sprintf(buff,"/home/jianxin/turtle%d.txt",fileNo);
  std::string filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; (unsigned)i<timeLog.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vx %.3f vy %.3f vz %.3f ax %.3f ay %.3f az %.3f roll %.3f pitch %.3f yaw %.3f linear-vel %.3f angular-vel %.3f time %.6f\n", eye->getLogPos(i).x, eye->getLogPos(i).y, eye->getLogPos(i).z, eye->getLogVel(i).x,eye->getLogVel(i).y,eye->getLogVel(i).z, eye->getLogAcc(i).x, eye->getLogAcc(i).y, eye->getLogAcc(i).z, eye->getLogAngle(i).x, eye->getLogAngle(i).y, eye->getLogAngle(i).z, dummy->getLogVel(i), dummy->getLogOmega(i),timeLog[i]);
    }
  fclose(pFile);


  sprintf(buff,"/home/jianxin/turtleOdom%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; (unsigned)i<timeLog.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f roll %.3f pitch %.3f yaw %.3f time %.6f\n", turtlePosLog[i].x, turtlePosLog[i].y, turtlePosLog[i].z, turtleRPYLog[i].x, turtleRPYLog[i].y, turtleRPYLog[i].z, timeLog[i]);
    }
  fclose(pFile);


  sprintf(buff,"/home/jianxin/turtleTimer%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      fprintf(pFile,"Start turtle %.6f\n", timeTable[0]);
      fprintf(pFile,"Stop turtle %.6f\n", timeTable[2]);
    }
  fclose(pFile);
}

