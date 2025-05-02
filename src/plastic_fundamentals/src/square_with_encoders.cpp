#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoder.h"
#include <cmath>

float radius = 0.0325; // radius of the wheel in meters
float ticksPerRevolution = 5.0; // ticks per revolution
float error_factor = 26/0.84; // error factor
float ticksPerMeter =  ticksPerRevolution / (2 * M_PI * radius) + error_factor; // ticks per meter
ros::ServiceClient* diffDriveClient;
create_fundamentals::DiffDrive srv;


void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
  if(msg->encoderLeft >= ticksPerMeter || msg->encoderRight >= ticksPerMeter){
    //stop the robot
    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient->call(srv);
    ros::Duration(0.5).sleep();
  }
  else{
    //move the robot forward
    srv.request.left = 10;
    srv.request.right = 10;
    diffDriveClient->call(srv);
  }  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_with_encoders");
  ros::NodeHandle n;
  //should reset the encoders before starting
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  diffDriveClient = &diffDrive; // global pointer for use in callback


  ros::spin();
  return 0;
}
