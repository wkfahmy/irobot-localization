/**************************************************************
 *  square_with_encoders.cpp
 *  (Robotics Fundamentals – SoSe 2025)
 *
 *  Executes a 1 m × 1 m square using wheel-encoder feedback.
 *  No run-time tuning: all numbers fixed at compile time.
 *
 *  Topics  : /sensor_packet   (create_fundamentals/SensorPacket)
 *  Service : /diff_drive      (create_fundamentals/DiffDrive)
 *************************************************************/

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cmath>

double radius = 0.0325;
double ticksPerRevolution = 6.25;
double ticksPerRevolutionRot = 5.5;
double track_width  = 0.263;

double leftTicks = 0;
double rightTicks = 0;

ros::ServiceClient* diffDriveClient;
create_fundamentals::DiffDrive srv;

ros::ServiceClient* resetEncodersClient;

void resetEncoders() {
    ROS_INFO("Resetting the encoders");
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
}

double getTranslationTicks(double distance) {
    return ticksPerRevolution * distance / (2 * M_PI * radius);
}

double getRotationTicks(double angle_rad) {
    return ticksPerRevolutionRot * angle_rad * track_width / (4 * M_PI * radius);
}

void rotate(double angle_rad, double speed) {
    ROS_INFO("Rotation: %f", angle_rad);
    double side = std::floor(abs(angle_rad) / angle_rad);

    double ticks = getRotationTicks(angle_rad);

    resetEncoders();

    srv.request.left = - side * speed;
    srv.request.right = side * speed;

    double correction = 0.0;
    ros::Rate rate(10);  // 10 Hz control loop
    ros::spinOnce();

    // Continue rotating until the average of the ticks is greater than the target ticks
    while (ticks > (abs(leftTicks) + abs(rightTicks)) / 2) {

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            srv.request.left = - side * speed * (1 + correction / 2);
            srv.request.right = side * speed * (1 - correction / 2);
        }
        diffDriveClient->call(srv);
        ros::spinOnce();
    }

    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient->call(srv);

    resetEncoders();
}

void translate(double distance, double speed) {
    ROS_INFO("Translation: %f", distance);
    double side = std::floor(abs(distance) / distance);

    double ticks = getTranslationTicks(distance);

    resetEncoders();
    ros::spinOnce();

    srv.request.left = side * speed;
    srv.request.right = side * speed;

    double correction = 0.0;
    ros::Rate rate(10);  // 10 Hz control loop
    ros::spinOnce();

    // Continue moving until the average of the ticks is greater than the target ticks
    while (ticks > (abs(leftTicks) + abs(rightTicks)) / 2) {

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            srv.request.left = side * speed * (1 + correction / 2);
            srv.request.right = side * speed * (1 - correction / 2);
        }
        diffDriveClient->call(srv);

        rate.sleep();
        ros::spinOnce();
    }

    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient->call(srv);
    resetEncoders();
}

/* ───────── globals filled by sensor callback ────────────────────────────── */
static double enc_left  = 0.0;      // cumulative wheel radians
static double enc_right = 0.0;      // "
static bool   enc_ok    = false;

void sensorCB(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);

  leftTicks = msg->encoderLeft;
  rightTicks = msg->encoderRight;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_with_encoders");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  diffDriveClient = &diffDrive;
  ros::ServiceClient resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
  resetEncodersClient = &resetEncoders;

  double speed = 3.0;

  for (int i = 0; i < 20; i++) {
        translate(1.0, speed);
        ros::Duration(1.0).sleep();
        rotate(M_PI / 2, speed);
        ros::Duration(1.0).sleep();
  }

  return 0;
}
