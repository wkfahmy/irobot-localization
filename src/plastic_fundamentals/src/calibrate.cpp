#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <plastic_fundamentals/Point.h>
#include <plastic_fundamentals/Move.h>

#include <cmath>
#include <vector>
#include <cstdlib>
#include <optional>

ros::ServiceClient marker;

// CALIBRATION PARAMETERS
double k_rotation = 1.05; // Correction factor for rotation
double k_translation = 1.0; // Correction factor for translation


double WHEEL_RADIUS_M = 0.0325;
double TRACK_WIDTH_M = 0.263;

double RANSAC_DIST_THRESHOLD = 0.01;
int    RANSAC_MIN_INLIERS    = 40;
int    RANSAC_MAX_ITER       = 1000;

double leftTicks = 0;
double rightTicks = 0;

bool moving = false;

float min_distance;
int min_index;
int start_index;
int end_index;

ros::ServiceClient* rotateClient;
ros::ServiceClient* translateClient;


bool rotate(double angle_rad, double speed) {
    plastic_fundamentals::Move srv;
    srv.request.angle = angle_rad;
    srv.request.speed = speed;
    bool success = rotateClient->call(srv);
    return success;
}

bool translate(double distance, double speed) {
    plastic_fundamentals::Move srv;
    srv.request.distance = distance;
    srv.request.speed = speed;
    bool success = translateClient->call(srv);
    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::ServiceClient rotate_client = nh.serviceClient<plastic_fundamentals::Move>("perform_rotation");
    rotateClient = &rotate_client;
    ros::ServiceClient translate_client = nh.serviceClient<plastic_fundamentals::Move>("perform_translation");
    translateClient = &translate_client;


    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(20);
    ros::spinOnce();

    int turn_count = 4;
    translate(1.0, 10.0);
    //double turn_angle = M_PI / 2;
    rate.sleep();
    ros::spinOnce();
    //while (ros::ok() && turn_count > 0) {
     //   translate(0.25, 10.0);
    //    turn_count--;


    //}
    return 0;
}
