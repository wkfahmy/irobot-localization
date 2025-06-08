#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <plastic_fundamentals/Point.h>

#include <cmath>
#include <vector>
#include <cstdlib>
#include <optional>

ros::ServiceClient* resetEncodersClient;
create_fundamentals::DiffDrive diffDriveSrv;
ros::ServiceClient* diffDriveClient;

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

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;
}

double getRotationTicks(double angle_rad) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double wheel_travel_distance = (TRACK_WIDTH_M * angle_rad) / 2;
    return k_rotation * (wheel_travel_distance / wheel_circumference) * ticks_per_revolution;
}

double getTranslationTicks(double distance) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    return k_translation * (distance / wheel_circumference) * ticks_per_revolution;
}


void rotate(double angle_rad, double speed) {
    moving = true;

    double ticks = getRotationTicks(fabs(angle_rad));


    resetEncoders();
    ros::spinOnce();

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    diffDriveSrv.request.left = -side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;


    while (ticks * 0.98 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 4)) {
            error_based_speed = (speed / 4);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = -side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();

    moving = false;
}


void translate(double distance, double speed) {
    moving = true;

    double ticks = getTranslationTicks(distance);

    resetEncoders();
    ros::spinOnce();

    double side = (distance > 0) ? 1.0 : -1.0;
    diffDriveSrv.request.left = side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while (ticks * 0.99 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 2)) {
            error_based_speed = (speed / 2);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();

    moving = false;
}

void driveArc(double angle_rad, double arc_radius, double speed) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double b = TRACK_WIDTH_M;

    double abs_angle = std::abs(angle_rad);

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    double d_left  = (arc_radius - side * b / 2.0) * abs_angle;
    double d_right = (arc_radius + side * b / 2.0) * abs_angle;

    double ticks_left  = (d_left  / wheel_circumference) * ticks_per_revolution;
    double ticks_right = (d_right / wheel_circumference) * ticks_per_revolution;

    double v_left, v_right;
    if (angle_rad > 0) {
        v_left  = speed * (d_left / d_right);
        v_right = speed;
    } else if (angle_rad < 0) {
        v_left  = speed;
        v_right = speed * (d_right / d_left);
    } else {
        ROS_WARN("driveArc called with angle_rad = 0, nothing to do!");
        return;
    }


    resetEncoders();
    ros::spinOnce();

    diffDriveSrv.request.left  = v_left;
    diffDriveSrv.request.right = v_right;
    diffDriveClient->call(diffDriveSrv);

    ros::Rate rate(100);

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while ((fabs(leftTicks) < fabs(ticks_left)) || (fabs(rightTicks) < fabs(ticks_right))) {

        diffDriveSrv.request.left  = v_left;
        diffDriveSrv.request.right = v_right;
        diffDriveClient->call(diffDriveSrv);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle nh;

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::Subscriber sub = nh.subscribe("/sensor_packet", 1, sensorCallback);

    ros::ServiceClient diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diffDriveClient = &diffDrive;

    ros::ServiceClient resetEncoders = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    ros::Rate rate(100);
    int turn_count = 4;
    double turn_angle = M_PI / 2;
    while (ros::ok() && turn_count > 0) {
        rotate(turn_angle, 7.0);
        turn_count--;

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
