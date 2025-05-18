#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

constexpr double WHEEL_RADIUS_M   = 0.0325;
constexpr double TRACK_WIDTH_M    = 0.276;


void driveForward(ros::ServiceClient& diffDrive, double speed, double duration_sec) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = speed;
    srv.request.right = speed;

    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send drive forward command!");
    }

    ros::Duration(duration_sec).sleep(); // Move forward for the specified time

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send stop command!");
    }
}

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M; // rad/s
    const double duration_s = std::fabs(angle_rad) / std::fabs(omega_robot);

    create_fundamentals::DiffDrive srv;
    srv.request.right = (angle_rad >= 0.0 ? wheel_speed_rad_s : -wheel_speed_rad_s);
    srv.request.left = -(srv.request.right);

    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send spin command!");

    ros::Duration(duration_s).sleep();

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send stop command!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "square_no_sensors");
    ros::NodeHandle n;

    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    double forward_speed = 5.0;
    double turn_speed = 3.0;

    double drive_duration = 6.773;
    double turn_duration = 2.06;

    // Do the square: 4 * 5 = 20 times move + turn
    for (int i = 0; i < 4; ++i) {
        //driveForward(diffDrive, forward_speed, drive_duration);
        //ros::Duration(1.0).sleep();

        spinInPlace(diffDrive, M_PI / 2, forward_speed);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Finished driving the square!");
    return 0;
}
