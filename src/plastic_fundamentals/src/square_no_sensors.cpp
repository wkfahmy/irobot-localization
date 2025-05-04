#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

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

void turnLeft(ros::ServiceClient& diffDrive, double speed, double duration_sec) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = -speed;
    srv.request.right = speed * 1.1; // Correcting for the difference in the real wheel speed

    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send turn command!");
    }

    ros::Duration(duration_sec).sleep(); // Rotate for the specified time

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send stop command!");
    }
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
    for (int i = 0; i < 20; ++i) {
        driveForward(diffDrive, forward_speed, drive_duration);
        ros::Duration(1.0).sleep();

        turnLeft(diffDrive, turn_speed, turn_duration);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Finished driving the square!");
    return 0;
}
