#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"

ros::ServiceClient* resetEncodersClient;

void resetEncoders() {
    ROS_INFO("Resetting the encoders");
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        //leftTicks = 0;
        //rightTicks = 0;
    }
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);

  //leftTicks = msg->encoderLeft;
  //rightTicks = msg->encoderRight;
}

// Function to drive the robot forward
void driveForward(ros::ServiceClient& diffDrive, double speed, double duration_sec) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = speed;
    srv.request.right = speed;

    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send drive forward command!");
    }

    ros::Duration(duration_sec).sleep(); // Move forward for the specified time

    // Stop after moving
    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send stop command!");
    }
}

// Function to turn the robot 90 degrees to the left
void turnLeft(ros::ServiceClient& diffDrive, double speed, double duration_sec) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = -speed;
    srv.request.right = speed * 1.1;

    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send turn command!");
    }

    ros::Duration(duration_sec).sleep(); // Rotate for the specified time

    // Stop after turning
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

    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);

    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    ros::ServiceClient resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    double forward_speed = 5.0; // [rad/s] wheel speed for moving forward
    double turn_speed = 6.0;    // [rad/s] wheel speed for turning

    double drive_duration = 6.773; // [seconds] Time to drive 1 meter (to be tuned!)
    double turn_duration = 2.06;  // [seconds] Time to rotate 90° (to be tuned!)
    create_fundamentals::ResetEncoders srv;

    // Do the square: 4 times move + turn
    for (int i = 0; i < 20; ++i) {
    //resetEncoders();
        /*resetEncodersClient->call(srv);
        driveForward(diffDrive, forward_speed, drive_duration);

        ros::Duration(1.0).sleep(); // Small pause for stability
        ros::spinOnce();*/

        resetEncodersClient->call(srv);
        ros::Duration(1.0).sleep(); // Small pause for stability

        turnLeft(diffDrive, turn_speed, turn_duration);
        ros::spinOnce();
        ros::Duration(1.0).sleep(); // Small pause after turn
    }

    ROS_INFO("Finished driving the square!");

    return 0;
}
