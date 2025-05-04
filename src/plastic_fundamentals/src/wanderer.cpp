#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"


float min_distance;
int min_index;
int start_index;
int end_index;
ros::ServiceClient diffDriveClient;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    min_distance = std::numeric_limits<float>::infinity();
    min_index = -1;
    // Define desired bounds in radians
    float angle_min_limit = -M_PI / 4; // -45 degrees
    float angle_max_limit =  M_PI / 4; // +45 degrees

    // Compute indices for -45 to +45 degrees
    start_index = std::max(0, static_cast<int>((angle_min_limit - msg->angle_min) / msg->angle_increment));
    end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, static_cast<int>((angle_max_limit - msg->angle_min) / msg->angle_increment));

    for(int i = start_index; i < end_index; i++){
        float value = msg->ranges[i];
        if(!isnan(value) && value >= msg->range_min && value <= msg->range_max){
            // Check if the value is less than the current minimum distance
            if(value < min_distance){
                min_distance = value;
                min_index = i;
            }
        }

    }


}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    if(msg->bumpLeft == 1 || msg->bumpRight == 1) {
        ROS_INFO("Bumper hit");
        // make the robot move backward
        create_fundamentals::DiffDrive srv;
        srv.request.left = -10;
        srv.request.right = -10;
        diffDriveClient.call(srv);
        ros::Duration(0.5).sleep();
        // make the robot turn
        srv.request.left = -5;
        srv.request.right = 5;
        diffDriveClient.call(srv);
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;
    ros::Subscriber subLidar = n.subscribe("scan_filtered", 1, laserCallback);

    diffDriveClient = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::Subscriber subSensor = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::service::waitForService("diff_drive");

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10); // Loop at 10 Hz
    while(ros::ok()) {
        // Check if the minimum distance is less than 0.3
        if(min_distance < 0.3) {
            // Stop the robot 
            int diff1 = min_index - start_index;
            int diff2 = end_index - min_index;
            if(diff1 < diff2){
                // Turn left
                srv.request.left = -10;
                srv.request.right = 10;
            } else {
                // Turn right
                srv.request.left = 10;
                srv.request.right = -10;
            }
        } else {
            // Move forward
            srv.request.left = 10;
            srv.request.right = 10;
        }

        // Call the service
        if(diffDriveClient.call(srv)) {
            ROS_INFO("DiffDrive called successfully");
        } else {
            ROS_ERROR("Failed to call DiffDrive service");
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}