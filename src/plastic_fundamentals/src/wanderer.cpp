#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    float32 min_distance = std::numeric_limits<float32>::infinity();
    for(int i = 0; i < msg->ranges.size(); i++){
        if(msg->ranges[i] < min_distance){
}
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;
    ros::Subscriber subLidar = n.subscribe("scan_filtered", 1, laserCallback);

    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10); // Loop at 10 Hz

    return 0;
}
