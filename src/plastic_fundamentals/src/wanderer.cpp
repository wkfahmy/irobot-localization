#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

float lidar_distance = -1.f; // Minimum distance in front sector
bool scan_received = false;  // Flag to check if we got a scan
int min_distance_index;
int start_index = -1;        // Now global
int end_index = -1;
bool isSpinning = false;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int ranges_size = msg->ranges.size(); // How many distance measurements we have in this scan. (usually ~682 for Hokuyo URG-04LX-UG01)
    float angle_min = msg->angle_min; // The angle corresponding to ranges[0] (leftmost beam), in radians (probably around –2.094 rad = –120°)
    float angle_increment = msg->angle_increment; // Angular separation between two consecutive beams, in radians (very small, around 0.36° ≈ 0.0063 rad).

    // The sector that we care about: -30° to +30° (in radians)
    float sector_min_angle = -M_PI / 6; // -30 degrees
    float sector_max_angle =  M_PI / 6; // +30 degrees

    // Computing the indices corresponding to -30° and +30°
    start_index = std::max(0, int((sector_min_angle - angle_min) / angle_increment));
    end_index = std::min(ranges_size - 1, int((sector_max_angle - angle_min) / angle_increment));

    // Finding the minimum distance in that sector
    float min_distance = std::numeric_limits<float>::infinity();

    for (int i = start_index; i <= end_index; ++i) {
        float d = msg->ranges[i];
        if (!std::isnan(d) && d > msg->range_min && d < msg->range_max) // here it can also handles the NaN readings
        {
            if (d < min_distance) {
                min_distance = d;
                min_distance_index = i;
            }
        }
    }

    if (min_distance == std::numeric_limits<float>::infinity()) {
        lidar_distance = -1.f; // No valid readings
    } else {
        lidar_distance = min_distance;
    }

    scan_received = true; // We have a valid scan
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    // (Optional) Print encoder values if needed
    // ROS_INFO("Left encoder: %f, Right encoder: %f", msg->encoderLeft, msg->encoderRight);

    // (Optional) Print the minimum measured distance in the defined sector
    ROS_INFO("Min distance in sector: %f meters", lidar_distance);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;

    ros::Subscriber subDrive = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::Subscriber subLidar = n.subscribe("scan_filtered", 1, laserCallback);

    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10); // Loop at 10 Hz

    while (ros::ok()) {
        ros::spinOnce();

        if (!scan_received) {
            rate.sleep();
            continue;
        }
        if (isSpinning && lidar_distance > 0.3f) {
            isSpinning = false;
        }

        if ((lidar_distance > 0.3f || lidar_distance < 0.f) && !isSpinning) {
            // No obstacle close → drive forward
            srv.request.left = 5.0;
            srv.request.right = 5.0;
        } else {
            int index_diff_left = min_distance_index - start_index;
            int index_diff_right = end_index - min_distance_index;

            if (index_diff_left <= index_diff_right) {
                srv.request.left = -3.0;
                srv.request.right = 3.0;
            } else {
                srv.request.left = 3.0;
                srv.request.right = -3.0;
            }
            isSpinning = true;
        }



        if (!diffDrive.call(srv)) {
            ROS_ERROR("Failed to call diff_drive service!");
        }

        rate.sleep();
    }

    return 0;
}
