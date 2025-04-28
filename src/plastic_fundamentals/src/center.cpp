#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

typedef struct {
    double x;
    double y;
} Point;

std::vector<Point> boundaryPoints;
std::vector<Point> obstaclePoints;
std::vector<double> boundaryDistances;

const float fov_deg = 240.0f;
const float R     = 0.34f;  // 34 cm robot radius
const float dL    = 0.12f;  // 12 cm lidar offset


int min_distance_index;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int ranges_size = msg->ranges.size();
    if (boundaryDistances.size() != ranges_size) {
        boundaryPoints.resize(ranges_size);
        obstaclePoints.resize(ranges_size);
        boundaryDistances.resize(ranges_size);
        float step = (fov_deg / ranges_size) * M_PI / 180.0f;
        float start_angle = (-fov_deg / 2.0f) * M_PI / 180.0f;
        for (int i = 0; i < ranges_size; ++i) {
            float theta = start_angle + i * step;
            float sin_t = sin(theta);
            float cos_t = cos(theta);
            float disc = R*R - dL ^ 2 * sin_t ^ 2; // We know that disc > 0 because the LIDAR is in the robot's boundaries
            if (disc < 0) disc = 0;  // just in case of numerical round-off

            double t_exit = -dL * cos_t + sqrt(disc);

            boundaryPoints[i].x = dL + t_exit * c;
            boundaryPoints[i].y =        t_exit * s;
        }
    }

    for (int i = 0; i < ranges_size; ++i) {
        float theta = start_angle + i * step;
        float sin_t = sin(theta);
        float cos_t = cos(theta);
        float distance = msg->ranges[i] - boundaryDistances[i];
        double r = msg->ranges[i];
        obstaclePoints[i].x = dL + distance * cos_t;
        obstaclePoints[i].y = r * sin_t;
    }

    for (size_t j = 0; j < ranges_size; ++j) {
        double min_d = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < j + ; ++k) {
            double dx = boundaryPoints[j].x - obstaclePoints[k].x;
            double dy = boundaryPoints[j].y - obstaclePoints[k].y;
            double d2 = dx*dx + dy*dy;
            if (d2 < min_d) {
                min_d = d2;
            }
        }
        boundaryDistances[j] = std::sqrt(min_d);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "center");
    ros::NodeHandle n;

    ros::Subscriber subLidar = n.subscribe("scan_filtered", 1, laserCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10);

    while (ros::ok()) {

    }

    return 0;
}
