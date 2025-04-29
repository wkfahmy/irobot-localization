#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"


// --- robot-specific constants ------------------------------------------------
constexpr double WHEEL_RADIUS_M   = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M    = 0.266;    // 26.3 cm

// -----------------------------------------------------------------------------
// Spin the robot through `angle_deg` (CCW positive) at the given wheel speed
// (magnitude |ω_wheel| in rad/s).  Geometry gives the duration automatically.
void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_deg,
                 double wheel_speed_rad_s)
{
    // 1.  Work out how long we must turn
    const double angle_rad   = angle_deg * M_PI / 180.0;
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M; // rad/s
    const double duration_s  = std::fabs(angle_rad) / std::fabs(omega_robot);

    // 2.  Build DiffDrive request (sign decides direction)
    create_fundamentals::DiffDrive srv;
    srv.request.right =  (angle_deg >= 0.0 ?  wheel_speed_rad_s : -wheel_speed_rad_s);
    srv.request.left  = -(srv.request.right);   // opposite direction for pure spin

    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send spin command!");

    ros::Duration(duration_s).sleep();          // 3.  Wait while wheels spin

    // 4.  Stop
    srv.request.left  = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send stop command!");
}


void driveStraight(ros::ServiceClient& diffDrive, double distance, double speed) {
    create_fundamentals::DiffDrive srv;
    double side = abs(distance) / distance; // 1 for forward, -1 for backward
    ROS_INFO("Drive straight : %f", side);
    srv.request.left = side * speed;
    srv.request.right = side * speed;

    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send drive forward command!");
    }

    ros::Duration(6.773 / 5 * speed * abs(distance)).sleep(); // Move forward for the specified time

    // Stop after moving
    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) {
        ROS_ERROR("Failed to send stop command!");
    }
}

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    double angle;
    double distance;
} AngularPoint;

std::vector<Point> obstaclePoints;
std::vector<AngularPoint> angleBoundaryDistances;

const double fov_deg = 240.0;
const double R     = 0.17;  // 34 cm robot radius
const double dL    = 0.12;  // 12 cm lidar offset

const double lidar_min_distance = 0.05; // 10 cm
const double lidar_max_distance = 0.95; // 3.5 m

int rays = 0;
double step = -1.0;
double start_angle = -1.0;

int min_distance_index = -1;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    rays = msg->ranges.size();

    // If the laser has not sent any data yet, initialize the vectors
    if (obstaclePoints.size() != rays) {
        obstaclePoints.resize(rays);
        angleBoundaryDistances.resize(rays);
        step = (fov_deg / rays) * M_PI / 180.0;
        start_angle = (-fov_deg / 2.0) * M_PI / 180.0;
    }

    float min_distance = std::numeric_limits<double>::infinity();
    //min_distance_index = -1;

    for (int i = 0; i < rays; ++i) {
        double theta = start_angle + i * step;
        double sin_t = sin(theta);
        double cos_t = cos(theta);
        double distance = msg->ranges[i];
        if (distance < lidar_min_distance || distance > lidar_max_distance) {
            obstaclePoints[i].x = std::numeric_limits<double>::infinity();
            obstaclePoints[i].y = std::numeric_limits<double>::infinity();

            angleBoundaryDistances[i].angle = 0;
            angleBoundaryDistances[i].distance = std::numeric_limits<double>::infinity();
        } else {
            obstaclePoints[i].x = dL + distance * cos_t;
            obstaclePoints[i].y = distance * sin_t;

            angleBoundaryDistances[i].angle = atan2(obstaclePoints[i].x, obstaclePoints[i].y);
            angleBoundaryDistances[i].distance = std::sqrt(pow(obstaclePoints[i].x,2) + pow(obstaclePoints[i].y,2)) - R;
        }

        if (angleBoundaryDistances[i].distance < min_distance) {
            min_distance = angleBoundaryDistances[i].distance;
            min_distance_index = i;
        }
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

    double wheel_speed_rad_s = 3.0;    // wheel angular speed magnitude

    while (ros::ok()) {
        ros::spinOnce();
        spinInPlace(diffDrive, 90.0, wheel_speed_rad_s);
        ros::Duration(2.0).sleep();
        if(min_distance_index > 0)  {
            ROS_INFO("Aligning with min distance: %f degrees", angleBoundaryDistances[min_distance_index].angle * 180.0 / M_PI);
            spinInPlace(diffDrive, - angleBoundaryDistances[min_distance_index].angle * 180.0 / M_PI , wheel_speed_rad_s);

            ROS_INFO("Driving straight for alignement, moving %f meters", angleBoundaryDistances[min_distance_index].distance - 0.4);
            driveStraight(diffDrive, angleBoundaryDistances[min_distance_index].distance - 0.4, wheel_speed_rad_s);
            spinInPlace(diffDrive, 90.0, wheel_speed_rad_s);

            min_distance_index = -1;
        } else {
            ROS_INFO("No wall found, trying rotating 120");
            spinInPlace(diffDrive, 120.0, wheel_speed_rad_s);
        }
    }

    return 0;
}
