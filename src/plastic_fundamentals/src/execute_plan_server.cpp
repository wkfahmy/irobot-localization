#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <plastic_fundamentals/ExecutePlan.h>

constexpr double WHEEL_RADIUS_M   = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M    = 0.263;    // 26.3 cm
ros::ServiceClient* diff_drive_client;

enum Direction {
    RIGHT = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3
};

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M;
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

void moveLinear(ros::ServiceClient& diffDrive,
                double distance_m,
                double wheel_speed_rad_s)
{
    const double linear_speed = WHEEL_RADIUS_M * wheel_speed_rad_s;
    const double duration_s = std::fabs(distance_m) / std::fabs(linear_speed);

    create_fundamentals::DiffDrive srv;
    srv.request.left = (distance_m >= 0.0 ? wheel_speed_rad_s : -wheel_speed_rad_s);
    srv.request.right = srv.request.left;

    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send move command!");

    ros::Duration(duration_s).sleep();

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send stop command!");
    ros::Duration(2).sleep(); // Allow some time for the robot to stop    
}




bool executePlan(plastic_fundamentals::ExecutePlan::Request &req,
                  plastic_fundamentals::ExecutePlan::Response &res)
{
    ROS_INFO("Executing plan with %zu steps", req.plan.size());
    for(int dir : req.plan) {
        switch(dir) {
            case RIGHT:
                spinInPlace(*diff_drive_client, -M_PI_2, 5.0);
                moveLinear(*diff_drive_client, 0.8, 5.0);
                break;
            case UP:
                moveLinear(*diff_drive_client, 0.8, 5.0);
                break;
            case LEFT:
                spinInPlace(*diff_drive_client, M_PI_2, 5.0);
                moveLinear(*diff_drive_client, 0.8, 5.0);
                break;
            case DOWN:
                moveLinear(*diff_drive_client, -0.8, 5.0);
                break;
            default:
                ROS_ERROR("Invalid direction in plan");
                res.success = false;
                return false;
        }
    }
    res.success = true;
    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    // should we call align here directly?
    // or should we run align first and then call this?

    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;
    ros::ServiceServer service = n.advertiseService("execute_plan", executePlan);
    ROS_INFO("Service /execute_plan ready.");
    ros::spin();
    return 0;
}



