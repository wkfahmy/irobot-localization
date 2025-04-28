#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include <cmath>     // std::fabs, M_PI
#include <cstdlib>   // std::atof

// --- robot-specific constants ------------------------------------------------
constexpr double WHEEL_RADIUS_M   = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M    = 0.263;    // 26.3 cm

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

// -----------------------------------------------------------------------------
// Usage:   rosrun <pkg> spin_test   [angle_deg] [wheel_speed_rad_s]
int main(int argc, char** argv)
{
    ros::init(argc, argv, "spin_test");
    ros::NodeHandle n;

    ros::ServiceClient diffDrive =
        n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    // --- parameters from CLI or defaults -------------------------------------
    double angle_deg         = 90.0;   // CCW
    double wheel_speed_rad_s = 3.0;    // wheel angular speed magnitude

    if (argc > 1) angle_deg         = std::atof(argv[1]);
    if (argc > 2) wheel_speed_rad_s = std::atof(argv[2]);

    ROS_INFO_STREAM("Spinning " << angle_deg << " °  at |ω| = "
                                << wheel_speed_rad_s << " rad/s");

    spinInPlace(diffDrive, angle_deg, wheel_speed_rad_s);

    ROS_INFO("Done.");
    return 0;
}
