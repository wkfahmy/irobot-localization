#include <ros/ros.h>
#include <create_fundamentals/DiffDrive.h>
#include <plastic_fundamentals/ExecutePlan.h>
#include <cmath>
#include <algorithm>

// Constants
const double WHEEL_RADIUS = 0.0325;
const double TRACK_WIDTH = 0.263;
const double CELL_SIZE = 0.8; // meters

// Direction enum
enum Direction {
    RIGHT = 0, // rotates to the left and then forward
    UP = 1, // moves forward
    LEFT = 2, // rotates to the right and then forward
    DOWN = 3
};

ros::ServiceClient* diff_drive_client;

void driveArc(ros::ServiceClient& diffDrive, double radius_m, double angle_rad, double outer_wheel_speed_rad_s) {
    double v_outer = WHEEL_RADIUS * outer_wheel_speed_rad_s;
    double v_inner = v_outer * (radius_m - TRACK_WIDTH / 2) / (radius_m + TRACK_WIDTH / 2);
    double w_inner = v_inner / WHEEL_RADIUS;

    create_fundamentals::DiffDrive srv;
    if (angle_rad > 0) {
        srv.request.left = w_inner;
        srv.request.right = outer_wheel_speed_rad_s;
    } else {
        srv.request.left = outer_wheel_speed_rad_s;
        srv.request.right = w_inner;
    }

    double arc_length = std::abs(angle_rad * radius_m);
    double v_avg = (v_inner + v_outer) / 2;
    double duration_s = arc_length / v_avg;

    if (!diffDrive.call(srv)) ROS_ERROR("Failed arc drive");
    ros::Duration(duration_s).sleep();

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) ROS_ERROR("Stop after arc failed");
}

void moveLinearSmooth(ros::ServiceClient& diffDrive, double distance_m, double max_wheel_speed) {
    double accel_time = 0.5; // with 0.5 should deliver ~0.8 m in under 5 s
    double linear_speed = WHEEL_RADIUS * max_wheel_speed;
    double cruise_time = std::max(0.0, (distance_m - 2 * accel_time * linear_speed) / linear_speed);

    create_fundamentals::DiffDrive srv;

    for (double t = 0.0; t < accel_time; t += 0.05) {
        // 0.05s corresponds to 20Hz
        double ramped_speed = max_wheel_speed * (t / accel_time);
        srv.request.left = ramped_speed;
        srv.request.right = ramped_speed;
        if (!diffDrive.call(srv)) ROS_ERROR("Ramping up failed");
        ros::Duration(0.05).sleep();
    }

    srv.request.left = max_wheel_speed;
    srv.request.right = max_wheel_speed;
    if (!diffDrive.call(srv)) ROS_ERROR("Cruise failed");
    ros::Duration(cruise_time).sleep();

    for (double t = accel_time; t >= 0.0; t -= 0.05) {
        double ramped_speed = max_wheel_speed * (t / accel_time);
        srv.request.left = ramped_speed;
        srv.request.right = ramped_speed;
        if (!diffDrive.call(srv)) ROS_ERROR("Ramping down failed");
        ros::Duration(0.05).sleep();
    }

    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv)) ROS_ERROR("Stop failed");
}

bool executePlan(plastic_fundamentals::ExecutePlan::Request &req,
    plastic_fundamentals::ExecutePlan::Response &res)
{
    ROS_INFO("Executing plan with %zu steps", req.plan.size());

    int current_heading = UP;
    size_t i = 0;

    while (i < req.plan.size()) {
        int current_dir = req.plan[i];

        size_t j = i;
        while (j < req.plan.size() && req.plan[j] == current_dir)
            ++j;

        int segment_length = j - i;

        // Check if next direction is different
        if (current_heading != current_dir) {
            int delta = current_dir - current_heading;

            // Normalize delta
            if (delta > 3) delta -= 4;
            if (delta < -3) delta += 4;

            // Move shortened straight segment first
            moveLinearSmooth(*diff_drive_client, 0.486, 5.0); // hardcoded

            // Then do fixed-radius arc
            if (delta == 1 || delta == -3) {
                driveArc(*diff_drive_client, 0.2, -M_PI_2, 5.0); // right turn
            } else if (delta == -1 || delta == 3) {
                driveArc(*diff_drive_client, 0.2, M_PI_2, 5.0);  // left turn
            } else if (delta == 2 || delta == -2) {
                driveArc(*diff_drive_client, 0.2, M_PI, 5.0);    // u-turn
            }

            current_heading = current_dir;

            // Only consume 1 step (turn into new direction)
            segment_length = 1;
            j = i + 1;
        }

        // Full straight segment for remaining same-direction steps
        if (segment_length > 0) {
            moveLinearSmooth(*diff_drive_client, segment_length * 0.8, 5.0);
        }

        i = j;
    }

    res.success = true;
    return true;
}

// how to run:
    // rosrun plastic_fundamentals execute_plan_server
    // rosservice call /execute_plan "[1, 0]"

int main(int argc, char **argv) {
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

    ros::ServiceServer service = n.advertiseService("execute_plan", executePlan);
    ROS_INFO("Service /execute_plan ready.");
    ros::spin();
    return 0;
}
