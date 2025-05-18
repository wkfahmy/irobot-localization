#include <ros/ros.h>
#include <create_fundamentals/DiffDrive.h>
#include <plastic_fundamentals/ExecutePlan.h>
#include <cmath>
#include <algorithm>

// Constants
const double WHEEL_RADIUS = 0.0325;
const double TRACK_WIDTH = 0.263;
const double CELL_SIZE = 0.87;
const double ARC_RADIUS = 0.4;
const double MAX_WHEEL_SPEED = 8;  // rad/s

// Direction enum (must match ExecutePlan.srv)
enum Direction {
    RIGHT = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3
};

ros::ServiceClient* diff_drive_client;

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    const double omega_robot = 2.0 * WHEEL_RADIUS * wheel_speed_rad_s / TRACK_WIDTH;
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

    ros::Duration(0.1).sleep();

}

void driveArc(double angle_rad) {
    double outer_speed = MAX_WHEEL_SPEED;
    double v_outer = WHEEL_RADIUS * outer_speed;
    double v_inner = v_outer * (ARC_RADIUS - TRACK_WIDTH/2) / (ARC_RADIUS + TRACK_WIDTH/2);
    double w_inner = v_inner / WHEEL_RADIUS;

    create_fundamentals::DiffDrive srv;
    if(angle_rad < 0) {  // Left turn
        srv.request.left = w_inner;
        srv.request.right = outer_speed;
    } else {  // Right turn
        srv.request.left = outer_speed;
        srv.request.right = w_inner;
    }

    double arc_length = std::abs(angle_rad * ARC_RADIUS);
    double duration = arc_length / ((v_outer + v_inner)/2);

    if(!diff_drive_client->call(srv)) {
        ROS_ERROR("Failed to execute arc");
    }
    ros::Duration(duration).sleep();
}

void moveLinear(ros::ServiceClient& diffDrive,
                double distance_m,
                double wheel_speed_rad_s)
{
    const double linear_speed = WHEEL_RADIUS * wheel_speed_rad_s;
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
}

bool executePlan(plastic_fundamentals::ExecutePlan::Request &req,
                 plastic_fundamentals::ExecutePlan::Response &res) 
{
    int current_heading = UP;
    size_t idx = 0;

    Direction target_dir = static_cast<Direction>(req.plan[idx]);
    if (target_dir != current_heading) {
        float dir = target_dir - 1;
        if(dir == 2) {
            spinInPlace(*diff_drive_client, M_PI, 5.0);
        } else {
            spinInPlace(*diff_drive_client, dir * M_PI_2, 5.0);
        }

        current_heading = target_dir;
    }

    //bool final_segment = (segment_end == req.plan.size());
    moveLinear(*diff_drive_client, CELL_SIZE / 2, 8.0);


    idx++;

    while(idx < req.plan.size()) {
        // Group consecutive same directions
        Direction target_dir = static_cast<Direction>(req.plan[idx]);
        size_t segment_end = idx;
        while(segment_end < req.plan.size() && req.plan[segment_end] == target_dir) {
            segment_end++;
        }
        int segment_length = segment_end - idx;

        // Handle direction change
        if(current_heading != target_dir) {
            int delta = (target_dir - current_heading + 4) % 4;
            double turn_angle = 0.0;

            if(delta == 1){
                turn_angle = -M_PI_2;      // Left turn
                driveArc(turn_angle);
            }
            
            else if(delta == 3){
                turn_angle = M_PI_2;  // Right turn
                driveArc(turn_angle);
            }
            else if(delta == 2){
                turn_angle = M_PI;    // rotate 180 degrees
                spinInPlace(*diff_drive_client, turn_angle, MAX_WHEEL_SPEED);
            }
            current_heading = target_dir;
            segment_length--;

        } /*else {
            moveLinear(segment_length * CELL_SIZE - CELL_SIZE / 2, final_segment);
        }*/


        if(segment_length > 0) {
            bool final_segment = (segment_end == req.plan.size());
            moveLinear(*diff_drive_client, segment_length * CELL_SIZE, 8.0);
        }

        bool final_segment = (segment_end == req.plan.size());
        if (final_segment) {
            moveLinear(*diff_drive_client, CELL_SIZE / 2, 8.0);
        }

        idx = segment_end;
    }

    // Full stop at end
    create_fundamentals::DiffDrive stop_srv;
    stop_srv.request.left = 0.0;
    stop_srv.request.right = 0.0;
    diff_drive_client->call(stop_srv);

    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "smooth_plan_executor");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;
    
    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlan);
    ROS_INFO("Smooth plan executor ready");
    ros::spin();
    return 0;
}