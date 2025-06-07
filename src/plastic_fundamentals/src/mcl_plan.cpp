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

double getRotationTicks(double angle_rad) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double wheel_travel_distance = (TRACK_WIDTH_M * angle_rad) / 2;
    return (wheel_travel_distance / wheel_circumference) * ticks_per_revolution;
}

double getTranslationTicks(double distance) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    return (distance / wheel_circumference) * ticks_per_revolution;
}


void rotate(double angle_rad, double speed) {
    ROS_INFO("Rotation: %f", angle_rad);

    double ticks = getRotationTicks(angle_rad);

    resetEncoders();
    ros::spinOnce();

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    diffDriveSrv.request.left = -side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while (ticks * 0.98 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 4)) {
            error_based_speed = (speed / 4);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = -side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        double delta_left = leftTicks - last_left_ticks;
        double delta_right = rightTicks - last_right_ticks;
        double delta_ticks = (std::abs(delta_left) + std::abs(delta_right)) / 2.0;
        double incremental_rotation = (delta_ticks / ticks) * angle_rad;
        updateParticlesMotion(0.0, incremental_rotation);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    double delta_left = leftTicks - last_left_ticks;
    double delta_right = rightTicks - last_right_ticks;
    double delta_ticks = (delta_left + delta_right) / 2.0;
    double incremental_rotation = (delta_ticks / ticks) * angle_rad;
    updateParticlesMotion(0.0, incremental_rotation);

    resetEncoders();
}


void translate(double distance, double speed) {
    ROS_INFO("Translation: %f", distance);

    double ticks = getTranslationTicks(distance);

    resetEncoders();
    ros::spinOnce();

    double side = (distance > 0) ? 1.0 : -1.0;
    diffDriveSrv.request.left = side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(10);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while (ticks * 0.99 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 2)) {
            error_based_speed = (speed / 2);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        double delta_left = leftTicks - last_left_ticks;
        double delta_right = rightTicks - last_right_ticks;
        double delta_ticks = (delta_left + delta_right) / 2.0;
        double incremental_distance = (delta_ticks / ticks) * distance;
        updateParticlesMotion(incremental_distance, 0.0);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    double delta_left = leftTicks - last_left_ticks;
    double delta_right = rightTicks - last_right_ticks;
    double incremental_distance = side * (delta_left + delta_right) / 2.0 * WHEEL_RADIUS_M;

    updateParticlesMotion(incremental_distance, 0.0);

    resetEncoders();
}

void driveArc(double angle_rad, double arc_radius, double speed) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;

    double d_left  = (arc_radius - TRACK_WIDTH_M/2.0) * std::abs(angle_rad);
    double d_right = (arc_radius + TRACK_WIDTH_M/2.0) * std::abs(angle_rad);

    double ticks_left  = (d_left  / wheel_circumference) * ticks_per_revolution;
    double ticks_right = (d_right / wheel_circumference) * ticks_per_revolution;

    double turn = (angle_rad > 0) ? 1.0 : -1.0;

    resetEncoders();
    ros::spinOnce();

    double v_left  = speed * (d_left  / std::max(d_right, d_left));
    double v_right = speed * (d_right / std::max(d_right, d_left));

    if (angle_rad < 0) {
        diffDriveSrv.request.left =  turn * v_left;
        diffDriveSrv.request.right = turn * v_right;
    } else {
        diffDriveSrv.request.left =  turn * v_left;
        diffDriveSrv.request.right = turn * v_right;
    }

    ros::Rate rate(100);

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while ((std::abs(leftTicks) < std::abs(ticks_left)) || (std::abs(rightTicks) < std::abs(ticks_right))) {
        double corr = 0.0;
        if (rightTicks != 0.0 && leftTicks != 0.0) {
            double ratio = (d_left > d_right) ?
                (std::abs(leftTicks) / std::abs(rightTicks)) :
                (std::abs(rightTicks) / std::abs(leftTicks));
            corr = 1 - ratio;
            diffDriveSrv.request.left  = turn * v_left  * (1 + corr/2);
            diffDriveSrv.request.right = turn * v_right * (1 - corr/2);
        }

        diffDriveClient->call(diffDriveSrv);

        double delta_left = leftTicks - last_left_ticks;
        double delta_right = rightTicks - last_right_ticks;
        double avg_arc = (delta_left * d_left / ticks_left + delta_right * d_right / ticks_right) / 2.0;
        double incremental_angle = (delta_right - delta_left) * wheel_circumference / (TRACK_WIDTH_M * ticks_per_revolution);
        updateParticlesMotion(avg_arc, incremental_angle);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();
}

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
                driveArc(turn_angle, 0.4, MAX_WHEEL_SPEED);
            }
            
            else if(delta == 3){
                turn_angle = M_PI_2;  // Right turn
                driveArc(turn_angle, 0.4, MAX_WHEEL_SPEED);
            }
            else if(delta == 2){
                turn_angle = M_PI;    // rotate 180 degrees
                rotate(turn_angle, MAX_WHEEL_SPEED);
            }
            current_heading = target_dir;
            segment_length--;
        }

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
    ros::init(argc, argv, "execute_smooth_plan_server");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;
    
    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlan);
    ROS_INFO("Smooth plan executor ready");
    ros::spin();
    return 0;
}