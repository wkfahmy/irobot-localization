#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <plastic_fundamentals/Move.h>
#include <plastic_fundamentals/ExecutePlan.h>
#include <plastic_fundamentals/MoveToPosition.h>
#include <plastic_fundamentals/Pose.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <plastic_fundamentals/AbsEncoder.h>
#include <algorithm>
#include <cmath>
#include "graph_utils.hpp"

double leftTicks = 0;
double rightTicks = 0;

std::vector<float> lidar_ranges;
double angle_min, angle_increment;
double min_range, max_range;
bool lidar_data_received = false;

struct Pose2D {
    int row, col;
    double x, y, theta;
};

Pose2D current_pose;
bool pose_received = false;

bool isFlying = false;
bool will_hit_obstacle = false;

const double ROBOT_RADIUS = 0.18;
const double LIDAR_OFFSET = 0.16;
const double SAFETY_MARGIN = 0.03;

const double danger_distance = ROBOT_RADIUS + LIDAR_OFFSET + SAFETY_MARGIN;

constexpr double LIDAR_OFFSET_M = 0.16;

constexpr float DESIRED_DISTANCE = 0.4f;
constexpr float KP_LATERAL = 0.8f;
constexpr float MAX_CORRECTION = 0.8f;
constexpr float FRONT_SLOW_THRESHOLD = 0.45f;
constexpr float FRONT_STOP_THRESHOLD = 0.38f;
constexpr float MAX_SIDE_RANGE = 0.8f;

constexpr double MAX_SPEED = 20;
constexpr double CELL_SIZE = 0.8;
constexpr double LOOKAHEAD_DIST = 0.4;
constexpr double MAX_STEP = 0.02;
constexpr double OBSTACLE_MARGIN = 0.1;

constexpr double TRACK_WIDTH_M = 0.263;
constexpr double WHEEL_RADIUS_M = 0.0325;

ros::ServiceClient diffDriveClient;
ros::ServiceClient resetEncodersClient;
ros::ServiceClient marker;
ros::Publisher absEncoderPub;

template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

int last_left_ticks = 0;
int last_right_ticks = 0;

void updateOdometry(Pose2D& pose, int new_left_ticks, int new_right_ticks) {
    int delta_left = new_left_ticks - last_left_ticks;
    int delta_right = new_right_ticks - last_right_ticks;
    last_left_ticks = new_left_ticks;
    last_right_ticks = new_right_ticks;

    double d_left = delta_left * WHEEL_RADIUS_M;
    double d_right = delta_right * WHEEL_RADIUS_M;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / TRACK_WIDTH_M;

    pose.x += d_center * std::cos(pose.theta + d_theta / 2.0);
    pose.y += d_center * std::sin(pose.theta + d_theta / 2.0);
    pose.theta += d_theta;

    while (pose.theta > M_PI) pose.theta -= 2 * M_PI;
    while (pose.theta < -M_PI) pose.theta += 2 * M_PI;
}

static ros::Time last_pose_time = ros::Time(0);

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;

    if(msg->wheeldropCaster == true || msg->wheeldropLeft == true || msg->wheeldropRight == true) {
        isFlying = true;
    } else {
        isFlying = false;
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    lidar_ranges = msg->ranges;
    angle_min = msg->angle_min;
    min_range = msg->range_min;
    max_range = msg->range_max;
    angle_increment = msg->angle_increment;
    lidar_data_received = true;
}

void poseCallback(const plastic_fundamentals::Pose::ConstPtr& msg) {
    current_pose.row = msg->row;
    current_pose.col = msg->column;
    current_pose.x = msg->x;
    current_pose.y = msg->y;
    current_pose.theta = msg->theta;

    pose_received = true;
}

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if (resetEncodersClient.call(srv)) {
        leftTicks = 0.0;
        rightTicks = 0.0;

        lidar_ranges.clear();
        lidar_data_received = false;

        plastic_fundamentals::AbsEncoder msg;
        msg.reset = true;
        absEncoderPub.publish(msg);
    } else {
        ROS_WARN("Failed to reset encoders.");
    }
}

Eigen::Vector2d cellToWorld(int i, int j) {
    return {i * CELL_SIZE + CELL_SIZE / 2.0, j * CELL_SIZE + CELL_SIZE / 2.0};
}

Eigen::Vector2d cubicBezier(double t, const Eigen::Vector2d& P0,
                            const Eigen::Vector2d& P1,
                            const Eigen::Vector2d& P2,
                            const Eigen::Vector2d& P3) {
    double u = 1.0 - t;
    return u*u*u*P0 + 3*u*u*t*P1 + 3*u*t*t*P2 + t*t*t*P3;
}

double getRotationTicks(double angle_rad) {

    return (TRACK_WIDTH_M * angle_rad) / (2.0 * WHEEL_RADIUS_M);
}

double getTranslationTicks(double distance) {
    constexpr double WHEEL_RADIUS_M = 0.0325;
    return (distance / WHEEL_RADIUS_M);
}

float calculateLateralError() {
    if (!lidar_data_received || lidar_ranges.empty()) return 0.0f;

    float left_min = MAX_SIDE_RANGE;
    float right_min = MAX_SIDE_RANGE;

    double angle = angle_min;
    for (size_t i = 0; i < lidar_ranges.size(); ++i, angle += angle_increment) {
        float r = lidar_ranges[i];
        if (r < 0.1f || r > MAX_SIDE_RANGE || std::isnan(r)) continue;

        if (angle > 1.396 && angle < 1.745) {
            left_min = std::min(left_min, r);
        }
        if (angle > -1.745 && angle < -1.396) {
            right_min = std::min(right_min, r);
        }
    }

    ROS_INFO_THROTTLE(1.0, "Left min: %.2f m | Right min: %.2f m", left_min, right_min);

    bool left_valid = left_min < MAX_SIDE_RANGE;
    bool right_valid = right_min < MAX_SIDE_RANGE;

    if (left_valid && right_valid) {
        float lateral_error = (right_min - left_min) / 2.0f;
        return clamp(lateral_error, -0.2f, 0.2f);
    } else if (left_valid) {
        return DESIRED_DISTANCE - left_min;
    } else if (right_valid) {
        return -(DESIRED_DISTANCE - right_min);
    }

    return 0.0f;
}


float calculateFrontDistance() {
    if (!lidar_data_received || lidar_ranges.empty()) return std::numeric_limits<double>::infinity();

    double front_min = std::numeric_limits<double>::infinity();
    double angle = angle_min;
    for (size_t i = 0; i < lidar_ranges.size(); ++i, angle += angle_increment) {
        float r = lidar_ranges[i];
        double sin_t = sin(angle);
        double cos_t = cos(angle);

        double x = r * cos_t + LIDAR_OFFSET_M;
        double y = r * sin_t;

        double distance = sqrt(x * x + y * y);
        if (distance < min_range || distance > max_range || std::isnan(r)) continue;

        if (angle > -0.26 && angle < 0.26) {
            front_min = std::min(front_min, distance);
        }
    }

    return front_min;
}

bool handleRotate(plastic_fundamentals::Move::Request &req,
                  plastic_fundamentals::Move::Response &res) {
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

    bool correctionEnabled = req.correction;

    double ticks = getRotationTicks(fabs(req.angle));
    double side = (req.angle > 0) ? 1.0 : -1.0;
    resetEncoders();

    double small_margin = ticks * 0.015;

    while ((abs(leftTicks) + abs(rightTicks)) / 2.0 < ticks - small_margin && ros::ok() && !isFlying) {
        double error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2.0);
        double error_based_speed = req.speed * (error / ticks);
        if (error_based_speed < req.speed / 4.0)
            error_based_speed = req.speed / 4.0;

        double correction = 0.0;
        if (fabs(rightTicks) > 1e-3) {
            correction = clamp(1.0 - fabs(leftTicks) / fabs(rightTicks), -0.5, 0.5);
        }

        if (correctionEnabled) {
            float lateral_error = calculateLateralError();
            correction += clamp(KP_LATERAL * lateral_error, -0.2f, 0.2f);
        }

        diffDriveSrv.request.left = -side * error_based_speed * (1.0 + correction / 2.0);
        diffDriveSrv.request.right =  side * error_based_speed * (1.0 - correction / 2.0);
        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = leftTicks;
        msg.abs_right = rightTicks;
        msg.reset = false;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient.call(diffDriveSrv);

    res.success = true;
    resetEncoders();
    return true;
}

bool handleTranslate(plastic_fundamentals::Move::Request &req,
                     plastic_fundamentals::Move::Response &res) {
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

    bool correctionEnabled = req.correction;

    double ticks = getTranslationTicks(fabs(req.distance));
    double side = (req.distance > 0) ? 1.0 : -1.0;
    resetEncoders();

    double small_margin = ticks * 0.005;

    while ((abs(leftTicks) + abs(rightTicks)) / 2.0 < ticks - small_margin && ros::ok() && !isFlying && !will_hit_obstacle) {
        double error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2.0);
        double error_based_speed = req.speed * (error / ticks);
        if (error_based_speed < req.speed / 2.0)
            error_based_speed = req.speed / 2.0;

        float front_dist = calculateFrontDistance();
        if (front_dist < FRONT_STOP_THRESHOLD) {
            ROS_WARN("Too close to front wall (%.2fm), stopping early!", front_dist);
            break;
        } else if (front_dist < FRONT_SLOW_THRESHOLD) {
            ROS_WARN_THROTTLE(1.0, "Approaching front wall (%.2fm), slowing down.", front_dist);
            error_based_speed *= 0.5;
        }

        double correction = 0.0;
        if (fabs(rightTicks) > 1e-3) {
            correction = clamp(1.0 - fabs(leftTicks)/fabs(rightTicks), -0.5, 0.5);
        }

        if (correctionEnabled) {
            float lateral_error = calculateLateralError();
            float lidar_correction = clamp(KP_LATERAL * lateral_error, -MAX_CORRECTION, MAX_CORRECTION);

            correction += lidar_correction;
        }

        diffDriveSrv.request.left = side * error_based_speed * (1.0 + correction / 2.0);
        diffDriveSrv.request.right = side * error_based_speed * (1.0 - correction / 2.0);

        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = leftTicks;
        msg.abs_right = rightTicks;
        msg.reset = false;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient.call(diffDriveSrv);
    resetEncoders();

    res.success = !will_hit_obstacle;
    return true;
}

bool isObstacleInFront(double max_forward = 0.3) {
    if (!lidar_data_received) return false;

    const double robot_radius = 0.18;
    const double safety_margin = 0.04;
    const double effective_radius = robot_radius + safety_margin;


    int center_index = static_cast<int>((0.0 - angle_min) / angle_increment);
    int half_window = static_cast<int>((25.0 * M_PI / 180.0) / angle_increment);

    for (int i = center_index - half_window; i <= center_index + half_window; ++i) {
        if (i < 0 || i >= static_cast<int>(lidar_ranges.size())) continue;

        float r = lidar_ranges[i];
        if (r < min_range || r > max_range) continue;

        double angle = angle_min + i * angle_increment;
        double x = r * std::cos(angle) + LIDAR_OFFSET;
        double y = r * std::sin(angle);

        if (x > 0 && x < max_forward && std::abs(y) < effective_radius) {
            return true;
        }
    }
    return false;
}


Pose2D getNextTarget(const std::vector<Eigen::Vector2d>& path) {
    Pose2D target;
    double best_score = 1e9;
    bool found = false;

    for (const auto& point : path) {
        double dx = point.x() - current_pose.x;
        double dy = point.y() - current_pose.y;
        double dist = std::hypot(dx, dy);
        if (dist < 0.05) continue;

        double heading = std::atan2(dy, dx);
        double angle_diff = heading - current_pose.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (dist > 0.1 && std::abs(angle_diff) < M_PI / 2.0 && !isObstacleInFront(dist)) {
            double score = dist + 0.5 * std::abs(angle_diff);
            if (score < best_score) {
                target.x = point.x();
                target.y = point.y();
                target.theta = heading;
                best_score = score;
                found = true;
            }
        }
    }

    if (!found && !path.empty()) {
        Eigen::Vector2d p = path.back();
        target.x = p.x();
        target.y = p.y();
        target.theta = std::atan2(p.y() - current_pose.y, p.x() - current_pose.x);
    }

    return target;
}

void stopRobot() {
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient.call(srv);
}


void followTarget(const Pose2D& target_pose) {
    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double rho = std::hypot(dx, dy);

    double alpha = std::atan2(dy, dx) - current_pose.theta;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    constexpr double k_rho = 120.0;          // Linear gain
    constexpr double k_alpha = 300.0;        // Higher angular gain for fast spin
    constexpr double angle_threshold = M_PI / 3.0;
    constexpr double close_angle_threshold = M_PI / 6.0;
    constexpr double distance_threshold = 0.3;

    bool blocked = lidar_data_received && isObstacleInFront();
    bool turn_in_place = std::abs(alpha) > angle_threshold || (rho < distance_threshold && std::abs(alpha) > close_angle_threshold) || blocked;

    double linear = 0.0;
    double angular = 0.0;

    if (!turn_in_place) {
        double curvature_penalty = std::clamp(std::cos(alpha), 0.1, 1.0);  // Smooth down linear speed in curves
        linear = std::clamp(k_rho * rho * curvature_penalty, -MAX_SPEED, MAX_SPEED);
        angular = std::clamp(k_alpha * alpha, -MAX_SPEED, MAX_SPEED);
    } else {
        // Full angular speed for spin in place
        angular = std::clamp(k_alpha * alpha, -2.0 * MAX_SPEED, 2.0 * MAX_SPEED);
    }

    double v_left = linear - (TRACK_WIDTH_M / 2.0) * angular;
    double v_right = linear + (TRACK_WIDTH_M / 2.0) * angular;

    v_left = std::clamp(v_left, -MAX_SPEED, MAX_SPEED);
    v_right = std::clamp(v_right, -MAX_SPEED, MAX_SPEED);

    create_fundamentals::DiffDrive srv;
    srv.request.left = v_left;
    srv.request.right = v_right;
    diffDriveClient.call(srv);
}



void publishMarkerPath(const std::vector<Eigen::Vector2d>& path) {
    plastic_fundamentals::PublishMarker srv;
    srv.request.marker_type = "BezierMarker";

    for (const auto& pt : path) {
        plastic_fundamentals::Point p;
        p.x = pt.x();
        p.y = pt.y();
        srv.request.points.push_back(p);
    }

    marker.call(srv);
}

void publishControlPath(const std::vector<Eigen::Vector2d>& path) {
    plastic_fundamentals::PublishMarker srv;
    srv.request.marker_type = "ControlMarker";

    for (const auto& pt : path) {
        plastic_fundamentals::Point p;
        p.x = pt.x();
        p.y = pt.y();
        srv.request.points.push_back(p);
    }

    marker.call(srv);
}

std::vector<std::pair<int, int>> planToCells(int start_x, int start_y, const std::vector<int>& plan) {
    std::vector<std::pair<int, int>> cells;
    int x = start_x;
    int y = start_y;
    cells.emplace_back(x, y);
    for (int dir : plan) {
        if (dir == 0) y += 1;
        else if (dir == 2) y -= 1;
        else if (dir == 1) x -= 1;
        else if (dir == 3) x += 1;
        cells.emplace_back(x, y);
    }

    std::vector<Eigen::Vector2d> cell_points;

    for (int i = 0; i < cells.size(); ++i) {
        cell_points.push_back({cells[i].first * CELL_SIZE, cells[i].second * CELL_SIZE});
        ROS_INFO("Cell %d: (%d, %d)", i, cells[i].first, cells[i].second);
    }


    return cells;
}

std::vector<Eigen::Vector2d> generateBezierPath(const Pose2D& start_pose, const std::vector<std::pair<int, int>>& cells) {
    std::vector<Eigen::Vector2d> path;
    if (cells.empty()) return path;

    Eigen::Vector2d current_pos(start_pose.x, start_pose.y);

    std::vector<Eigen::Vector2d> points;
    points.push_back(current_pos);
    for (const auto& cell : cells) {
        points.push_back(cellToWorld(cell.first, cell.second));
    }

    for (size_t i = 0; i + 2 < points.size(); i += 2) {
        Eigen::Vector2d P0 = points[i];
        Eigen::Vector2d P1 = points[i + 1];
        Eigen::Vector2d P2 = points[i + 2];

        for (double t = 0.0; t <= 1.0; t += MAX_STEP) {
            Eigen::Vector2d pt = std::pow(1 - t, 2) * P0 + 2 * (1 - t) * t * P1 + std::pow(t, 2) * P2;
            path.push_back(pt);
        }
    }

    if ((points.size() - 1) % 2 != 0 && points.size() >= 2) {
        Eigen::Vector2d P0 = points[points.size() - 2];
        Eigen::Vector2d P1 = points.back();
        for (double t = 0.0; t <= 1.0; t += MAX_STEP) {
            Eigen::Vector2d pt = (1 - t) * P0 + t * P1;
            path.push_back(pt);
        }
    }

    publishControlPath(points);

    return path;
}

bool shouldReplan(const Pose2D& initial_pose, const Pose2D& current_pose, ros::Time start_time) {
    const double REPLAN_DISTANCE_THRESHOLD = 0.8; // meters
    const double STUCK_TIME_THRESHOLD = 10.0; // seconds
    static Pose2D last_pose = current_pose;
    static ros::Time last_move_time = start_time;

    double dx = current_pose.x - last_pose.x;
    double dy = current_pose.y - last_pose.y;
    double moved = std::hypot(dx, dy);

    if (moved > 0.05) {
        last_move_time = ros::Time::now();
        last_pose = current_pose;
    }

    if (std::hypot(current_pose.x - initial_pose.x, current_pose.y - initial_pose.y) > REPLAN_DISTANCE_THRESHOLD) {
        ROS_WARN("Robot deviated too far from planned path. Triggering replanning.");
        return true;
    }

    if ((ros::Time::now() - last_move_time).toSec() > STUCK_TIME_THRESHOLD) {
        ROS_WARN("Robot seems to be stuck. Triggering replanning.");
        return true;
    }

    return false;
}

void executePath(std::vector<std::pair<int, int>>& cells, ros::Rate rate, std::function<bool()> shouldReplanCallback) {
    resetEncoders();

    while (ros::ok() && !cells.empty()) {
        if (shouldReplanCallback()) {
            resetEncoders();
            ROS_WARN("Replanning requested by callback during path execution.");
            break;
        }

        Eigen::Vector2d goal = cellToWorld(cells.front().first, cells.front().second);
        double dx = goal.x() - current_pose.x;
        double dy = goal.y() - current_pose.y;
        if (std::hypot(dx, dy) < 0.3) {
            cells.erase(cells.begin());
            continue;
        }

        int n = std::min(4, static_cast<int>(cells.size()));
        std::vector<std::pair<int, int>> segment(cells.begin(), cells.begin() + n);
        std::vector<Eigen::Vector2d> path = generateBezierPath(current_pose, segment);
        publishMarkerPath(path);

        Pose2D target = getNextTarget(path);
        followTarget(target);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = leftTicks;
        msg.abs_right = rightTicks;
        msg.reset = false;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    create_fundamentals::DiffDrive stop;
    stop.request.left = 0;
    stop.request.right = 0;
    diffDriveClient.call(stop);

    resetEncoders();
}

std::mutex pose_mutex;

bool moveToPosition(plastic_fundamentals::MoveToPosition::Request &req, plastic_fundamentals::MoveToPosition::Response &res) {
    int goal_row = req.row;
    int goal_col = req.column;
    std::pair<int, int> goal = {goal_row, goal_col};

    ROS_INFO("Planning path to (%d, %d)", goal.first, goal.second);

    bool success = false;
    const int max_attempts = 3;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        int start_row, start_col;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            if (!pose_received) {
                ROS_ERROR("Current pose not received yet. Cannot plan path.");
                res.success = false;
                return true;
            }
            start_row = current_pose.row;
            start_col = current_pose.col;
        }

        std::pair<int, int> start = {start_row, start_col};
        auto graph = createGraph();
        std::vector<std::pair<int, int>> path_positions = findShortestPath(graph, start, goal);

        if (path_positions.empty()) {
            ROS_WARN("No valid path found to the goal (%d, %d).", goal_row, goal_col);
            res.success = false;
            return true;
        }

        Pose2D initial_pose = current_pose;
        ros::Time start_time = ros::Time::now();

        executePath(path_positions, ros::Rate(30), [&]() {
            return shouldReplan(initial_pose, current_pose, start_time);
        });

        // Vérifie si on est suffisamment proche de la destination
        double dx = current_pose.x - cellToWorld(goal.first, goal.second).x();
        double dy = current_pose.y - cellToWorld(goal.first, goal.second).y();
        if (std::hypot(dx, dy) < 0.4) {
            success = true;
            ROS_INFO("Successfully executed path to (%d, %d) on attempt %d.", goal_row, goal_col, attempt);
            break;
        } else {
            ROS_WARN("Execution failed or replanned on attempt %d.", attempt);
            ros::Duration(0.5).sleep();
        }
    }

    res.success = success;
    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("sensor_packet", 1, sensorCallback);
    ros::Subscriber lidar_sub = nh.subscribe("scan_filtered", 1, lidarCallback);
    ros::Subscriber pose_sub = nh.subscribe("pose", 1, poseCallback);
    diffDriveClient = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    resetEncodersClient = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    absEncoderPub = nh.advertise<plastic_fundamentals::AbsEncoder>("absolute_encoders", 10);

    ros::ServiceServer rotateSrv = nh.advertiseService("perform_rotation", handleRotate);
    ros::ServiceServer translateSrv = nh.advertiseService("perform_translation", handleTranslate);
    ros::ServiceServer moveSrv = nh.advertiseService("move_to_position", moveToPosition);

    ROS_INFO("LIDAR-enhanced Encoder Mover with Front Wall Detection ready.");
    ros::spin();
    return 0;
}
