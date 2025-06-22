#include <ros/ros.h>
#include <plastic_fundamentals/Move.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <plastic_fundamentals/AbsEncoder.h>
#include <algorithm>
#include <cmath>

double leftTicks = 0;
double rightTicks = 0;

std::vector<float> lidar_ranges;
float angle_min, angle_increment;
bool lidar_data_received = false;

bool isFlying = false;
bool will_hit_obstacle = false;

const double ROBOT_RADIUS = 0.18;
const double LIDAR_OFFSET = 0.16;
const double SAFETY_MARGIN = 0.03;

const double danger_distance = ROBOT_RADIUS + LIDAR_OFFSET + SAFETY_MARGIN;


constexpr float DESIRED_DISTANCE = 0.4f;
constexpr float MAX_LIDAR_RANGE = 1.0f;
constexpr float KP_LATERAL = 0.8f;
constexpr float MAX_CORRECTION = 0.8f;
constexpr float FRONT_SLOW_THRESHOLD = 0.4f;
constexpr float FRONT_STOP_THRESHOLD = 0.3f;
constexpr float MAX_SIDE_RANGE = 0.8f;

ros::ServiceClient diffDriveClient;
ros::ServiceClient resetEncodersClient;
ros::Publisher absEncoderPub;

template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

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
    angle_increment = msg->angle_increment;
    lidar_data_received = true;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;

        // Garder seulement les angles proches de 0° (devant)
        if (std::abs(angle) <= M_PI / 24) {
            double distance = msg->ranges[i];

            if (distance >= msg->range_min && distance <= msg->range_max) {
                if (distance < danger_distance) {
                    will_hit_obstacle = true;
                    return;
                }
            }
        }
    }
    will_hit_obstacle = false;
}

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if (resetEncodersClient.call(srv)) {
        leftTicks = 0.0;
        rightTicks = 0.0;

        lidar_ranges.clear();
        lidar_data_received = false;
    } else {
        ROS_WARN("Failed to reset encoders.");
    }
}

double getRotationTicks(double angle_rad) {
    constexpr double TRACK_WIDTH_M = 0.263;
    constexpr double WHEEL_RADIUS_M = 0.0325;
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

        if (angle > 1.396 && angle < 1.745) { // +90° ±10°
            left_min = std::min(left_min, r);
        }
        if (angle > -1.745 && angle < -1.396) { // -90° ±10°
            right_min = std::min(right_min, r);
        }
    }

    ROS_INFO_THROTTLE(1.0, "Left min: %.2f m | Right min: %.2f m", left_min, right_min);

    bool left_valid = left_min < MAX_SIDE_RANGE;
    bool right_valid = right_min < MAX_SIDE_RANGE;

    if (left_valid && right_valid) {
        // Use both sides to center
        float lateral_error = (right_min - left_min) / 2.0f;
        return clamp(lateral_error, -0.2f, 0.2f); // Optional clamp to limit overreaction
    } else if (left_valid) {
        return DESIRED_DISTANCE - left_min;
    } else if (right_valid) {
        return -(DESIRED_DISTANCE - right_min);
    }

    return 0.0f;
}


float calculateFrontDistance() {
    if (!lidar_data_received || lidar_ranges.empty()) return MAX_LIDAR_RANGE;

    float front_min = MAX_LIDAR_RANGE;
    double angle = angle_min;
    for (size_t i = 0; i < lidar_ranges.size(); ++i, angle += angle_increment) {
        float r = lidar_ranges[i];
        if (r < 0.1f || r > MAX_LIDAR_RANGE || std::isnan(r)) continue;

        if (angle > -0.26 && angle < 0.26) {  // -15° to +15°
            front_min = std::min(front_min, r);
        }
    }

    return front_min;
}

bool handleRotate(plastic_fundamentals::Move::Request &req,
                  plastic_fundamentals::Move::Response &res) {
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

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

        float lateral_error = calculateLateralError();
        correction += clamp(KP_LATERAL * lateral_error, -0.2f, 0.2f);

        diffDriveSrv.request.left = -side * error_based_speed * (1.0 + correction / 2.0);
        diffDriveSrv.request.right =  side * error_based_speed * (1.0 - correction / 2.0);
        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = leftTicks;
        msg.abs_right = rightTicks;
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

bool handleTranslate(plastic_fundamentals::Move::Request &req,
                     plastic_fundamentals::Move::Response &res) {
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

    double ticks = getTranslationTicks(fabs(req.distance));
    double side = (req.distance > 0) ? 1.0 : -1.0;
    resetEncoders();

    double small_margin = ticks * 0.01;

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

        double encoder_correction = 0.0;
        if (fabs(rightTicks) > 1e-3) {
            encoder_correction = clamp(1.0 - fabs(leftTicks)/fabs(rightTicks), -0.5, 0.5);
        }

        float lateral_error = calculateLateralError();
        float lidar_correction = clamp(KP_LATERAL * lateral_error, -MAX_CORRECTION, MAX_CORRECTION);

        double total_correction = encoder_correction + lidar_correction;

        diffDriveSrv.request.left = side * error_based_speed * (1.0 + total_correction / 2.0);
        diffDriveSrv.request.right = side * error_based_speed * (1.0 - total_correction / 2.0);

        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = leftTicks;
        msg.abs_right = rightTicks;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient.call(diffDriveSrv);
    resetEncoders();

    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("sensor_packet", 1, sensorCallback);
    ros::Subscriber lidar_sub = nh.subscribe("scan_filtered", 1, lidarCallback);
    diffDriveClient = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    resetEncodersClient = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    absEncoderPub = nh.advertise<plastic_fundamentals::AbsEncoder>("absolute_encoders", 10);

    ros::ServiceServer rotateSrv = nh.advertiseService("perform_rotation", handleRotate);
    ros::ServiceServer translateSrv = nh.advertiseService("perform_translation", handleTranslate);

    ROS_INFO("LIDAR-enhanced Encoder Mover with Front Wall Detection ready.");
    ros::spin();
    return 0;
}
