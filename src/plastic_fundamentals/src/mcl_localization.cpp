#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include "create_fundamentals/SensorPacket.h"
#include <plastic_fundamentals/PublishMarker.h>
#include <create_fundamentals/PlaySong.h>
#include <create_fundamentals/StoreSong.h>
#include <plastic_fundamentals/Grid.h>
#include <plastic_fundamentals/Row.h>
#include <plastic_fundamentals/Cell.h>
#include <plastic_fundamentals/Pose.h>
#include <plastic_fundamentals/ExecutePlan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>


constexpr double WHEEL_RADIUS_M = 0.0325;
constexpr double TRACK_WIDTH_M = 0.263;
constexpr double LIDAR_OFFSET_M = 0.16;
constexpr double CELL_SIZE = 0.8;

constexpr int NUM_PARTICLES = 1000;
constexpr double XY_INIT_NOISE = 0.1;
constexpr double THETA_INIT_NOISE = 0.2;

constexpr double XY_MOTION_NOISE = 0.1;
constexpr double THETA_MOTION_NOISE = 0.1;

constexpr double XY_RESAMPLE_NOISE = 0.1;
constexpr double THETA_RESAMPLE_NOISE = 0.2;

constexpr double RESAMPLE_THRESHOLD = 0.4;

bool isFlying = false; // Flag to indicate if the robot is not on the ground

double leftTicks = 0;
double rightTicks = 0;

enum LocalizationPhase {
    SPIN,
    MOVE
};


// Particle structure
struct Particle {
    double x;
    double y;
    double theta;
    double weight;
    int row;            // Grid row
    int col;            // Grid column
};


ros::ServiceClient* resetEncodersClient;

ros::ServiceClient* storeSong;
ros::ServiceClient* playSong;

create_fundamentals::DiffDrive diffDriveSrv;
ros::ServiceClient* diffDriveClient;
plastic_fundamentals::Grid::ConstPtr map_data;
Particle current_pose;

ros::ServiceClient marker;


bool is_localized = false;
bool first_localization = true;

bool processing_done = false;

bool pathClear = false;

ros::Publisher particle_pub;
ros::Publisher pose_pub;
std::default_random_engine random_generator;

sensor_msgs::LaserScan::ConstPtr actual_scan;

std::vector<plastic_fundamentals::Line> map_lines;

std::vector<Particle> particles;

int angleToDirection(double angle_rad) {
    double angle = angle_rad - M_PI/2;

    while (angle < 0) angle += 2*M_PI;
    while (angle >= 2*M_PI) angle -= 2*M_PI;

    int dir = static_cast<int>(std::floor((angle + M_PI/4) / (M_PI/2))) % 4;
    return dir;
}

void initializeParticles(const plastic_fundamentals::Grid::ConstPtr& map) {
    if (!map) {
        ROS_ERROR("Cannot initialize particles: No map data");
        return;
    }
    
    particles.clear();
    particles.reserve(NUM_PARTICLES);
    
    std::uniform_real_distribution<double> xy_noise_dist(-XY_INIT_NOISE, XY_INIT_NOISE);
    std::uniform_real_distribution<double> theta_noise_dist(-THETA_INIT_NOISE, THETA_INIT_NOISE);
    
    int num_rows = map->rows.size();
    int num_cols = map->rows[0].cells.size();
    int total_cells = num_rows * num_cols;
    
    int particles_per_cell = NUM_PARTICLES / (total_cells * 4);
    if (particles_per_cell < 1) particles_per_cell = 1;
    
    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_cols; ++col) {
            double cell_center_x = (row + 0.5) * CELL_SIZE;
            double cell_center_y = (col + 0.5) * CELL_SIZE;
            
            for (int orientation = 0; orientation < 4; ++orientation) {
                double base_theta = (orientation - 1) * M_PI / 2.0;

                for (int i = 0; i < particles_per_cell; ++i) {
                    if (particles.size() >= NUM_PARTICLES) break;
                    
                    Particle p;
                    p.x = cell_center_x + xy_noise_dist(random_generator);
                    p.y = cell_center_y + xy_noise_dist(random_generator);
                    p.theta = base_theta + theta_noise_dist(random_generator);
                    p.weight = 1.0 / NUM_PARTICLES;
                    p.row = row;
                    p.col = col;

                    particles.push_back(p);
                }
            }
        }
    }
    
    while (particles.size() < NUM_PARTICLES) {
        int idx = rand() % particles.size();
        particles.push_back(particles[idx]);
    }
    
    ROS_INFO("Initialized %zu particles", particles.size());

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointMarker";

    for (auto& p : particles) {
        plastic_fundamentals::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.w = p.theta;
        srv.request.points.push_back(pt);
    }

    if (!marker.call(srv)) {
        ROS_ERROR("Failed to call service marker_service");
    }

}

void updateParticlesMotion(double distance, double angle) {
    double dist_stddev = std::abs(distance) * XY_MOTION_NOISE;
    double angle_stddev = std::abs(angle) * THETA_MOTION_NOISE;

    std::normal_distribution<double> translation_noise(0, dist_stddev);
    std::normal_distribution<double> rotation_noise(0, angle_stddev);

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointMarker";

    for (auto& p : particles) {
        if (distance != 0) {
            double distance_noise = distance + translation_noise(random_generator);
            p.x += distance_noise * std::cos(p.theta);
            p.y += distance_noise * std::sin(p.theta);
        }

        if (angle != 0) {
            p.theta += angle + rotation_noise(random_generator);
        }

        while (p.theta > M_PI) p.theta -= 2 * M_PI;
        while (p.theta <= -M_PI) p.theta += 2 * M_PI;

        p.col = static_cast<int>(p.y / CELL_SIZE);
        p.row = static_cast<int>(p.x / CELL_SIZE);

        plastic_fundamentals::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.w = p.theta;

        srv.request.points.push_back(pt);
    }

    if (!marker.call(srv)) {
        ROS_ERROR("Failed to call service marker_service");
    }
}

void normalizeWeights() {
    double sum = 0.0;
    for (const auto& p : particles) {
        sum += p.weight;
    }
    
    if (sum > 0) {
        for (auto& p : particles) {
            p.weight /= sum;
        }
    } else {
        // If all weights are zero, reset to uniform
        double uniform_weight = 1.0 / particles.size();
        for (auto& p : particles) {
            p.weight = uniform_weight;
        }
    }
}

double calculateEffectiveParticles() {
    double sum_squared_weights = 0.0;
    double total_weight = 0.0;

    for (const auto& p : particles) {
        total_weight += p.weight;
        sum_squared_weights += p.weight * p.weight;
    }

    if (total_weight > 0.0) {
        return 1.0 / sum_squared_weights * total_weight * total_weight;
    }

    return 0.0;
}

void resampleParticles() {
    double n_eff = calculateEffectiveParticles();
    if (n_eff > RESAMPLE_THRESHOLD * particles.size()) {
        //ROS_INFO("Skipping resampling, effective particles: %.2f", n_eff);
        return;
    }
    
    ROS_INFO("Resampling particles, effective particles: %.2f", n_eff);
    
    std::vector<Particle> new_particles;
    new_particles.reserve(particles.size());
    
    std::vector<double> cumulative_sum(particles.size());
    cumulative_sum[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i) {
        cumulative_sum[i] = cumulative_sum[i-1] + particles[i].weight;
    }
    
    std::uniform_real_distribution<double> dist(0, 1.0 / particles.size());
    double r = dist(random_generator);
    size_t i = 0;

    std::normal_distribution<double> pos_noise(0, XY_RESAMPLE_NOISE);
    std::normal_distribution<double> ang_noise(0, THETA_RESAMPLE_NOISE);

    for (size_t m = 0; m < particles.size(); ++m) {
        double u = r + m * (1.0 / particles.size());
        while (u > cumulative_sum[i] && i < particles.size() - 1) {
            i++;
        }

        Particle p = particles[i];

        p.x += pos_noise(random_generator);
        p.y += pos_noise(random_generator);
        p.theta += ang_noise(random_generator);

        while (p.theta > M_PI) p.theta -= 2 * M_PI;
        while (p.theta <= -M_PI) p.theta += 2 * M_PI;

        new_particles.push_back(particles[i]);
    }
    
    particles = new_particles;
    
    double uniform_weight = 1.0 / particles.size();

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointMarker";

    for (auto& p : particles) {
        p.weight = uniform_weight;

        plastic_fundamentals::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.w = p.theta;
        srv.request.points.push_back(pt);
    }

    if (!marker.call(srv)) {
        ROS_ERROR("Failed to call service marker_service");
    }
}

void getMapLines(const plastic_fundamentals::Grid::ConstPtr& msg) {
    map_lines.clear();
    int rows = msg->rows.size();
    int cols = msg->rows[0].cells.size();

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            const auto& cell = msg->rows[i].cells[j];
            double x = i * CELL_SIZE;
            double y = j * CELL_SIZE;

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::LEFT) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x;
                line.y1 = y;
                line.x2 = x + CELL_SIZE;
                line.y2 = y;
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::BOTTOM) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x + CELL_SIZE;
                line.y1 = y;
                line.x2 = x + CELL_SIZE;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::RIGHT) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x;
                line.y1 = y + CELL_SIZE;
                line.x2 = x + CELL_SIZE;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::TOP) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x;
                line.y1 = y;
                line.x2 = x;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }
        }
    }
}


bool checkIntersection(const plastic_fundamentals::Line& ray, const plastic_fundamentals::Line& segment, double& intersect_x, double& intersect_y) {
    double det = (ray.x1 - ray.x2) * (segment.y1 - segment.y2) - (ray.y1 - ray.y2) * (segment.x1 - segment.x2);

    const double epsilon = 1e-6;
    if (std::fabs(det) < epsilon) {
        return false;
    }

    double t = ((ray.x1 - segment.x1) * (segment.y1 - segment.y2) - (ray.y1 - segment.y1) * (segment.x1 - segment.x2)) / det;
    double s = ((ray.x1 - segment.x1) * (ray.y1 - ray.y2) - (ray.y1 - segment.y1) * (ray.x1 - ray.x2)) / det;

    if (t >= 0 && s >= 0 && s <= 1) {
        intersect_x = ray.x1 + t * (ray.x2 - ray.x1);
        intersect_y = ray.y1 + t * (ray.y2 - ray.y1);
        return true;
    }

    return false;
}

double getRayDistance(const plastic_fundamentals::Grid& grid, double px, double py, double ray_angle_rad) {
    double dx = cos(ray_angle_rad);
    double dy = sin(ray_angle_rad);

    double min_dist = std::numeric_limits<double>::infinity();
    double intersect_x, intersect_y;

    for (const auto& line : map_lines) {
        plastic_fundamentals::Line ray;
        ray.x1 = px;
        ray.y1 = py;
        ray.x2 = px + dx;
        ray.y2 = py + dy;

        if (checkIntersection(ray, line, intersect_x, intersect_y)) {
            double dist = std::sqrt(std::pow(intersect_x - px, 2) + std::pow(intersect_y - py, 2));
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }

    return (min_dist != std::numeric_limits<double>::infinity()) ? min_dist : std::numeric_limits<double>::infinity();
}

double getDistanceToWall(double angle, double lidar_origin_x, double lidar_origin_y) {
    plastic_fundamentals::Line ray;
    ray.x1 = lidar_origin_x;
    ray.y1 = lidar_origin_y;

    ray.x2 = lidar_origin_x + cos(angle);
    ray.y2 = lidar_origin_y + sin(angle);

    double closest_distance = 1.0;
    double closest_intersect_x, closest_intersect_y;

    for (const auto& wall : map_lines) {
        double intersect_x, intersect_y;
        if (checkIntersection(ray, wall, intersect_x, intersect_y)) {
            double distance = std::sqrt(std::pow(intersect_x - lidar_origin_x, 2) + std::pow(intersect_y - lidar_origin_y, 2));
            if (distance < closest_distance) {
                closest_distance = distance;
            }
        }
    }

    return closest_distance;
}

std::vector<double> calculateExpectedScan(const Particle& p, const std::vector<double>& angles) {
    std::vector<double> expected_distances;
    expected_distances.reserve(angles.size());

    double lidar_x = p.x + LIDAR_OFFSET_M * cos(p.theta);
    double lidar_y = p.y + LIDAR_OFFSET_M * sin(p.theta);

    for (double angle : angles) {
        double ray_angle = p.theta + angle;
        double dist = getDistanceToWall(ray_angle, lidar_x, lidar_y);
        expected_distances.push_back(dist);
    }
    return expected_distances;
}

void updateParticlesSensor(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<double> angles;
    for (size_t i = 0; i < scan->ranges.size(); i += 10) {
        angles.push_back(scan->angle_min + i * scan->angle_increment);
    }
    
    for (auto& p : particles) {
        std::vector<double> expected_scan = calculateExpectedScan(p, angles);
        
        double likelihood = 1.0;
        int valid_rays = 0;
        
        for (size_t i = 0, j = 0; i < scan->ranges.size() && j < angles.size(); i += 10, ++j) {
            float measured = scan->ranges[i];
            double expected = expected_scan[j];
            
            if (std::isnan(measured) || measured < scan->range_min || measured > scan->range_max) {
                continue;
            }


            if (expected > 0.95 || std::isinf(expected)) {
                continue;
            }
            
            double sigma = 0.1;
            double diff = measured - expected;
            double prob = exp(-0.5 * diff * diff / (sigma * sigma));
            
            likelihood *= prob;
            valid_rays++;
        }
        
        if (valid_rays > 0) {
            p.weight *= pow(likelihood, 1.0 / valid_rays);
        } else {
            p.weight *= 0.1;
        }
    }
    
    normalizeWeights();
}

Particle estimatePose() {
    Particle pose;
    
    auto max_it = std::max_element(particles.begin(), particles.end(),
                                  [](const Particle& a, const Particle& b) {
                                      return a.weight < b.weight;
                                  });
    
    if (max_it != particles.end()) {
        pose.x = max_it->x;
        pose.y = max_it->y;
        pose.theta = max_it->theta;
        pose.row = max_it->row;
        pose.col = max_it->col;
    } else {
        double sum_x = 0, sum_y = 0;
        double sum_sin_theta = 0, sum_cos_theta = 0;
        double sum_weights = 0;
        
        for (const auto& p : particles) {
            sum_x += p.weight * p.x;
            sum_y += p.weight * p.y;
            sum_sin_theta += p.weight * sin(p.theta);
            sum_cos_theta += p.weight * cos(p.theta);
            sum_weights += p.weight;
        }
        
        if (sum_weights > 0) {
            double avg_x = sum_x / sum_weights;
            double avg_y = sum_y / sum_weights;
            double avg_theta = atan2(sum_sin_theta, sum_cos_theta);
            double normalized_theta = avg_theta;
            while (normalized_theta < 0) normalized_theta += 2 * M_PI;

            pose.x = avg_x;
            pose.y = avg_y;
            pose.theta = normalized_theta;

            pose.row = static_cast<int>(avg_x / CELL_SIZE);
            pose.col = static_cast<int>(avg_y / CELL_SIZE);
        }
    }
    
    return pose;
}


double k_rotation = 1.03; // Correction factor for rotation
double k_translation = 1.0; // Correction factor for translation

double absEncoderLeft = 0.0;
double absEncoderRight = 0.0;

double lastLeftTicks = 0.0;
double lastRightTicks = 0.0;

bool skipNextSensorCallback = false;

void resetEncoders() {
    ROS_INFO("Resetting the encoders");
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        absEncoderLeft += leftTicks;
        absEncoderRight += rightTicks;

        leftTicks = 0;
        rightTicks = 0;

        lastLeftTicks  = absEncoderLeft;
        lastRightTicks = absEncoderRight;
    }
}



void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    if(msg->wheeldropCaster == true || msg->wheeldropLeft == true || msg->wheeldropRight == true) {
        isFlying = true;
        diffDriveSrv.request.left = 0;
        diffDriveSrv.request.right = 0;
        diffDriveClient->call(diffDriveSrv);
        ROS_WARN("Wheeldrop detected, shutting down the node");
        resetEncoders();
         ros::shutdown();
        return;
    }
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;

    double delta_left = (absEncoderLeft + leftTicks) - lastLeftTicks;
    double delta_right = (absEncoderRight + rightTicks) - lastRightTicks;

    if (delta_left == 0.0 && delta_right == 0.0 || fabs(delta_left) > 6.0 && fabs(delta_right) > 6.0) {
        return;
    }

    lastLeftTicks = absEncoderLeft + leftTicks;
    lastRightTicks = absEncoderRight + rightTicks;

    ROS_INFO("Delta Left: %f, Delta Right: %f", delta_left, delta_right);

    double d_left = delta_left * (2 * M_PI * WHEEL_RADIUS_M) / 6.0;
    double d_right = delta_right * (2 * M_PI * WHEEL_RADIUS_M) / 6.0;

    double d = (d_left + d_right) / 2.0;
    double dtheta = (d_right - d_left) / TRACK_WIDTH_M;

    updateParticlesMotion(d, dtheta);
}

double getRotationTicks(double angle_rad) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double wheel_travel_distance = (TRACK_WIDTH_M * angle_rad) / 2;
    return k_rotation * (wheel_travel_distance / wheel_circumference) * ticks_per_revolution;
}

double getTranslationTicks(double distance) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    return k_translation * (distance / wheel_circumference) * ticks_per_revolution;
}


void rotate(double angle_rad, double speed) {
    ROS_INFO("Rotation: %f", angle_rad);

    double ticks = getRotationTicks(fabs(angle_rad));

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    diffDriveSrv.request.left = -side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);

    resetEncoders();
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while (ticks * 0.95 > (abs(leftTicks) + abs(rightTicks)) / 2) {
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


void translate(double distance, double speed) {
    ROS_INFO("Translation: %f", distance);

    double ticks = getTranslationTicks(distance);

    double side = (distance > 0) ? 1.0 : -1.0;
    diffDriveSrv.request.left = side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    resetEncoders();
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

void driveArc(double angle_rad, double arc_radius, double speed) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double b = TRACK_WIDTH_M;

    double abs_angle = std::abs(angle_rad);

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    double d_left  = (arc_radius - side * b / 2.0) * abs_angle;
    double d_right = (arc_radius + side * b / 2.0) * abs_angle;

    double ticks_left  = (d_left  / wheel_circumference) * ticks_per_revolution;
    double ticks_right = (d_right / wheel_circumference) * ticks_per_revolution;

    double v_left, v_right;
    if (angle_rad > 0) {
        v_left  = speed * (d_left / d_right);
        v_right = speed;
    } else if (angle_rad < 0) {
        v_left  = speed;
        v_right = speed * (d_right / d_left);
    } else {
        ROS_WARN("driveArc called with angle_rad = 0, nothing to do!");
        return;
    }

    resetEncoders();
    ros::spinOnce();

    diffDriveSrv.request.left  = v_left;
    diffDriveSrv.request.right = v_right;
    diffDriveClient->call(diffDriveSrv);

    ros::Rate rate(100);

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while ((fabs(leftTicks) < fabs(ticks_left)) || (fabs(rightTicks) < fabs(ticks_right))) {

        diffDriveSrv.request.left  = v_left;
        diffDriveSrv.request.right = v_right;
        diffDriveClient->call(diffDriveSrv);

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

bool isClearPath(const sensor_msgs::LaserScan::ConstPtr& msg, double obstacle_threshold) {
    float angle_min_limit = -M_PI / 32;
    float angle_max_limit =  M_PI / 32;

    int start_index = std::max(0, static_cast<int>((angle_min_limit - msg->angle_min) / msg->angle_increment));
    int end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, static_cast<int>((angle_max_limit - msg->angle_min) / msg->angle_increment));

    int invalid = 0;

    for(int i = start_index; i < end_index; i++){
        float value = msg->ranges[i];
        if(!isnan(value) && value >= msg->range_min && value <= msg->range_max){
            if(value < obstacle_threshold){
                invalid = invalid + 1;
            }
        }
    }

    if (invalid > (end_index - start_index) * 0.2) {
        return false;
    }

    return true;
}

void rotate_to(double target_theta, double speed) {
    double tolerance = 0.10;
    ros::Rate rate(100);
    int max_iter = 3;

    double angle_diff = target_theta - current_pose.theta;
    double prev_angle_diff = angle_diff;

    for (int i = 0; i < max_iter; ++i) {
        angle_diff = target_theta - current_pose.theta;
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;

        if (std::abs(angle_diff) < tolerance) break;

        rotate(angle_diff, speed);
        ros::spinOnce();
        rate.sleep();
    }
}

void translate_to(double target_x, double target_y, double speed) {
    double tolerance = 0.05;
    ros::Rate rate(100);
    int max_iter = 3;

    for (int i = 0; i < max_iter; ++i) {
        double dx = target_x - current_pose.x;
        double dy = target_y - current_pose.y;
        double distance = std::hypot(dx, dy);

        ROS_INFO("Target: (%.3f, %.3f) | Current: (%.3f, %.3f) | d=%.3f",
                 target_x, target_y, current_pose.x, current_pose.y, distance);
        if (distance < tolerance) break;

        double theta_to_target = std::atan2(dy, dx);
        rotate_to(theta_to_target, speed);

        translate(distance, speed);
        ros::spinOnce();
        rate.sleep();
    }
}

bool executePlan(plastic_fundamentals::ExecutePlan::Request &req, plastic_fundamentals::ExecutePlan::Response &res) {
    int grid_x = current_pose.row;
    int grid_y = current_pose.col;
    double theta = current_pose.theta;

    for(size_t idx = 0; idx < req.plan.size(); ++idx) {
        int dir = req.plan[idx];

        int target_grid_x = grid_x;
        int target_grid_y = grid_y;
        double target_theta = 0.0;

        if (dir == 0) {
            target_grid_y += 1;
            target_theta = M_PI_2;
        } else if (dir == 1) {
            target_grid_x -= 1;
            target_theta = M_PI;
        } else if (dir == 2) {
            target_grid_y -= 1;
            target_theta = -M_PI_2;
        } else if (dir == 3) {
            target_grid_x += 1;
            target_theta = 0.0;
        }

        double target_x = target_grid_x * CELL_SIZE + 0.4;
        double target_y = target_grid_y * CELL_SIZE + 0.4;


        rotate_to(target_theta, 5.0);

        translate_to(target_x, target_y, 7.0);

        grid_x = target_grid_x;
        grid_y = target_grid_y;
        theta = target_theta;
    }

    res.success = true;
    return true;
}

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    if (map_data) {
        return;
    }

    map_data = msg;
    ROS_INFO("Received map data");
    getMapLines(map_data);


    initializeParticles(map_data);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //if (processing_done) return;
    if (!map_data || particles.empty()) return;

    pathClear = false;
    if(isClearPath(msg, 0.8)) {
        pathClear = true;
    }

    updateParticlesSensor(msg);
    
    resampleParticles();
    
    current_pose = estimatePose();

    double n_eff = calculateEffectiveParticles();
    double effective_particles_threshold = 0.4 * NUM_PARTICLES;

    double mean_x = 0, mean_y = 0, mean_theta = 0;
    double max_weight = 0;
    double total_weight = 0.0;

    for (const auto& p : particles) {
        mean_x += p.x;
        mean_y += p.y;
        mean_theta += p.theta;
    }
    mean_x /= particles.size();
    mean_y /= particles.size();
    mean_theta /= particles.size();

    double var_x = 0, var_y = 0, var_theta = 0;
    for (const auto& p : particles) {
        var_x += pow(p.x - mean_x, 2);
        var_y += pow(p.y - mean_y, 2);
        var_theta += pow(p.theta - mean_theta, 2);

        max_weight = std::max(max_weight, p.weight);
        total_weight += p.weight;

    }
    double avg_weight = total_weight / particles.size();

    var_x /= particles.size();
    var_y /= particles.size();
    var_theta /= particles.size();

    double stddev_x = sqrt(var_x);
    double stddev_y = sqrt(var_y);
    double stddev_theta = sqrt(var_theta);

    if (stddev_x < 0.05 && stddev_y < 0.05 && stddev_theta < 0.05 && n_eff > effective_particles_threshold) {
        is_localized = true;

        plastic_fundamentals::Pose current_pose_msg;
        current_pose_msg.row = current_pose.row;
        current_pose_msg.column = current_pose.col;
        current_pose_msg.orientation = angleToDirection(current_pose.theta);

        pose_pub.publish(current_pose_msg);

        ROS_INFO("Localized at row=%d, column=%d, orientation=%d, angle=%f (confidence=%.3f)",
                           current_pose.row, current_pose.col, current_pose_msg.orientation, current_pose.theta, max_weight);


        if (first_localization) {
            first_localization = false;
            create_fundamentals::PlaySong play_srv;
            play_srv.request.number = 1;
            if (playSong->call(play_srv)) {
                ROS_INFO("Guile theme playing!");
            } else {
                ROS_ERROR("Failed to play song.");
            }
        }
    }

    if (is_localized && max_weight < 0.003 && avg_weight < 0.001 && n_eff < 0.2 * NUM_PARTICLES) {
        ROS_WARN("Robot is lost! Re-initializing particles.");
        initializeParticles(map_data);
        is_localized = false;
    }
}

void localizationRoutine(ros::Rate rate) {
    LocalizationPhase localization_phase = SPIN;

    double move_distance = 0.8;

    while (ros::ok() && !processing_done) {
        if (is_localized) {
            processing_done = true;
            ROS_INFO("Localization complete, publishing pose and waiting");
        } else {
            switch (localization_phase) {
                case SPIN:
                    ROS_INFO("Performing rotation to gather more information...");
                    rotate(M_PI / 2, 5.0);
                    localization_phase = MOVE;
                    break;

                case MOVE:
                    ros::spinOnce();
                    if (pathClear) {
                        ROS_INFO("Moving robot to cover more area...");
                        translate(move_distance, 7.0);
                        localization_phase = SPIN;
                    } else {
                        ROS_INFO("Obstacle detected, changing direction...");
                        rotate(M_PI / 2, 5.0);
                    }
                    break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void initSong() {

    int MEASURE = 160;
    int Q = MEASURE/4;
    int HALF = MEASURE/2;

    std::vector<unsigned int> guile_song = {
        69, 9,
        74, 9,
        77, 9,
        38, 70,
        57, 33,
        36, 33,
        69, 9,
        74, 9,
        77, 9,
        67, 9,
        72, 9,
        76, 9,
        36, 33,
        67, 9,
        72, 9,
        76, 9,
        69, 69,
        74, 69,
        77, 73,
        43, 9,
        45, 9,
        43, 9,
        41, 9,
        38, 33,
        69, 9,
        74, 9,
        40, 9,
        67, 9,
        72, 9,
        76, 9,
        38, 9,
        36, 9,
    };

    create_fundamentals::StoreSong store_srv;
    store_srv.request.number = 1;
    store_srv.request.song = guile_song;
    if (storeSong->call(store_srv)) {
        ROS_INFO("Guile theme uploaded!");
    } else {
        ROS_ERROR("Failed to upload song.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcl_localization");
    ros::NodeHandle nh;

    ros::service::waitForService("store_song");
    ros::service::waitForService("play_song");

    ros::ServiceClient store_song = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    storeSong = &store_song;
    ros::ServiceClient play_song = nh.serviceClient<create_fundamentals::PlaySong>("play_song");
    playSong = &play_song;

    initSong();

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::Subscriber sub = nh.subscribe("sensor_packet", 1, sensorCallback);

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, scanCallback);
    
    pose_pub = nh.advertise<plastic_fundamentals::Pose>("/pose", 10);
    particle_pub = nh.advertise<visualization_msgs::MarkerArray>("/particles", 10);
    
    ros::ServiceClient diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diffDriveClient = &diffDrive;

    ros::ServiceClient resetEncoders = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlan);

    ROS_INFO("MCL localization node started");


    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(2).sleep();

        ros::spinOnce();
    }

    ros::Rate rate(100);

    localizationRoutine(rate);

    ROS_INFO("Localization routine completed, waiting for a plan to execute...");

    while (ros::ok() && !isFlying) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
