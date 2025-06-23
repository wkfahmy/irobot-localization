#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include "create_fundamentals/SensorPacket.h"
#include <plastic_fundamentals/PublishMarker.h>
#include <create_fundamentals/PlaySong.h>
#include <create_fundamentals/StoreSong.h>
#include <plastic_fundamentals/Move.h>
#include <plastic_fundamentals/Align.h>
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
#include <plastic_fundamentals/AbsEncoder.h>
#include <queue>

constexpr double WHEEL_RADIUS_M = 0.0325;
constexpr double TRACK_WIDTH_M = 0.263;
constexpr double LIDAR_OFFSET_M = 0.16;
constexpr double CELL_SIZE = 0.8;

int num_rows = 0;
int num_cols = 0;

int min_particles = 500;
int max_particles = 3000;
int current_num_particles = 1000;

double uncertainty = std::numeric_limits<double>::infinity();

constexpr double XY_INIT_NOISE = 0.1;
constexpr double THETA_INIT_NOISE = 0.4;

constexpr double XY_MOTION_NOISE = 0.05;
constexpr double THETA_MOTION_NOISE = 0.2;

constexpr double XY_RESAMPLE_NOISE = 0.05;
constexpr double THETA_RESAMPLE_NOISE = 0.2;

constexpr double RESAMPLE_THRESHOLD = 0.1;

double leftTicks = 0;
double rightTicks = 0;

double lastLeftTicks = 0;
double lastRightTicks = 0;

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
    int row;
    int col;
};

std::vector<std::vector<Particle>> clusters;

ros::ServiceClient storeSong;
ros::ServiceClient playSong;

ros::ServiceClient rotateClient;
ros::ServiceClient translateClient;
ros::ServiceClient alignClient;

create_fundamentals::DiffDrive diffDriveSrv;
ros::ServiceClient* diffDriveClient;
plastic_fundamentals::Grid::ConstPtr map_data;
Particle current_pose;

ros::ServiceClient marker;

int lost_counter = 0;
bool is_localized = false;
bool first_localization = true;

bool processing_done = false;

bool pathClear = false;
int openDirection = 1;

ros::Publisher particle_pub;
ros::Publisher pose_pub;
std::default_random_engine random_generator;

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
    particles.reserve(current_num_particles);

    std::uniform_real_distribution<double> xy_noise_dist(-XY_INIT_NOISE, XY_INIT_NOISE);
    std::uniform_real_distribution<double> theta_noise_dist(-THETA_INIT_NOISE, THETA_INIT_NOISE);

    num_rows = map->rows.size();
    num_cols = map->rows[0].cells.size();
    int total_cells = num_rows * num_cols;

    int particles_per_cell = current_num_particles / (total_cells * 4);
    if (particles_per_cell < 1) particles_per_cell = 1;

    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_cols; ++col) {
            double cell_center_x = (row + 0.5) * CELL_SIZE;
            double cell_center_y = (col + 0.5) * CELL_SIZE;

            for (int orientation = 0; orientation < 4; ++orientation) {
                double base_theta = (orientation - 1) * M_PI / 2.0;

                for (int i = 0; i < particles_per_cell; ++i) {
                    if (particles.size() >= current_num_particles) break;

                    Particle p;
                    p.x = cell_center_x + xy_noise_dist(random_generator);
                    p.y = cell_center_y + xy_noise_dist(random_generator);
                    p.theta = base_theta + theta_noise_dist(random_generator);
                    p.weight = 1.0 / current_num_particles;
                    p.row = row;
                    p.col = col;

                    particles.push_back(p);
                }
            }
        }
    }

    while (particles.size() < current_num_particles) {
        int idx = rand() % particles.size();
        particles.push_back(particles[idx]);
    }

    ROS_INFO("Initialized %zu particles", particles.size());

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointsMarker";

    for (auto& p : particles) {
        plastic_fundamentals::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.w = p.theta;
        srv.request.points.push_back(pt);
    }

    if (!marker.call(srv)) {
        //ROS_ERROR("Failed to call service marker_service");
    }

}

void updateParticlesMotion(double distance, double angle) {
    double dist_stddev = std::abs(distance) * XY_MOTION_NOISE;
    double angle_stddev = std::abs(angle) * THETA_MOTION_NOISE;

    std::normal_distribution<double> translation_noise(0, dist_stddev);
    std::normal_distribution<double> rotation_noise(0, angle_stddev);

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointsMarker";

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
        //ROS_ERROR("Failed to call service marker_service");
    }
}

double normalizeWeights() {
    double sum_squared_weights = 0.0;
    double sum = 0.0;
    for (const auto& p : particles) {
        sum += p.weight;
        sum_squared_weights += p.weight * p.weight;
    }

    if (sum > 0) {
        for (auto& p : particles) {
            p.weight /= sum;
        }
        return 1.0 / sum_squared_weights * sum * sum;
    } else {
        // If all weights are zero, reset to uniform
        double uniform_weight = 1.0 / particles.size();
        for (auto& p : particles) {
            p.weight = uniform_weight;
        }
        return 0.0;

    }
}

void resampleParticles(double n_eff) {
    if (n_eff > RESAMPLE_THRESHOLD * particles.size() && particles.size() == current_num_particles) {
        return;
    }

    std::shuffle(particles.begin(), particles.end(), random_generator);
    std::vector<double> cumulative(particles.size());
    cumulative[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + particles[i].weight;
    }

    std::uniform_real_distribution<double> dist_r(0.0, 1.0 / particles.size());
    std::normal_distribution<double> noise_xy(0.0, XY_RESAMPLE_NOISE);
    std::normal_distribution<double> noise_theta(0.0, THETA_RESAMPLE_NOISE);
    std::uniform_real_distribution<double> rand_x(0, num_cols * 0.8);
    std::uniform_real_distribution<double> rand_y(0, num_rows * 0.8);
    std::uniform_real_distribution<double> rand_theta(-M_PI, M_PI);

    const double RANDOM_PARTICLE_RATIO = 0.3;
    int num_random = static_cast<int>(RANDOM_PARTICLE_RATIO * current_num_particles);
    int num_resampled = current_num_particles - num_random;

    std::vector<Particle> new_particles;
    new_particles.reserve(current_num_particles);

    double r = dist_r(random_generator);
    double c = cumulative[0];
    size_t i = 0;

    for (int m = 0; m < num_resampled; ++m) {
        double U = r + m * (1.0 / particles.size());
        while (U > c && i < particles.size() - 1) {
            i++;
            c = cumulative[i];
        }

        Particle p = particles[i];
        p.x += noise_xy(random_generator);
        p.y += noise_xy(random_generator);
        p.theta += noise_theta(random_generator);
        while (p.theta > M_PI) p.theta -= 2 * M_PI;
        while (p.theta <= -M_PI) p.theta += 2 * M_PI;
        p.weight = 1.0 / current_num_particles;
        new_particles.push_back(p);
    }

    for (int k = 0; k < num_random; ++k) {
        Particle p;
        p.x = rand_x(random_generator);
        p.y = rand_y(random_generator);
        p.theta = rand_theta(random_generator);
        p.weight = 1.0 / current_num_particles;
        new_particles.push_back(p);
    }

    particles.resize(current_num_particles);
    particles = std::move(new_particles);

    plastic_fundamentals::PublishMarker srv;
    srv.request.marker_type = "PointsMarker";

    for (auto& p : particles) {
        plastic_fundamentals::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.w = p.theta;
        srv.request.points.push_back(pt);
    }

    if (!marker.call(srv)) {
        //ROS_ERROR("Failed to call service marker_service");
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

double updateParticlesSensor(const sensor_msgs::LaserScan::ConstPtr& scan) {
    constexpr int step = 10;
    constexpr double sigma = 0.1;
    constexpr double sigma2_inv = 1.0 / (sigma * sigma);
    const int num_beams = (scan->ranges.size() - 4 * step) / step;

    std::vector<double> angles(num_beams);
    for (int j = 0; j < num_beams; ++j) {
        int i = 20 + j * step;
        angles[j] = scan->angle_min + i * scan->angle_increment;
    }

    std::vector<float> valid_ranges(num_beams);
    std::vector<bool> is_valid(num_beams, false);
    for (int j = 0; j < num_beams; ++j) {
        int i = 20 + j * step;
        float measured = scan->ranges[i];
        if (!std::isnan(measured) && measured >= scan->range_min && measured <= scan->range_max) {
            valid_ranges[j] = measured;
            is_valid[j] = true;
        }
    }

    for (auto& p : particles) {
        std::vector<double> expected = calculateExpectedScan(p, angles);

        double log_likelihood = 0.0;
        int valid_rays = 0;

        for (int j = 0; j < num_beams; ++j) {
            double expected_val = expected[j];

            if (is_valid[j]) {
                double diff = valid_ranges[j] - expected_val;
                log_likelihood += -0.5 * diff * diff * sigma2_inv;
                ++valid_rays;
            } else {

                if (expected_val < scan->range_max - 0.03) {
                    log_likelihood += -1.0;
                } else {
                    log_likelihood += 1.0;
                }
                ++valid_rays;
            }
        }

        p.weight *= (valid_rays > 0) ? std::exp(log_likelihood / valid_rays) : 0.5;
    }

    return normalizeWeights();
}

Particle estimatePose(int top_k = 30) {
    Particle pose;

    if (particles.empty()) return pose;

    std::vector<Particle> sorted_particles = particles;

    std::partial_sort(sorted_particles.begin(),
                      sorted_particles.begin() + std::min(top_k, (int)sorted_particles.size()),
                      sorted_particles.end(),
                      [](const Particle& a, const Particle& b) {
                          return a.weight > b.weight;
                      });

    double sum_x = 0.0, sum_y = 0.0;
    double sum_sin_theta = 0.0, sum_cos_theta = 0.0;
    double sum_weights = 0.0;

    int k = std::min(top_k, (int)sorted_particles.size());

    for (int i = 0; i < k; ++i) {
        const Particle& p = sorted_particles[i];
        sum_x += p.weight * p.x;
        sum_y += p.weight * p.y;
        sum_sin_theta += p.weight * sin(p.theta);
        sum_cos_theta += p.weight * cos(p.theta);
        sum_weights += p.weight;
    }

    if (sum_weights > 0.0) {
        pose.x = sum_x / sum_weights;
        pose.y = sum_y / sum_weights;
        pose.theta = atan2(sum_sin_theta / sum_weights, sum_cos_theta / sum_weights);
        if (pose.theta < 0) pose.theta += 2 * M_PI;

        pose.row = static_cast<int>(pose.x / CELL_SIZE);
        pose.col = static_cast<int>(pose.y / CELL_SIZE);
    }

    return pose;
}


Particle estimatePoseFromCluster(const std::vector<Particle>& particles, double& max_weight, double cluster_radius = 0.3) {
    int best_cluster_size = 0;
    double best_x = 0.0, best_y = 0.0, best_theta = 0.0;
    max_weight = 0.0;

    for (const auto& center : particles) {
        double cluster_x = 0.0, cluster_y = 0.0;
        double sin_sum = 0.0, cos_sum = 0.0;
        double total_weight = 0.0;
        int count = 0;

        for (const auto& p : particles) {
            double dx = p.x - center.x;
            double dy = p.y - center.y;
            if (dx * dx + dy * dy < cluster_radius * cluster_radius) {
                cluster_x += p.x * p.weight;
                cluster_y += p.y * p.weight;
                sin_sum += std::sin(p.theta) * p.weight;
                cos_sum += std::cos(p.theta) * p.weight;
                total_weight += p.weight;
                ++count;
                if (p.weight > max_weight) max_weight = p.weight;
            }
        }

        if (count > best_cluster_size) {
            best_cluster_size = count;
            best_x = cluster_x / total_weight;
            best_y = cluster_y / total_weight;
            best_theta = std::atan2(sin_sum, cos_sum);
        }
    }

    Particle result;
    result.x = best_x;
    result.y = best_y;
    result.theta = best_theta;
    result.row = std::round(best_y);
    result.col = std::round(best_x);
    return result;
}

double lastAbsLeft = 0;
double lastAbsRight = 0;

void absEncoderCallback(const plastic_fundamentals::AbsEncoder::ConstPtr& msg) {
    double delta_left = msg->abs_left - lastAbsLeft;
    double delta_right = msg->abs_right - lastAbsRight;

    lastAbsLeft = msg->abs_left;
    lastAbsRight = msg->abs_right;

    double d_left = delta_left * WHEEL_RADIUS_M;
    double d_right = delta_right * WHEEL_RADIUS_M;

    double d = (d_left + d_right) / 2.0;
    double dtheta = (d_right - d_left) / TRACK_WIDTH_M;

    updateParticlesMotion(d, dtheta);
}

bool rotate(double angle_rad, double speed, bool correction = false) {
    plastic_fundamentals::Move srv;
    srv.request.angle = angle_rad;
    srv.request.speed = speed;
    srv.request.correction = correction;
    bool success = rotateClient.call(srv);
    lastAbsLeft = 0;
    lastAbsRight = 0;
    return success;
}

bool translate(double distance, double speed, bool correction = false) {
    plastic_fundamentals::Move srv;
    srv.request.distance = distance;
    srv.request.speed = speed;
    srv.request.correction = correction;
    bool success = translateClient.call(srv);
    lastAbsLeft = 0;
    lastAbsRight = 0;
    return success;
}

bool isClearPath(const sensor_msgs::LaserScan::ConstPtr& msg, double obstacle_threshold) {
    double angle_range = M_PI / 24.0;

    int start_index = std::max(0, static_cast<int>((-angle_range - msg->angle_min) / msg->angle_increment));
    int end_index = std::min(static_cast<int>((angle_range - msg->angle_min) / msg->angle_increment), static_cast<int>(msg->ranges.size()) - 1);

    int blocked_rays = 0;
    int valid_rays = 0;

    for (int i = start_index; i <= end_index; ++i) {
        float r = msg->ranges[i];
        valid_rays++;
        if (!std::isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            if (r < obstacle_threshold) {
                blocked_rays++;
            }
        }
    }

    return static_cast<double>(blocked_rays) / valid_rays < 0.4;
}

int getMostOpenDirection(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int size = msg->ranges.size();
    int third = size / 3;

    double left_sum = 0, right_sum = 0;
    int left_count = 0, right_count = 0;

    for (int i = 2 * third; i < size; i += 5) {
        float r = msg->ranges[i];
        if (!std::isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            left_sum += r;
            left_count++;
        }
    }

    for (int i = 0; i < third; i += 5) {
        float r = msg->ranges[i];
        if (!std::isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            right_sum += r;
            right_count++;
        }
    }

    double left_avg = (left_count > 0) ? left_sum / left_count : 0;
    double right_avg = (right_count > 0) ? right_sum / right_count : 0;

    if (left_avg > right_avg) {
        return 1;
    } else {
        return -1;
    }
}

void rotate_to(double target_theta, double speed) {
    double tolerance = 0.5;
    ros::Rate rate(100);
    int max_iter = 1;

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
    double tolerance = 0.02;
    ros::Rate rate(100);
    int max_iter = 1;
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

        if (dir == 0) {
            target_grid_y += 1;
        } else if (dir == 1) {
            target_grid_x -= 1;
        } else if (dir == 2) {
            target_grid_y -= 1;
        } else if (dir == 3) {
            target_grid_x += 1;
        }

        double target_x = target_grid_x * CELL_SIZE + 0.4;
        double target_y = target_grid_y * CELL_SIZE + 0.4;

        translate_to(target_x, target_y, 10.0);

        grid_x = target_grid_x;
        grid_y = target_grid_y;
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

void adaptParticleCount(double uncertainty) {
    int previous = current_num_particles;

    if (uncertainty > 1.5 && lost_counter > 5) {
        current_num_particles = std::min(current_num_particles * 2, max_particles);
    }
    else if (uncertainty < 0.4 && current_num_particles > min_particles) {
        current_num_particles = std::max(current_num_particles / 2, min_particles);
    }

    if (current_num_particles != previous) {
        ROS_INFO("Adjusted particle count to %d", current_num_particles);
    }
}

void calculateUncertainty(double& spatial_uncertainty, double& dominant_ratio) {
    double mean_x = 0.0, mean_y = 0.0;
    double total_weight = 0.0;

    for (const auto& p : particles) {
        mean_x += p.x * p.weight;
        mean_y += p.y * p.weight;
        total_weight += p.weight;
    }
    if (total_weight == 0.0) return;

    mean_x /= total_weight;
    mean_y /= total_weight;

    double var_x = 0.0, var_y = 0.0;
    for (const auto& p : particles) {
        var_x += p.weight * (p.x - mean_x) * (p.x - mean_x);
        var_y += p.weight * (p.y - mean_y) * (p.y - mean_y);
    }
    var_x /= total_weight;
    var_y /= total_weight;

    spatial_uncertainty = std::sqrt(var_x + var_y);

    double weightmap[6][6] = {{0.0}};
    for (const auto& p : particles) {
        int i = p.row;
        int j = p.col;
        if (i >= 0 && i < 6 && j >= 0 && j < 6) {
            weightmap[i][j] += p.weight;
        }
    }

    double max_cell_weight = 0.0;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            max_cell_weight = std::max(max_cell_weight, weightmap[i][j]);

    dominant_ratio = max_cell_weight / total_weight;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (!map_data || particles.empty()) return;

    pathClear = isClearPath(msg, 0.8);
    openDirection = getMostOpenDirection(msg);

    double n_eff = updateParticlesSensor(msg);
    double effective_particles_threshold = 0.4 * current_num_particles;

    double spatial_uncertainty = 0.0;
    double dominant_ratio = 0.0;
    calculateUncertainty(spatial_uncertainty, dominant_ratio);
    adaptParticleCount(spatial_uncertainty);

    resampleParticles(n_eff);

    current_pose = estimatePose();

    if ((spatial_uncertainty < 0.65 && dominant_ratio > 0.45) || (spatial_uncertainty < 0.8 && dominant_ratio > 0.8)) {
        is_localized = true;
        lost_counter = 0;

        plastic_fundamentals::Pose current_pose_msg;
        current_pose_msg.row = current_pose.row;
        current_pose_msg.column = current_pose.col;
        current_pose_msg.orientation = angleToDirection(current_pose.theta);

        pose_pub.publish(current_pose_msg);


        plastic_fundamentals::PublishMarker srv;

        srv.request.marker_type = "PointMarker";

        plastic_fundamentals::Point pt;
        pt.x = current_pose.x;
        pt.y = current_pose.y;
        srv.request.point = pt;

        marker.call(srv);

        ROS_INFO("Localized at row=%d, column=%d, orientation=%d (confidence=%.3f ratio=%.3f)",
                 current_pose.row, current_pose.col, current_pose_msg.orientation, spatial_uncertainty, dominant_ratio);

        if (first_localization) {
            first_localization = false;
            create_fundamentals::PlaySong play_srv;
            play_srv.request.number = 1;
            if (playSong.call(play_srv)) {
                ROS_INFO("Guile theme playing!");
            } else {
                ROS_ERROR("Failed to play song.");
            }
        }
    } else if (spatial_uncertainty > 1.2 || dominant_ratio < 0.2) {
        lost_counter++;
        if (lost_counter > 10) {
            ROS_WARN("Localization lost, resetting particles");
            lost_counter = 0;
            is_localized = false;
        }
        ROS_WARN("Localization not achieved, row=%d, column=%d, orientation=%d (confidence=%.3f ratio=%.3f)",
                         current_pose.row, current_pose.col, angleToDirection(current_pose.theta), spatial_uncertainty, dominant_ratio);
    }
}

void localizationRoutine(ros::Rate rate) {
    LocalizationPhase localization_phase = SPIN;
    double move_distance = 0.8;

    while (ros::ok() && !is_localized) {
        if (pathClear) {
            localization_phase = MOVE;
        } else {
            localization_phase = SPIN;
        }

        switch (localization_phase) {
            case SPIN:
                ROS_INFO("Rotation robot to cover more area...");
                rotate(openDirection * M_PI / 2, 7.0, true);
                break;
            case MOVE:
                ROS_INFO("Moving robot to cover more area...");
                translate(move_distance, 10.0, true);
                break;
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
    if (storeSong.call(store_srv)) {
        ROS_INFO("Guile theme uploaded!");
    } else {
        ROS_ERROR("Failed to upload song.");
    }
}

std::mutex align_mutex;

void alignRobot() {
    std::lock_guard<std::mutex> lock(align_mutex);
    plastic_fundamentals::Align srv;
    if (alignClient.call(srv)) {
        ROS_INFO("Robot successfully aligned");
    } else {
        ROS_ERROR("Failed to align robot");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mcl_localization");
    ros::NodeHandle nh;

    //ros::service::waitForService("store_song");
    //ros::service::waitForService("play_song");
    ros::service::waitForService("/align");

    storeSong = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    playSong = nh.serviceClient<create_fundamentals::PlaySong>("play_song");

    initSong();

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, scanCallback);

    ros::Subscriber abs_sub = nh.subscribe("/absolute_encoders", 1, absEncoderCallback);

    pose_pub = nh.advertise<plastic_fundamentals::Pose>("/pose", 10);
    particle_pub = nh.advertise<visualization_msgs::MarkerArray>("/particles", 10);

    rotateClient = nh.serviceClient<plastic_fundamentals::Move>("perform_rotation");
    translateClient = nh.serviceClient<plastic_fundamentals::Move>("perform_translation");

    alignClient = nh.serviceClient<plastic_fundamentals::Align>("/align");

    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlan);

    ROS_INFO("MCL localization node started");

    ros::Duration(2).sleep();
    if (!alignClient.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("align service not available");
    } else {
        alignRobot();
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(0.5).sleep();

        ros::spinOnce();
    }

    ros::Rate rate(100);

    while (ros::ok()) {
        localizationRoutine(rate);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
