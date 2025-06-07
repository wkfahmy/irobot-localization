#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include "create_fundamentals/ResetEncoders.h"
#include <plastic_fundamentals/Grid.h>
#include <plastic_fundamentals/Row.h>
#include <plastic_fundamentals/Cell.h>
#include <plastic_fundamentals/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <create_fundamentals/SensorPacket.h>


double x_bar = 0.0;
double y_bar = 0.0;
double theta_bar = 0.0;

double x_bar_prime = 0.0;
double y_bar_prime = 0.0;
double theta_bar_prime = 0.0;

float min_distance;
int min_index;
int start_index;
int end_index;

double leftTicks = 0;
double rightTicks = 0;
double radius = 0.0325;
double ticksPerRevolution = 6.25;      // might need adjustment
double ticksPerRevolutionRot = 5.5;    // might need adjustment

constexpr double WHEEL_RADIUS_M = 0.0325;
constexpr double TRACK_WIDTH_M = 0.263;
constexpr double LIDAR_OFFSET_M = 0.16;
constexpr double CELL_SIZE = 0.8;

constexpr int NUM_PARTICLES = 1000;
constexpr double INIT_TRANSLATION_NOISE = 0.1;
constexpr double INIT_THETA_NOISE = 0.2;
constexpr double MOTION_TRANSLATION_NOISE = 0.02;
constexpr double MOTION_THETA_NOISE = 0.05;
constexpr double RESAMPLE_THRESHOLD = 0.5;

ros::ServiceClient* diff_drive_client;

ros::ServiceClient* reset_encoders_client;

plastic_fundamentals::Grid::ConstPtr map_data;
plastic_fundamentals::Pose current_pose;
bool is_localized = false;
bool first_localization = true;

bool processing_done = false;

ros::Publisher particle_pub;
ros::Publisher pose_pub;

ros::Publisher marker_pub;


std::default_random_engine random_generator;

std::vector<double> actual_scan;

// Particle structure
struct Particle {
    double x;
    double y;
    double theta;
    double weight;
    int row;            
    int col;           
};

// Global particle set
std::vector<Particle> particles;

double getTranslationTicks(double distance) {
    return ticksPerRevolution * distance / (2 * M_PI * radius);
}

double getRotationTicks(double angle_rad) {
    return ticksPerRevolutionRot * angle_rad * TRACK_WIDTH_M / (4 * M_PI * radius);
}

double getTranslationFromTicks(double leftTicks, double rightTicks) {
    double avgTicks = (leftTicks + rightTicks) / 2.0;
    return (2 * M_PI * WHEEL_RADIUS_M) * (avgTicks / 6.25);  
}

double getRotationFromTicks(double leftTicks, double rightTicks) {
    double tick_diff = rightTicks - leftTicks;
    return (2 * M_PI * WHEEL_RADIUS_M) * (tick_diff / 5.5) / TRACK_WIDTH_M;  
}

void resetEncoders() {
    ROS_INFO("Resetting the encoders");
    create_fundamentals::ResetEncoders srv;
    if(reset_encoders_client->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
}

void initializeParticles(const plastic_fundamentals::Grid::ConstPtr& map) {
    if (!map) {
        ROS_ERROR("Cannot initialize particles: No map data");
        return;
    }

    particles.clear();
    particles.reserve(NUM_PARTICLES);

    //map boundaries
    double max_x = 2.4;
    double max_y = 2.4;

    // Random distributions
    std::uniform_real_distribution<double> x_dist(0.0, max_x);
    std::uniform_real_distribution<double> y_dist(0.0, max_y);
    std::uniform_real_distribution<double> theta_dist(0.0, 2*M_PI);  // Full 360° range
    std::normal_distribution<double> noise_dist(0.0, 0.05);  // Small position noise

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle p;
        
        // Random position in maze (with small noise for variation)
        do {
            p.x = x_dist(random_generator) + noise_dist(random_generator);
            p.y = y_dist(random_generator) + noise_dist(random_generator);
        } while (p.x < 0 || p.x >= max_x || p.y < 0 || p.y >= max_y);  // Ensure within bounds

        // Random orientation
        p.theta = theta_dist(random_generator);
        
        // Calculate grid cell indices
        p.col = static_cast<int>(p.x / CELL_SIZE);
        p.row = static_cast<int>(p.y / CELL_SIZE);
        
        p.weight = 1.0 / NUM_PARTICLES;
        particles.push_back(p);
    }

    ROS_INFO("Initialized %zu particles randomly in maze", particles.size());
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
    // Check if resampling is needed
    double n_eff = calculateEffectiveParticles();
    if (n_eff > RESAMPLE_THRESHOLD * particles.size()) {
        ROS_INFO("Skipping resampling, effective particles: %.2f", n_eff);
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
    
    for (size_t m = 0; m < particles.size(); ++m) {
        double u = r + m * (1.0 / particles.size());
        while (u > cumulative_sum[i] && i < particles.size() - 1) {
            i++;
        }
        new_particles.push_back(particles[i]);
    }
    
    particles = new_particles;
    
    double uniform_weight = 1.0 / particles.size();
    for (auto& p : particles) {
        p.weight = uniform_weight;
    }
}

void updateParticlesMotion(double x_bar, double y_bar, double theta_bar, double x_bar_prime, double y_bar_prime)
{ 
    double delta_rot1 = theta_bar - atan2(y_bar_prime - y_bar, x_bar_prime - x_bar); //reversed so it matches our axis
    double delta_trans = std::sqrt(std::pow(x_bar_prime - x_bar, 2) + std::pow(y_bar_prime - y_bar, 2));

    // Normalize angles
    while (delta_rot1 > M_PI) delta_rot1 -= 2 * M_PI;
    while (delta_rot1 <= -M_PI) delta_rot1 += 2 * M_PI;

    std::vector<Particle> updated_particles;

    for (auto& p : particles) {
        // Noise variances
        double var_rot1 = INIT_TRANSLATION_NOISE * delta_rot1 * delta_rot1 + MOTION_TRANSLATION_NOISE * delta_trans * delta_trans;
        double var_trans = MOTION_THETA_NOISE * delta_trans * delta_trans + MOTION_THETA_NOISE * (delta_rot1 * delta_rot1);
        // Sample noise
        std::normal_distribution<double> rot1_noise(0, std::sqrt(var_rot1));
        std::normal_distribution<double> trans_noise(0, std::sqrt(var_trans));

        double delta_rot1_hat = delta_rot1 + rot1_noise(random_generator);
        double delta_trans_hat = delta_trans + trans_noise(random_generator);

        // Update position
        double new_x = p.x + delta_trans_hat * cos(p.theta + delta_rot1_hat); //changed plus to minus
        double new_y = p.y + delta_trans_hat * sin(p.theta + delta_rot1_hat); //changed plus to minus
        double new_theta = p.theta + delta_rot1_hat;

        // Normalize angle
        while (new_theta > M_PI) new_theta -= 2 * M_PI;
        while (new_theta <= -M_PI) new_theta += 2 * M_PI;

        // Boundary check
        if (new_x < 0 || new_x >= 2.4 || new_y < 0 || new_y >= 2.4) continue;

        // Grid index
        int col = static_cast<int>(new_x / CELL_SIZE);
        int row = static_cast<int>(new_y / CELL_SIZE);

        if (row >= 0 && row < 3 && col >= 0 && col < 3) {
            // Valid particle, keep it
            p.x = new_x;
            p.y = new_y;
            p.theta = new_theta;
            p.row = row;
            p.col = col;
            updated_particles.push_back(p);
        }
    }

    // Replace with filtered valid particles
    particles = std::move(updated_particles);
}

double intersectRaySegment(double ox, double oy, double dx, double dy, double x1, double y1, double x2, double y2)
{
    double den = (x1 - x2) * dy - (y1 - y2) * dx;
    if (std::abs(den) < 1e-6) return std::numeric_limits<double>::infinity();

    double t = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
    double u = -((x1 - x2) * (y1 - oy) - (y1 - y2) * (x1 - ox)) / den;

    if (t >= 0 && t <= 1 && u >= 0) {
        return u;
    }
    return std::numeric_limits<double>::infinity();
}

void getWallSegment(int row, int col, int wallSide, double &x1, double &y1, double &x2, double &y2) {
    switch (wallSide) {
        case plastic_fundamentals::Cell::RIGHT:
            x1 = (col + 1) * CELL_SIZE; y1 = row * CELL_SIZE;
            x2 = x1; y2 = (row + 1) * CELL_SIZE;
            break;
        case plastic_fundamentals::Cell::LEFT:
            x1 = col * CELL_SIZE; y1 = row * CELL_SIZE;
            x2 = x1; y2 = (row + 1) * CELL_SIZE;
            break;
        case plastic_fundamentals::Cell::TOP:
            x1 = col * CELL_SIZE; y1 = row * CELL_SIZE;
            x2 = (col + 1) * CELL_SIZE; y2 = y1;
            break;
        case plastic_fundamentals::Cell::BOTTOM:
            x1 = col * CELL_SIZE; y1 = (row + 1) * CELL_SIZE;
            x2 = (col + 1) * CELL_SIZE; y2 = y1;
            break;
    }
}

double getRayDistance(const plastic_fundamentals::Grid& grid, double px, double py, double ray_angle_rad) {
    int row = int(py / CELL_SIZE);
    int col = int(px / CELL_SIZE);

    if (row < 0 || row >= (int)grid.rows.size() || col < 0 || col >= (int)grid.rows[0].cells.size())
        return std::numeric_limits<double>::infinity();

    double dx = cos(ray_angle_rad);
    double dy = sin(ray_angle_rad);

    auto checkCellWalls = [&](int r, int c) -> double {
        if (r < 0 || r >= (int)grid.rows.size() || c < 0 || c >= (int)grid.rows[0].cells.size())
            return std::numeric_limits<double>::infinity();

        const auto& cell = grid.rows[r].cells[c];
        double min_dist = std::numeric_limits<double>::infinity();

        for (int wall : cell.walls) {
            double x1, y1, x2, y2;
            getWallSegment(r, c, wall, x1, y1, x2, y2);
            double dist = intersectRaySegment(px, py, dx, dy, x1, y1, x2, y2);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    };

    double dist_current = checkCellWalls(row, col);

    int stepX = (dx > 0) ? 1 : -1;
    int stepY = (dy > 0) ? 1 : -1;

    double cellBoundaryX = (stepX > 0) ? (col + 1) * CELL_SIZE : col * CELL_SIZE;
    double cellBoundaryY = (stepY > 0) ? (row + 1) * CELL_SIZE : row * CELL_SIZE;

    double tMaxX = (dx != 0) ? (cellBoundaryX - px) / dx : std::numeric_limits<double>::infinity();
    double tMaxY = (dy != 0) ? (cellBoundaryY - py) / dy : std::numeric_limits<double>::infinity();

    int next_row = row;
    int next_col = col;

    if (tMaxX < tMaxY) {
        next_col += stepX;
    } else {
        next_row += stepY;
    }

    double dist_next = checkCellWalls(next_row, next_col);

    const double max_range = 1.0;

    double dist = std::min(dist_current, dist_next);
    return (dist <= max_range) ? dist : std::numeric_limits<double>::infinity();
}

std::vector<double> calculateExpectedScan(const Particle& p, const std::vector<double>& angles) {
    std::vector<double> expected_distances;
    expected_distances.reserve(angles.size());

    double lidar_x = p.x + LIDAR_OFFSET_M * cos(p.theta);
    double lidar_y = p.y + LIDAR_OFFSET_M * sin(p.theta);

    for (double angle : angles) {
        double ray_angle = p.theta + angle;
        double dist = getRayDistance(*map_data, lidar_x, lidar_y, ray_angle);
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
        double log_likelihood = 0.0;  // Using log probabilities for numerical stability
        int valid_rays = 0;
        
        for (size_t i = 0, j = 0; i < scan->ranges.size() && j < angles.size(); i += 10, ++j) {
            float measured = scan->ranges[i];
            double expected = expected_scan[j];
            
            if (!std::isfinite(measured) || measured < scan->range_min || measured > scan->range_max) 
                continue;
            
            if (!std::isfinite(expected) || expected > scan->range_max)
                continue;
            
            // More discriminative measurement model
            double sigma = 0.05;  // Smaller sigma = more selective
            double z_diff = measured - expected;
            log_likelihood += -0.5 * z_diff * z_diff / (sigma * sigma);
            valid_rays++;
        }
        
        if (valid_rays > 0) {
            // Convert back from log space with normalization
            p.weight *= exp(log_likelihood / valid_rays);
        } else {
            p.weight *= 1e-6;  // Much stronger penalty for complete mismatches
        }
    }
    
    normalizeWeights();
}

plastic_fundamentals::Pose estimatePose() {
    plastic_fundamentals::Pose pose;
    
    auto max_it = std::max_element(particles.begin(), particles.end(),
                                  [](const Particle& a, const Particle& b) {
                                      return a.weight < b.weight;
                                  });
    
    if (max_it != particles.end()) {
        pose.row = max_it->row;
        pose.column = max_it->col;
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
            
            // Convert to grid coordinates
            pose.row = static_cast<int>(avg_y / CELL_SIZE);
            pose.column = static_cast<int>(avg_x / CELL_SIZE);
            
            // Convert to grid orientation
            double normalized_theta = avg_theta;
            while (normalized_theta < 0) normalized_theta += 2 * M_PI;
            pose.orientation = static_cast<int>((normalized_theta + M_PI/4) / (M_PI/2)) % 4;
        }
    }
    
    return pose;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    actual_scan = std::vector<double>(msg->ranges.begin(), msg->ranges.end());

    if (processing_done) return;
    if (!map_data || particles.empty()) return;
    
    updateParticlesSensor(msg);
    
    resampleParticles();
    
    current_pose = estimatePose();

    double max_weight = 0;
    for (const auto& p : particles) {
        max_weight = std::max(max_weight, p.weight);
    }

    double n_eff = calculateEffectiveParticles();
    double effective_particles_threshold = 0.5 * NUM_PARTICLES;
    
    if (max_weight > 0.01 && n_eff > effective_particles_threshold) {
        is_localized = true;
        ROS_INFO("Localized at row=%d, column=%d, orientation=%d (confidence=%.3f)", 
                 current_pose.row, current_pose.column, current_pose.orientation, max_weight);

        pose_pub.publish(current_pose);
        
        if (first_localization) {
            first_localization = false;
            ROS_INFO("*BEEP* Successfully localized!");
        }
    }
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    min_distance = std::numeric_limits<float>::infinity();
    min_index = -1;
    // Define desired bounds in radians
    float angle_min_limit = -M_PI / 4; // -45 degrees
    float angle_max_limit =  M_PI / 4; // +45 degrees

    // Compute indices for -45 to +45 degrees
    start_index = std::max(0, static_cast<int>((angle_min_limit - msg->angle_min) / msg->angle_increment));
    end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, static_cast<int>((angle_max_limit - msg->angle_min) / msg->angle_increment));

    for(int i = start_index; i < end_index; i++){
        float value = msg->ranges[i];
        if(!isnan(value) && value >= msg->range_min && value <= msg->range_max){
            // Check if the value is less than the current minimum distance
            if(value < min_distance){
                min_distance = value;
                min_index = i;
            }
        }

    }



}

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    if (!particles.empty()) return;

    map_data = msg;
    ROS_INFO("Received map data");

    initializeParticles(map_data);
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    if(msg->bumpLeft == 1 || msg->bumpRight == 1) {
        ROS_INFO("Bumper hit");
        // make the robot move backward
        create_fundamentals::DiffDrive srv;
        srv.request.left = -10;
        srv.request.right = -10;
        diff_drive_client->call(srv);
        ros::Duration(0.5).sleep();
        // make the robot turn
        srv.request.left = -5;
        srv.request.right = 5;
        diff_drive_client->call(srv);
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mcl_wanderer");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, scanCallback);
    ros::Subscriber subLidar = nh.subscribe("/scan_filtered", 1, laserCallback);
    ros::Subscriber subSensor = nh.subscribe("/sensor_packet", 1, sensorCallback);

    ros::ServiceClient client = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;
    ros::ServiceClient reset_client = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    reset_encoders_client = &reset_client;
    ros::service::waitForService("reset_encoders");

    ros::service::waitForService("diff_drive");

    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(2).sleep();

        ros::spinOnce();
    }

    resetEncoders();

    double delta_trans = 0.0;
    double delta_rot = 0.0;
    double accumulated_rotation = 0;

    enum State {
        EXPLORING,
        ROTATING,
        TRANSLATING
    };

    State current_state = EXPLORING;
    State previous_state = EXPLORING;

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10); // Loop at 10 Hz
    while(ros::ok() && !is_localized) {
        // Behavior state machine
        switch(current_state) {
            case EXPLORING:
                if(min_distance < 0.3) {
                    // Object too close, need to rotate
                    current_state = ROTATING;
                    previous_state = EXPLORING;
                } else {
                    current_state = TRANSLATING;
                    previous_state = EXPLORING;
                }
                break;
                
            case ROTATING:
                if (previous_state == TRANSLATING) {
                    double delta_trans = getTranslationFromTicks(leftTicks, rightTicks);
                    x_bar_prime = x_bar + delta_trans * cos(theta_bar);
                    y_bar_prime = y_bar + delta_trans * sin(theta_bar);
                    updateParticlesMotion(x_bar, y_bar, theta_bar, x_bar_prime, y_bar_prime);
                    x_bar = x_bar_prime;
                    y_bar = y_bar_prime;
                    resetEncoders();
                }
                if (min_distance < 0.3) {
                    delta_rot = getRotationFromTicks(leftTicks, rightTicks);
                    resetEncoders();
                    
                    accumulated_rotation += delta_rot;
                    current_state = ROTATING;
                    previous_state = ROTATING;
                    int diff1 = min_index - start_index;
                    int diff2 = end_index - min_index;
                    if (diff1 < diff2) {
                        // Turn left
                        srv.request.left = -5;
                        srv.request.right = 5;
                        ros::Duration(0.5).sleep();
                    } else {
                        // Turn right
                        srv.request.left = 5;
                        srv.request.right = -5;
                        ros::Duration(0.5).sleep();
                    }
                }
                else {
                    theta_bar += accumulated_rotation;
                    current_state = TRANSLATING;
                    previous_state = ROTATING;
                }
                break;
                
            case TRANSLATING:
                resetEncoders();
                if(min_distance > 0.3) {
                    // Move forward
                    srv.request.left = 5;
                    srv.request.right = 5;
                } else {
                    // Object too close, need to rotate
                    current_state = ROTATING;
                    previous_state = TRANSLATING;
                }
                break;
        }

        // Execute the movement command
        if(!diff_drive_client->call(srv)) {
            ROS_ERROR("Failed to call DiffDrive service");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


