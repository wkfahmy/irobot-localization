#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
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

double x_bar = 0.0;
double y_bar = 0.0;
double theta_bar = 0.0;

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
constexpr double INIT_X_NOISE = 0.1;
constexpr double INIT_Y_NOISE = 0.1;
constexpr double INIT_THETA_NOISE = 0.2;
constexpr double MOTION_X_NOISE = 0.02;
constexpr double MOTION_Y_NOISE = 0.02;
constexpr double MOTION_THETA_NOISE = 0.05;
constexpr double RESAMPLE_THRESHOLD = 0.5;

ros::ServiceClient* diff_drive_client;
plastic_fundamentals::Grid::ConstPtr map_data;
plastic_fundamentals::Pose current_pose;
bool is_localized = false;
bool first_localization = true;

bool processing_done = false;

ros::Publisher particle_pub;
ros::Publisher pose_pub;
std::default_random_engine random_generator;

std::vector<double> actual_scan;

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
    int orientation;    // Grid orientation (0-3)
};

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

void resetEncoders(ros::ServiceClient& resetClient) {
    create_fundamentals::ResetEncoders srv;
    if (!resetClient.call(srv)) {
        ROS_WARN("Failed to reset encoders");
    } else {
        leftTicks = 0;
        rightTicks = 0;
    }
}

// Global particle set
std::vector<Particle> particles;

void initializeParticles(const plastic_fundamentals::Grid::ConstPtr& map) {
    if (!map) {
        ROS_ERROR("Cannot initialize particles: No map data");
        return;
    }
    
    particles.clear();
    particles.reserve(NUM_PARTICLES);
    
    // Create distributions for random noise
    std::uniform_real_distribution<double> x_noise_dist(-INIT_X_NOISE, INIT_X_NOISE);
    std::uniform_real_distribution<double> y_noise_dist(-INIT_Y_NOISE, INIT_Y_NOISE);
    std::uniform_real_distribution<double> theta_noise_dist(-INIT_THETA_NOISE, INIT_THETA_NOISE);
    
    int num_rows = map->rows.size();
    int num_cols = map->rows[0].cells.size();
    int total_cells = num_rows * num_cols;
    
    int particles_per_cell = NUM_PARTICLES / (total_cells * 4);
    if (particles_per_cell < 1) particles_per_cell = 1;
    
    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_cols; ++col) {
            double cell_center_x = (col + 0.5) * CELL_SIZE;
            double cell_center_y = (row + 0.5) * CELL_SIZE;
            
            for (int orientation = 0; orientation < 4; ++orientation) {
                double base_theta = orientation * M_PI / 2.0;
                
                for (int i = 0; i < particles_per_cell; ++i) {
                    if (particles.size() >= NUM_PARTICLES) break;
                    
                    Particle p;
                    p.x = cell_center_x + x_noise_dist(random_generator);
                    p.y = cell_center_y + y_noise_dist(random_generator);
                    p.theta = base_theta + theta_noise_dist(random_generator);
                    p.weight = 1.0 / NUM_PARTICLES;
                    p.row = row;
                    p.col = col;
                    p.orientation = orientation;
                    
                    particles.push_back(p);
                }
            }
        }
    }
    
    // If we didn't create enough particles, duplicate some
    while (particles.size() < NUM_PARTICLES) {
        int idx = rand() % particles.size();
        particles.push_back(particles[idx]);
    }
    
    ROS_INFO("Initialized %zu particles", particles.size());

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

void updateParticlesMotion(double x_bar, double y_bar, double theta_bar,
                           double x_bar_prime, double y_bar_prime, double theta_bar_prime) {
    // Motion noise parameters (tune these for your system)
    constexpr double ALPHA1 = 0.1;
    constexpr double ALPHA2 = 0.1;
    constexpr double ALPHA3 = 0.2;
    constexpr double ALPHA4 = 0.2;

    // Compute odometry-based motion components
    double delta_rot1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar;
    double delta_trans = std::sqrt(std::pow(x_bar_prime - x_bar, 2) + std::pow(y_bar_prime - y_bar, 2));
    double delta_rot2 = theta_bar_prime - theta_bar - delta_rot1;

    // Normalize angles
    while (delta_rot1 > M_PI) delta_rot1 -= 2 * M_PI;
    while (delta_rot1 <= -M_PI) delta_rot1 += 2 * M_PI;
    while (delta_rot2 > M_PI) delta_rot2 -= 2 * M_PI;
    while (delta_rot2 <= -M_PI) delta_rot2 += 2 * M_PI;

    for (auto& p : particles) {
        // Compute noise variances
        double var_rot1 = ALPHA1 * delta_rot1 * delta_rot1 + ALPHA2 * delta_trans * delta_trans;
        double var_trans = ALPHA3 * delta_trans * delta_trans + ALPHA4 * (delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2);
        double var_rot2 = ALPHA1 * delta_rot2 * delta_rot2 + ALPHA2 * delta_trans * delta_trans;

        // Sample noisy motion components
        std::normal_distribution<double> rot1_noise(0, std::sqrt(var_rot1));
        std::normal_distribution<double> trans_noise(0, std::sqrt(var_trans));
        std::normal_distribution<double> rot2_noise(0, std::sqrt(var_rot2));

        double delta_rot1_hat = delta_rot1 + rot1_noise(random_generator);
        double delta_trans_hat = delta_trans + trans_noise(random_generator);
        double delta_rot2_hat = delta_rot2 + rot2_noise(random_generator);

        // Update particle (hypothetical position update)
        p.x += delta_trans_hat * cos(p.theta + delta_rot1_hat);
        p.y += delta_trans_hat * sin(p.theta + delta_rot1_hat);
        p.theta += delta_rot1_hat + delta_rot2_hat;

        // Normalize angle
        while (p.theta > M_PI) p.theta -= 2 * M_PI;
        while (p.theta <= -M_PI) p.theta += 2 * M_PI;

        // Update grid indices
        p.col = static_cast<int>(p.x / CELL_SIZE);
        p.row = static_cast<int>(p.y / CELL_SIZE);

        // Update grid orientation (0: right, 1: up, 2: left, 3: down)
        double normalized_theta = p.theta;
        while (normalized_theta < 0) normalized_theta += 2 * M_PI;
        p.orientation = static_cast<int>((normalized_theta + M_PI/4) / (M_PI/2)) % 4;
    }

    // Log motion update for visualization
    data_log << ros::Time::now() << ",motion_update_odometry," << x_bar << "," << y_bar << "," << theta_bar << ","
             << x_bar_prime << "," << y_bar_prime << "," << theta_bar_prime << std::endl;
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

plastic_fundamentals::Pose estimatePose() {
    plastic_fundamentals::Pose pose;
    
    auto max_it = std::max_element(particles.begin(), particles.end(),
                                  [](const Particle& a, const Particle& b) {
                                      return a.weight < b.weight;
                                  });
    
    if (max_it != particles.end()) {
        pose.row = max_it->row;
        pose.column = max_it->col;
        pose.orientation = max_it->orientation;
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

void spinInPlace(ros::ServiceClient& diffDriveClient, ros::ServiceClient& resetClient, double angle_rad, double speed) {
    double direction = (angle_rad >= 0) ? 1.0 : -1.0;
    double ticksTarget = getRotationTicks(angle_rad);

    double x_bar_before = x_bar;
    double y_bar_before = y_bar;
    double theta_bar_before = theta_bar;

    resetEncoders(resetClient);

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10);

    srv.request.left = -direction * speed;
    srv.request.right = direction * speed;

    while ((std::abs(leftTicks) + std::abs(rightTicks)) / 2 < std::abs(ticksTarget)) {
        double correction = 0.0;
        if (std::abs(rightTicks) > 0.01) {
            correction = 1 - std::abs(leftTicks) / std::abs(rightTicks);
            srv.request.left = -direction * speed * (1 + correction / 2);
            srv.request.right = direction * speed * (1 - correction / 2);
        }

        diffDriveClient.call(srv);
        ros::spinOnce();
        rate.sleep();
    }

    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient.call(srv);

    // Estimate change in orientation
    double dtheta = getRotationFromTicks(leftTicks, rightTicks);

    double x_bar_after = x_bar_before;
    double y_bar_after = y_bar_before;
    double theta_bar_after = theta_bar_before + dtheta;

    // Normalize angle
    while (theta_bar_after > M_PI) theta_bar_after -= 2 * M_PI;
    while (theta_bar_after < -M_PI) theta_bar_after += 2 * M_PI;

    updateParticlesMotion(x_bar_before, y_bar_before, theta_bar_before, x_bar_after, y_bar_after, theta_bar_after);

    x_bar = x_bar_after;
    y_bar = y_bar_after;
    theta_bar = theta_bar_after;

    resetEncoders(resetClient);
}

bool isClearPath(const std::vector<double>& actual_scan, double obstacle_threshold) {
    double min_angle = -M_PI / 4;
    double max_angle = M_PI / 4;

    double angle_resolution = (max_angle - min_angle) / actual_scan.size();

    size_t left_index = static_cast<size_t>((-M_PI / 2 - min_angle) / angle_resolution);
    size_t right_index = static_cast<size_t>((M_PI / 2 - min_angle) / angle_resolution);

    left_index = std::max(left_index, static_cast<size_t>(0));
    right_index = std::min(right_index, actual_scan.size() - 1);

    for (size_t i = left_index; i <= right_index; ++i) {
        double distance = actual_scan[i];
        if (distance < obstacle_threshold) {
            return false;
        }
    }

    return true;
}

void moveLinear(ros::ServiceClient& diffDriveClient, ros::ServiceClient& resetClient, double distance_m, double speed) {
    double direction = (distance_m >= 0) ? 1.0 : -1.0;
    double ticksTarget = getTranslationTicks(distance_m);

    // Store pose before motion
    double x_bar_before = x_bar;
    double y_bar_before = y_bar;
    double theta_bar_before = theta_bar;

    resetEncoders(resetClient);

    create_fundamentals::DiffDrive srv;
    ros::Rate rate(10);

    srv.request.left = direction * speed;
    srv.request.right = direction * speed;

    while ((std::abs(leftTicks) + std::abs(rightTicks)) / 2 < std::abs(ticksTarget)) {
        double correction = 0.0;
        if (std::abs(rightTicks) > 0.01) {
            correction = 1 - std::abs(leftTicks) / std::abs(rightTicks);
            srv.request.left = direction * speed * (1 + correction / 2);
            srv.request.right = direction * speed * (1 - correction / 2);
        }

        diffDriveClient.call(srv);
        ros::spinOnce();
        rate.sleep();
    }

    srv.request.left = 0;
    srv.request.right = 0;
    diffDriveClient.call(srv);

    // Odometry-based displacement
    double dx = getTranslationFromTicks(leftTicks, rightTicks);
    double x_bar_after = x_bar_before + dx * cos(theta_bar_before);
    double y_bar_after = y_bar_before + dx * sin(theta_bar_before);
    double theta_bar_after = theta_bar_before;

    // Update particles
    updateParticlesMotion(x_bar_before, y_bar_before, theta_bar_before, x_bar_after, y_bar_after, theta_bar_after);

    // Update robot pose
    x_bar = x_bar_after;
    y_bar = y_bar_after;
    theta_bar = theta_bar_after;

    resetEncoders(resetClient);
}

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    map_data = msg;
    ROS_INFO("Received map data");
    
    if (!particles.empty()) return;
    
    initializeParticles(map_data);
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
                    spinInPlace(*diff_drive_client, M_PI / 2, 5.0);
                    localization_phase = MOVE;
                    break;

                case MOVE:
                    if (isClearPath(actual_scan, move_distance)) {
                        ROS_INFO("Moving robot to cover more area...");
                        moveLinear(*diff_drive_client, move_distance, 3.0);
                        localization_phase = SPIN;
                    } else {
                        ROS_INFO("Obstacle detected, changing direction...");
                        spinInPlace(*diff_drive_client, M_PI / 2, 5.0);
                    }
                    break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcl_localization");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, scanCallback);
    
    pose_pub = nh.advertise<plastic_fundamentals::Pose>("/pose", 10);
    particle_pub = nh.advertise<visualization_msgs::MarkerArray>("/particles", 10);
    
    ros::ServiceClient client = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;
    
    ROS_INFO("MCL localization node started");


    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(2).sleep();

        ros::spinOnce();
    }

    ros::Rate rate(20);

    ros::spinOnce();
    rate.sleep();

    localizationRoutine(rate);

    while (ros::ok()) {
        pose_pub.publish(current_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
