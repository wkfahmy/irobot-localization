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


constexpr double WHEEL_RADIUS_M = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M = 0.263;     // 26.3 cm
constexpr double LIDAR_OFFSET_M = 0.16;     // 16 cm from robot center
constexpr double CELL_SIZE = 0.8;         // 80 cm cell size

constexpr int NUM_PARTICLES = 10000;         // Number of particles

enum LocalizationPhase {
    SPIN,
    MOVE,
    LOCALIZE
};

void controlLoop() {
    switch (state) {
        case SPIN:
            spinInPlace(*diff_drive_client, 2*M_PI, 2.0);
            state = LOCALIZE;
            break;
        case LOCALIZE:
            ros::spinOnce();
            if (isLocalizedEnough()) {
                state = MOVING;
            } else {
                state = SPINNING;
            }
            break;
        case MOVE:
            moveLinear(*diff_drive_client, move_distance, 2.0);
            state = SPINNING;
            break;
    }
}

double intersectRaySegment(double ox, double oy, double dx, double dy, double x1, double y1, double x2, double y2)
{
    double den = (x1 - x2) * dy - (y1 - y2) * dx;
    if (std::abs(den) < 1e-8) return std::numeric_limits<double>::infinity();

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

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
    int row;
    int col;
    int orientation;
};

std::vector<Particle> particles;

void initializeParticles(const plastic_fundamentals::Grid::ConstPtr& map) {
    if (!map) {
        ROS_ERROR("Cannot initialize particles: No map data");
        return;
    }

    particles.clear();
    particles.reserve(NUM_PARTICLES);

    int num_rows = map->rows.size();
    int num_cols = map->rows[0].cells.size();

    std::uniform_real_distribution<double> x_dist(0, num_cols * CELL_SIZE);
    std::uniform_real_distribution<double> y_dist(0, num_rows * CELL_SIZE);
    std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle p;
        p.x = x_dist(random_generator);
        p.y = y_dist(random_generator);
        p.theta = theta_dist(random_generator);
        p.weight = 1.0 / NUM_PARTICLES;

        if (p.x % CELL_SIZE == 0 || p.y % CELL_SIZE == 0) {

            p.x += 0.01 * (random_generator() % 2 == 0 ? 1 : -1);
            p.y += 0.01 * (random_generator() % 2 == 0 ? 1 : -1);
        }

        particles.push_back(p);
    }
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

double gaussianLikelihood(double measurement, double expected, double sigma) {
    double diff = measurement - expected;
    return std::exp(-0.5 * diff * diff / (sigma * sigma));
}


double gaussianLikelihood(double measurement, double expected, double sigma) {
    double diff = measurement - expected;
    return std::exp(-0.5 * diff * diff / (sigma * sigma));
}

void updateParticleWeights(
    std::vector<Particle>& particles,
    const std::vector<double>& actual_scan,
    const std::vector<double>& scan_angles,
    double sensor_sigma,
    double lidar_offset,
    const plastic_fundamentals::Grid& map_data)
{
    for (auto& p : particles) {
        std::vector<double> expected_scan = calculateExpectedScan(p, scan_angles);

        double weight = 1.0;
        for (size_t i = 0; i < scan_angles.size() && i < actual_scan.size(); ++i) {
            double expected_dist = expected_scan[i];
            double actual_dist = actual_scan[i];

            if (std::isinf(expected_dist)) {
                if (actual_dist < 0.9) {
                    weight *= 0.01;
                }
                continue;
            }

            weight *= gaussianLikelihood(actual_dist, expected_dist, sensor_sigma);
        }

        p.weight = weight;
    }

    double sum_weights = 0.0;
    for (const auto& p : particles) sum_weights += p.weight;

    if (sum_weights > 0) {
        for (auto& p : particles) p.weight /= sum_weights;
    } else {
        double uniform_weight = 1.0 / particles.size();
        for (auto& p : particles) p.weight = uniform_weight;
    }
}

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    ROS_INFO("Received map data");

    initializeDistanceField(msg);
    initializeParticles(msg);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (processing_done) return;
    /* if (!map_data || particles.empty()) return;

    updateParticlesSensor(msg);
    resampleParticles();

    current_pose = estimatePose();

    double max_weight = 0;
    for (const auto& p : particles) {
        max_weight = std::max(max_weight, p.weight);
    }

    if (max_weight > 0.001) {
        is_localized = true;
        ROS_INFO("Localized at row=%d, column=%d, orientation=%d (confidence=%.3f)",
                 current_pose.row, current_pose.column, current_pose.orientation, max_weight);

        pose_pub.publish(current_pose);

        static bool first_localization = true;
        if (first_localization) {
            first_localization = false;
            ROS_INFO("*BEEP* Successfully localized!");

            system("echo -e '\a'");
        }
    }*/
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

    ros::Rate rate(10);
    while (ros::ok() && !processing_done) {
        if (is_localized) {
            processing_done = true;
            ROS_INFO("Localization complete, publishing pose and waiting");
        } else {
            // If not localized yet, perform additional rotations to gather more information
            static ros::Time last_rotation_time = ros::Time::now();
            if ((ros::Time::now() - last_rotation_time).toSec() > 5.0) {
                ROS_INFO("Performing additional rotation to gather more information...");
                spinInPlace(*diff_drive_client, M_PI/2, 2.0);
                last_rotation_time = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
