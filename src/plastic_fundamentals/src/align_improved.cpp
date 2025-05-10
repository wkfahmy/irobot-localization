/********* BEGIN MODIFIED align_improved.cpp (with comments) *********/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>        // For std::cmath, M_PI, M_PI_2
#include <random>       // For std::random_device, std::mt19937, std::uniform_int_distribution
#include <limits>       // For std::numeric_limits
#include <algorithm>    // For std::sort, std::min, std::max
#include <numeric>      // For std::accumulate (not explicitly used but good for vector ops)

// --- Configuration Constants ---
// These parameters define the robot, environment, and algorithm behavior.
// They can be tuned for optimal performance.

// Physical and Environmental Parameters
const double LIDAR_OFFSET_X = 0.16; // [m] Forward offset of LiDAR from robot's rotation center (base_link x-axis).
const double CELL_DIMENSION = 0.80; // [m] Assumed dimension (width and height) of the square cell.
const double CELL_HALF_DIMENSION = CELL_DIMENSION / 2.0; // [m] Half dimension, useful for centering calculations.

// RANSAC Algorithm Parameters
const int RANSAC_ITERATIONS = 100;            // Number of iterations RANSAC performs to find the best line model.
const double RANSAC_DISTANCE_THRESHOLD = 0.05; // [m] Max distance for a point to be considered an inlier to a line.
const int RANSAC_MIN_INLIERS_FOR_LINE = 10; // Minimum number of inlier points required to validate a detected line.
const int RANSAC_MIN_INLIERS_FOR_REFIT = 5; // Minimum inliers to attempt refitting a line with least squares for better accuracy.

// Control System Parameters
const double KP_LINEAR = 0.4;           // Proportional gain for linear velocity control.
const double KP_ANGULAR = 0.7;          // Proportional gain for angular velocity control.
const double DISTANCE_TOLERANCE = 0.03; // [m] Target position tolerance; robot is considered centered if within this distance.
const double ANGLE_TOLERANCE = 0.05;    // [rad] Target orientation tolerance (approx. 2.86 degrees).
const double MAX_LINEAR_SPEED = 0.15;   // [m/s] Maximum allowed linear speed of the robot.
const double MAX_ANGULAR_SPEED = 0.4;   // [rad/s] Maximum allowed angular speed of the robot.

// Pose Estimation Parameters
const double ANGLE_SIMILARITY_THRESHOLD = M_PI / 12.0; // [rad] Approx 15 deg. Used for grouping lines with similar angles.
const double ORTHOGONAL_THRESHOLD = M_PI / 12.0;     // [rad] Approx 15 deg. Tolerance for considering two lines orthogonal.

// --- Data Structures ---

// Represents a 2D point in Cartesian coordinates.
struct Point {
    double x, y;
};

// Represents a line in 2D space using the general form ax + by + c = 0.
struct Line {
    double a, b, c;                 // Coefficients of the line equation. Normalized such that a*a + b*b = 1.
    int inliers = 0;                // Number of points that are considered inliers to this line.
    std::vector<Point> inlier_points; // Actual inlier points, used for refitting the line.
    double angle_rad = 0.0;         // Angle of the line's direction vector (e.g., (b, -a)) in radians, typically in [-PI, PI].
    double distance_from_origin = 0.0; // Signed distance from the origin (0,0) to the line. If normalized, this is 'c'.
};

// --- Global Variables ---
// These variables are accessible throughout the node.

ros::Publisher cmd_vel_pub; // ROS publisher for sending velocity commands to the robot.
std::vector<Point> current_scan_points_robot_frame; // Stores the latest LiDAR scan points, transformed into the robot's base_link frame.
bool scan_received = false; // Flag indicating whether a new laser scan has been received and processed.

// Global storage for the latest estimated pose relative to the cell center.
double estimated_x_offset_global = 0.0;     // [m] Estimated X offset of the robot from the cell center.
double estimated_y_offset_global = 0.0;     // [m] Estimated Y offset of the robot from the cell center.
double estimated_angle_error_global = 0.0;  // [rad] Estimated angular error of the robot relative to the cell's X-axis.
bool pose_successfully_estimated = false; // Flag indicating if the last pose estimation attempt was successful.

// State machine for managing the alignment process.
enum AlignState {
    INITIALIZING,       // Initial state, preparing for operation.
    FINDING_WALLS,      // Actively trying to detect lines (walls) from LiDAR data using RANSAC.
    ESTIMATING_POSE,    // Using detected lines to estimate the robot's pose within the cell.
    MOVING_TO_CENTER,   // Controlling the robot to move towards the cell center.
    ALIGNING_ANGLE,     // Controlling the robot to align its orientation with the cell axes.
    FINISHED,           // Alignment task completed successfully.
    EXPLORATORY_TURN    // Performing a slow turn if pose estimation fails repeatedly, to get better sensor readings.
};
AlignState current_state = INITIALIZING; // Current state of the alignment process.

int consecutive_pose_failures = 0;              // Counter for consecutive failures in RANSAC or pose estimation.
const int MAX_CONSECUTIVE_POSE_FAILURES = 5;    // Threshold for switching to EXPLORATORY_TURN state.

// --- Utility Functions ---

/**
 * @brief Normalizes an angle to the range [-PI, PI].
 * @param angle The input angle in radians.
 * @return The normalized angle in radians.
 */
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief Calculates the perpendicular distance from a point to a line.
 * Assumes the line 'l' is normalized (l.a*l.a + l.b*l.b = 1).
 * @param p The point.
 * @param l The line.
 * @return The perpendicular distance.
 */
double point_line_distance(Point p, const Line& l) {
    return std::fabs(l.a * p.x + l.b * p.y + l.c);
}

/**
 * @brief Fits a line to two given points and normalizes its equation.
 * @param p1 The first point.
 * @param p2 The second point.
 * @return The fitted Line structure.
 */
Line fit_line_to_two_points(Point p1, Point p2) {
    Line l;
    l.a = p2.y - p1.y; // Coefficient a = y2 - y1
    l.b = p1.x - p2.x; // Coefficient b = x1 - x2
    double norm = std::sqrt(l.a * l.a + l.b * l.b);

    if (norm < 1e-6) { // Points are identical or very close, avoid division by zero.
        // Default to a vertical line through p1 if points are coincident.
        l.a = 1.0;
        l.b = 0.0;
        l.c = -p1.x;
    } else {
        // Normalize coefficients a and b.
        l.a /= norm;
        l.b /= norm;
        // Calculate c using one of the points: ax + by + c = 0 => c = -(ax + by).
        l.c = -(l.a * p1.x + l.b * p1.y);
    }
    // Angle of the line's direction vector (b, -a). atan2 provides angle in [-PI, PI].
    l.angle_rad = std::atan2(-l.a, l.b);
    // Since a,b are normalized, c is the signed distance from origin to the line.
    l.distance_from_origin = l.c;
    return l;
}

/**
 * @brief Refits a line using least squares to a given set of points.
 * This provides a more accurate line model than using just two points from RANSAC.
 * @param points The set of inlier points to fit the line to.
 * @param line_out Output parameter for the refitted line.
 * @return True if refit was successful, false otherwise (e.g., too few points).
 */
bool refit_line_least_squares(const std::vector<Point>& points, Line& line_out) {
    if (points.size() < RANSAC_MIN_INLIERS_FOR_REFIT) return false;

    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_yy = 0, sum_xy = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xx += p.x * p.x;
        sum_yy += p.y * p.y;
        sum_xy += p.x * p.y;
    }
    int N = points.size();
    double mean_x = sum_x / N;
    double mean_y = sum_y / N;

    // Calculate components of the covariance matrix of the points (centered).
    double Sxx = sum_xx - N * mean_x * mean_x; // sum_i (x_i - mean_x)^2
    double Syy = sum_yy - N * mean_y * mean_y; // sum_i (y_i - mean_y)^2
    double Sxy = sum_xy - N * mean_x * mean_y; // sum_i (x_i - mean_x)(y_i - mean_y)

    // Determine line parameters based on the principal component (eigenvector for largest eigenvalue of covariance matrix).
    // This method is more robust for lines that are nearly vertical or horizontal.
    if (Sxx < Syy) { // Line is more vertical (larger variance in Y when projected onto Y axis).
        // Fit x = my + q (regress x against y).
        double denominator_m = (N * sum_yy - sum_y * sum_y);
        double m = (std::fabs(denominator_m) < 1e-6) ? 0 : (N * sum_xy - sum_y * sum_x) / denominator_m;
        double q = (sum_x - m * sum_y) / N;
        // Line equation: 1*x - m*y - q = 0
        line_out.a = 1.0;
        line_out.b = -m;
        line_out.c = -q;
    } else { // Line is more horizontal.
        // Fit y = mx + q (regress y against x).
        double denominator_m = (N * sum_xx - sum_x * sum_x);
        double m = (std::fabs(denominator_m) < 1e-6) ? 0 : (N * sum_xy - sum_x * sum_y) / denominator_m;
        double q = (sum_y - m * sum_x) / N;
        // Line equation: m*x - 1*y + q = 0
        line_out.a = m;
        line_out.b = -1.0;
        line_out.c = q;
    }

    // Normalize the line equation.
    double norm = std::sqrt(line_out.a * line_out.a + line_out.b * line_out.b);
    if (norm < 1e-6) return false; // Should not happen if N >= 2 and points are not all identical.
    line_out.a /= norm;
    line_out.b /= norm;
    line_out.c /= norm;

    line_out.angle_rad = std::atan2(-line_out.a, line_out.b); // Angle of line direction vector (b, -a).
    line_out.distance_from_origin = line_out.c; // Signed distance from origin.
    line_out.inliers = N; // Update inlier count to actual points used for refit.
    line_out.inlier_points = points; // Store the points used for this refit.
    return true;
}

// --- RANSAC Implementation ---
/**
 * @brief Finds multiple lines in a 2D point cloud using the RANSAC algorithm.
 * Iteratively finds the best line, refits it, and removes its inliers to find subsequent lines.
 * @param points The input vector of 2D points.
 * @return A vector of Line structures representing the detected lines.
 */
std::vector<Line> find_lines_ransac(const std::vector<Point>& points) {
    std::vector<Line> found_lines;
    if (points.size() < 2) return found_lines; // Not enough points to form a line.

    std::vector<Point> remaining_points = points; // Work on a copy of points to remove inliers.
    std::random_device rd; // Seed for random number generator.
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd().

    // Attempt to find up to 4 lines (e.g., for a square cell).
    for (int line_count = 0; line_count < 4 && remaining_points.size() >= RANSAC_MIN_INLIERS_FOR_LINE; ++line_count) {
        int max_inliers_for_current_ransac_run = -1;
        std::vector<Point> best_inlier_set_for_current_ransac_run;

        // RANSAC iterations to find one line model.
        for (int i = 0; i < RANSAC_ITERATIONS; ++i) {
            if (remaining_points.size() < 2) break; // Not enough points left.

            // 1. Randomly select 2 distinct points from remaining_points.
            std::uniform_int_distribution<> distrib(0, remaining_points.size() - 1);
            int idx1 = distrib(gen);
            int idx2 = distrib(gen);
            if (idx1 == idx2) continue; // Ensure distinct points.

            Point p1 = remaining_points[idx1];
            Point p2 = remaining_points[idx2];

            // 2. Fit a line model to these two points.
            Line current_model = fit_line_to_two_points(p1, p2);

            // 3. Count inliers: points from remaining_points close to current_model.
            std::vector<Point> current_inlier_set;
            for (const auto& p_test : remaining_points) {
                if (point_line_distance(p_test, current_model) < RANSAC_DISTANCE_THRESHOLD) {
                    current_inlier_set.push_back(p_test);
                }
            }

            // 4. If this model has more inliers than previous best, update best model.
            if (current_inlier_set.size() > max_inliers_for_current_ransac_run) {
                max_inliers_for_current_ransac_run = current_inlier_set.size();
                best_inlier_set_for_current_ransac_run = current_inlier_set;
            }
        }

        // 5. If the best model found has enough inliers, accept it.
        if (max_inliers_for_current_ransac_run >= RANSAC_MIN_INLIERS_FOR_LINE) {
            Line refitted_line;
            // Refit the line using all its inliers for better accuracy.
            if (refit_line_least_squares(best_inlier_set_for_current_ransac_run, refitted_line)) {
                found_lines.push_back(refitted_line);

                // Remove these inliers from remaining_points for the next iteration.
                std::vector<Point> next_remaining_points;
                for (const auto& p_orig : remaining_points) {
                    bool is_inlier_of_refitted_line = false;
                    // Check if p_orig was part of the best_inlier_set (approximate check by coordinates).
                    for (const auto& p_inlier : best_inlier_set_for_current_ransac_run) {
                        if (std::fabs(p_orig.x - p_inlier.x) < 1e-6 && std::fabs(p_orig.y - p_inlier.y) < 1e-6) {
                            is_inlier_of_refitted_line = true;
                            break;
                        }
                    }
                    if (!is_inlier_of_refitted_line) {
                        next_remaining_points.push_back(p_orig);
                    }
                }
                remaining_points = next_remaining_points;
            } else {
                break; // Refit failed, stop trying to find more lines.
            }
        } else {
            break; // No more significant lines found (not enough inliers).
        }
    }
    return found_lines;
}

// --- Pose Estimation Logic ---
/**
 * @brief Estimates the robot's pose (x, y offset from cell center, and angular error) based on detected lines.
 * This is a simplified pose estimation. It tries to identify cell axes based on dominant line orientations.
 * @param lines Vector of Line structures detected by RANSAC (in robot's base_link frame).
 * @param est_x_offset Output: estimated X offset from cell center.
 * @param est_y_offset Output: estimated Y offset from cell center.
 * @param est_angle_err Output: estimated angular error relative to cell's X-axis.
 * @return True if pose estimation was successful (at least partially), false otherwise.
 */
bool perform_pose_estimation(const std::vector<Line>& lines,
                             double& est_x_offset, double& est_y_offset, double& est_angle_err) {
    if (lines.empty()) {
        ROS_WARN("Pose Estimation: No lines provided.");
        return false;
    }

    // Sort lines by number of inliers (descending) to prioritize stronger lines.
    std::vector<Line> sorted_lines = lines;
    std::sort(sorted_lines.begin(), sorted_lines.end(), [](const Line& a, const Line& b) {
        return a.inliers > b.inliers;
    });

    // --- Step 1: Estimate Cell Orientation (Angular Error) ---
    // This part tries to determine the orientation of the cell's axes in the robot's current frame.
    // est_angle_err will be the angle the robot needs to turn to align its X-axis with the cell's X-axis.
    double dominant_line_angle = sorted_lines[0].angle_rad; // Angle of the strongest detected line.

    // The cell's X-axis is assumed to be either parallel or perpendicular to this dominant line.
    // Candidate 1: Cell X-axis is parallel to the dominant line. Robot needs to turn by -dominant_line_angle.
    double angle_err_cand1 = normalize_angle(dominant_line_angle);
    // Candidate 2: Cell X-axis is perpendicular to the dominant line. Robot needs to turn by -(dominant_line_angle - PI/2).
    double angle_err_cand2 = normalize_angle(dominant_line_angle - M_PI_2);

    // Choose the candidate error that requires less rotation.
    est_angle_err = (std::fabs(angle_err_cand1) < std::fabs(angle_err_cand2)) ? angle_err_cand1 : angle_err_cand2;
    // est_angle_err is now the estimated angle of the cell's X-axis in the robot's frame.
    // A positive error means the robot needs to turn counter-clockwise to align.

    // --- Step 2: Estimate Position (X, Y Offsets) ---
    // Rotate all detected lines into a conceptual "cell-aligned robot frame".
    // This means if the robot were perfectly aligned with the cell (est_angle_err = 0),
    // these rotated lines would represent walls parallel to the cell's X or Y axes.
    double rotation_to_align_robot_with_cell = -est_angle_err;
    double cos_rot = std::cos(rotation_to_align_robot_with_cell);
    double sin_rot = std::sin(rotation_to_align_robot_with_cell);

    std::vector<double> x_wall_coords; // X-coordinates of vertical walls in cell-aligned frame.
    std::vector<double> y_wall_coords; // Y-coordinates of horizontal walls in cell-aligned frame.

    for (const auto& l_robot_frame : sorted_lines) {
        // The line in robot frame is l_robot_frame.a*x + l_robot_frame.b*y + l_robot_frame.c = 0
        // We need its parameters in the cell-aligned frame.
        // The normal vector (a,b) in robot frame rotates to (a_cell, b_cell) in cell frame.
        // a_cell = l_robot_frame.a * cos_rot - l_robot_frame.b * sin_rot;
        // b_cell = l_robot_frame.a * sin_rot + l_robot_frame.b * cos_rot;
        // The constant c remains the same as it's related to distance from origin.
        // Line angle in robot frame: l_robot_frame.angle_rad
        // Line angle in cell_aligned_frame: normalize_angle(l_robot_frame.angle_rad + rotation_to_align_robot_with_cell)

        double line_normal_angle_in_robot_frame = std::atan2(l_robot_frame.b, l_robot_frame.a);
        double line_normal_angle_in_cell_aligned_frame = normalize_angle(line_normal_angle_in_robot_frame + rotation_to_align_robot_with_cell);
        double dist_from_origin_to_line = -l_robot_frame.c; // ax+by=-c. If normal (a,b) points from origin to line, -c/norm is dist.
                                                          // Since our line is normalized, -c is distance along normal.
                                                          // More directly: distance_from_origin is already l_robot_frame.c (signed distance)
                                                          // If normal (a,b) points away from origin, c is negative, dist = -c.
                                                          // If normal (a,b) points towards origin, c is positive, dist = c.
                                                          // Our 'c' is -(a*x0 + b*y0), so -c = a*x0 + b*y0. This is projection of origin point onto normal.
                                                          // Let's use l_robot_frame.distance_from_origin which is 'c'.
                                                          // The distance of origin to line ax+by+c=0 is |c|/sqrt(a^2+b^2) = |c|.
                                                          // The signed distance is c. If c > 0, origin is on positive side of normal. If c < 0, on negative side.

        // Check if the line (in cell-aligned frame) is mostly vertical (normal along X-axis).
        if (std::fabs(std::cos(line_normal_angle_in_cell_aligned_frame)) > std::cos(M_PI/4.0 + ANGLE_SIMILARITY_THRESHOLD)) { // Normal mostly along X-axis
            // This line is a potential

/********* APPENDING COMMENTS to align_improved_commented.cpp *********/
            // This line is a potential "vertical" wall in the cell-aligned frame.
            // Its x-coordinate in this frame is its signed distance from the robot's (now cell-aligned) Y-axis.
            // The distance l_robot_frame.distance_from_origin is 'c' from ax+by+c=0.
            // If normal (a,b) points towards +X_cell_aligned, then c = -X_wall_coord.
            // If normal (a,b) points towards -X_cell_aligned, then c = +X_wall_coord.
            // The normal angle is atan2(b_cell, a_cell). If this is ~0, normal is +X. If ~PI, normal is -X.
            double x_coord_of_wall_in_cell_aligned_frame = -l_robot_frame.distance_from_origin / std::cos(line_normal_angle_in_cell_aligned_frame);

            // Filter out walls that are too far to be part of the 80cm cell centered roughly at robot
            if (std::fabs(x_coord_of_wall_in_cell_aligned_frame) < CELL_DIMENSION * 0.85) { // Allow some margin
                 x_wall_coords.push_back(x_coord_of_wall_in_cell_aligned_frame);
            }
        }
        // Check if the line (in cell-aligned frame) is mostly horizontal (normal along Y-axis).
        else if (std::fabs(std::sin(line_normal_angle_in_cell_aligned_frame)) > std::sin(M_PI/4.0 - ANGLE_SIMILARITY_THRESHOLD)) { // Normal mostly along Y-axis
            // This line is a potential "horizontal" wall.
            double y_coord_of_wall_in_cell_aligned_frame = -l_robot_frame.distance_from_origin / std::sin(line_normal_angle_in_cell_aligned_frame);
            if (std::fabs(y_coord_of_wall_in_cell_aligned_frame) < CELL_DIMENSION * 0.85) {
                y_wall_coords.push_back(y_coord_of_wall_in_cell_aligned_frame);
            }
        }
    }

    // --- Step 3: Calculate Offsets from Wall Coordinates ---
    // The robot's estimated position (est_x_offset, est_y_offset) is its center in the cell-aligned frame.
    // The cell center is at (0,0) in this frame.
    bool x_offset_determined = false;
    bool y_offset_determined = false;

    if (x_wall_coords.size() >= 2) {
        // Two vertical walls found. Assume they are the left and right walls of the cell.
        std::sort(x_wall_coords.begin(), x_wall_coords.end());
        double x_left_wall = x_wall_coords.front();    // Should be negative (e.g., -0.4m for cell center at 0)
        double x_right_wall = x_wall_coords.back();   // Should be positive (e.g., +0.4m)
        // Check if these form a plausible cell width.
        if (std::fabs((x_right_wall - x_left_wall) - CELL_DIMENSION) < CELL_DIMENSION * 0.35) { // Generous tolerance for width
            // Robot's X position is the midpoint of these two walls.
            est_x_offset = (x_left_wall + x_right_wall) / 2.0;
            x_offset_determined = true;
            ROS_DEBUG("Pose Est: 2 X-walls: L=%.2f, R=%.2f -> est_X=%.2f", x_left_wall, x_right_wall, est_x_offset);
        }
    } else if (x_wall_coords.size() == 1) {
        // Only one vertical wall found. Assume it's one side of the cell.
        double x_wall = x_wall_coords[0];
        if (x_wall < 0) { // Assumed left wall.
            est_x_offset = x_wall + CELL_HALF_DIMENSION;
        } else { // Assumed right wall.
            est_x_offset = x_wall - CELL_HALF_DIMENSION;
        }
        x_offset_determined = true;
        ROS_DEBUG("Pose Est: 1 X-wall: Xw=%.2f -> est_X=%.2f", x_wall, est_x_offset);
    }

    if (y_wall_coords.size() >= 2) {
        // Two horizontal walls found.
        std::sort(y_wall_coords.begin(), y_wall_coords.end());
        double y_bottom_wall = y_wall_coords.front(); // Should be negative (e.g., -0.4m)
        double y_top_wall = y_wall_coords.back();    // Should be positive (e.g., +0.4m)
        if (std::fabs((y_top_wall - y_bottom_wall) - CELL_DIMENSION) < CELL_DIMENSION * 0.35) {
            // Robot's Y position is the midpoint.
            est_y_offset = (y_bottom_wall + y_top_wall) / 2.0;
            y_offset_determined = true;
            ROS_DEBUG("Pose Est: 2 Y-walls: B=%.2f, T=%.2f -> est_Y=%.2f", y_bottom_wall, y_top_wall, est_y_offset);
        }
    } else if (y_wall_coords.size() == 1) {
        // Only one horizontal wall found.
        double y_wall = y_wall_coords[0];
        if (y_wall < 0) { // Assumed bottom wall.
            est_y_offset = y_wall + CELL_HALF_DIMENSION;
        } else { // Assumed top wall.
            est_y_offset = y_wall - CELL_HALF_DIMENSION;
        }
        y_offset_determined = true;
        ROS_DEBUG("Pose Est: 1 Y-wall: Yw=%.2f -> est_Y=%.2f", y_wall, est_y_offset);
    }

    // If only one dimension was robustly found, we might be less confident.
    // For now, we require at least one wall for each dimension or two for one dimension.
    // The est_angle_err is always determined if at least one line is found.
    // A more sophisticated approach would return a confidence score.
    if (!x_offset_determined && !y_offset_determined) {
        ROS_WARN("Pose Estimation: Could not determine X or Y offset.");
        return false; // Not enough information for position.
    }
    if (!x_offset_determined) {
        ROS_WARN("Pose Estimation: X offset not determined, assuming 0.");
        est_x_offset = 0.0; // Fallback if one dimension is missing.
    }
    if (!y_offset_determined) {
        ROS_WARN("Pose Estimation: Y offset not determined, assuming 0.");
        est_y_offset = 0.0; // Fallback.
    }

    ROS_INFO("Pose Estimation Result: X_off=%.3f, Y_off=%.3f, Angle_err=%.3f (rad)", est_x_offset, est_y_offset, est_angle_err);
    return true; // Pose estimation considered successful if we got this far.
}

// --- LaserScan Callback ---
/**
 * @brief Callback function for processing LaserScan messages.
 * Converts LiDAR points to the robot's base_link frame, accounting for LiDAR offset.
 * @param scan The received LaserScan message.
 */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    current_scan_points_robot_frame.clear();
    double current_angle_in_lidar_frame = scan->angle_min; // Start angle of the scan.

    for (float range : scan->ranges) {
        // Filter invalid ranges (NaN, Inf) and unreasonable distances.
        // Max range filter considers cell dimension, LiDAR offset, and some buffer.
        if (!std::isnan(range) && !std::isinf(range) &&
            range > scan->range_min && range < (CELL_DIMENSION + LIDAR_OFFSET_X + 0.20) ) {

            // Point in LiDAR's own coordinate frame.
            double x_lidar = range * std::cos(current_angle_in_lidar_frame);
            double y_lidar = range * std::sin(current_angle_in_lidar_frame);

            // Transform point from LiDAR frame to robot's base_link frame.
            // Assumes LiDAR is mounted LIDAR_OFFSET_X along the robot's X-axis,
            // and LiDAR's Y-axis is parallel to robot's Y-axis.
            Point p_robot_frame;
            p_robot_frame.x = x_lidar + LIDAR_OFFSET_X; // Add offset along X.
            p_robot_frame.y = y_lidar;                 // Y remains the same under this assumption.
            current_scan_points_robot_frame.push_back(p_robot_frame);
        }
        current_angle_in_lidar_frame += scan->angle_increment; // Increment for the next scan point.
    }
    scan_received = true; // Set flag indicating a new scan is processed.
    ROS_DEBUG("Scan Callback: Processed %zu points into robot frame.", current_scan_points_robot_frame.size());
}

// --- Control and State Logic ---

/**
 * @brief Publishes a zero-velocity command to stop the robot.
 */
void stopRobot() {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub.publish(stop_cmd);
    ROS_INFO("Robot STOP command issued.");
}

/**
 * @brief Executes control commands based on the current state and estimated pose errors.
 */
void execute_control() {
    geometry_msgs::Twist cmd_vel_msg;

    if (current_state == MOVING_TO_CENTER) {
        // Calculate distance to the target center point.
        double distance_to_target = std::sqrt(estimated_x_offset_global * estimated_x_offset_global +
                                            estimated_y_offset_global * estimated_y_offset_global);

        if (distance_to_target > DISTANCE_TOLERANCE) {
            // Calculate angle to the target point in the robot's frame.
            // This is the direction the robot needs to head towards.
            double angle_to_target_point_robot_frame = std::atan2(estimated_y_offset_global, estimated_x_offset_global);

            // Proportional control for linear speed.
            cmd_vel_msg.linear.x = KP_LINEAR * distance_to_target;
            // Cap linear speed.
            cmd_vel_msg.linear.x = std::min(cmd_vel_msg.linear.x, MAX_LINEAR_SPEED);

            // If the angle to the target point is large, prioritize turning towards it.
            // Reduce linear speed when making sharp turns.
            if (std::fabs(angle_to_target_point_robot_frame) > ANGLE_TOLERANCE * 2.0) { // If error is more than twice angle tolerance
                 cmd_vel_msg.linear.x *= std::max(0.0, 1.0 - std::fabs(angle_to_target_point_robot_frame) / (M_PI_2) );
            }

            // Proportional control for angular speed to face the target point.
            cmd_vel_msg.angular.z = KP_ANGULAR * angle_to_target_point_robot_frame;
            // Cap angular speed.
            cmd_vel_msg.angular.z = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, cmd_vel_msg.angular.z));

            ROS_INFO("State: MOVING_TO_CENTER. TargetDist=%.2fm, TargetAngle=%.2frad -> LinVel=%.2fm/s, AngVel=%.2frad/s",
                     distance_to_target, angle_to_target_point_robot_frame, cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        } else {
            // Robot is close enough to the center.
            ROS_INFO("State: MOVING_TO_CENTER. Reached center (dist %.3fm <= %.3fm). -> ALIGNING_ANGLE.", distance_to_target, DISTANCE_TOLERANCE);
            stopRobot();
            current_state = ALIGNING_ANGLE;
            ros::Duration(0.2).sleep(); // Short pause before aligning angle.
        }
    } else if (current_state == ALIGNING_ANGLE) {
        // Aligning the robot's orientation with the cell's X-axis.
        if (std::fabs(estimated_angle_error_global) > ANGLE_TOLERANCE) {
            cmd_vel_msg.linear.x = 0.0; // No linear movement during angle alignment.
            // Proportional control for angular speed.
            cmd_vel_msg.angular.z = KP_ANGULAR * estimated_angle_error_global;
            // Cap angular speed.
            cmd_vel_msg.angular.z = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, cmd_vel_msg.angular.z));
            ROS_INFO("State: ALIGNING_ANGLE. AngleError=%.2frad -> AngVel=%.2frad/s", estimated_angle_error_global, cmd_vel_msg.angular.z);
        } else {
            // Robot's orientation is aligned.
            ROS_INFO("State: ALIGNING_ANGLE. Aligned angle (err %.3frad <= %.3frad). -> FINISHED.", estimated_angle_error_global, ANGLE_TOLERANCE);
            stopRobot();
            current_state = FINISHED;
        }
    } else if (current_state == EXPLORATORY_TURN) {
        // If pose estimation fails repeatedly, perform a slow turn to get new sensor views.
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = MAX_ANGULAR_SPEED * 0.5; // Turn at half max angular speed.
        ROS_INFO("State: EXPLORATORY_TURN. Turning slowly to find walls.");
    }

    cmd_vel_pub.publish(cmd_vel_msg); // Publish the calculated velocity command.
}

// --- Main Program Loop ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "align_improved_node"); // Initialize ROS node.
    ros::NodeHandle nh; // Node handle for interacting with ROS system.

    // Advertise publisher for /cmd_vel topic.
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Subscribe to /scan_filtered topic for laser scan data.
    ros::Subscriber laser_sub = nh.subscribe("/scan_filtered", 10, laserScanCallback);

    ros::Rate loop_rate(10); // Set control loop frequency to 10 Hz.
    ROS_INFO("Improved Align node started. Cell: %.2fm x %.2fm. LiDAR Offset X: %.2fm. Waiting for laser scan...", CELL_DIMENSION, CELL_DIMENSION, LIDAR_OFFSET_X);

    ros::Time exploratory_turn_start_time; // Timer for exploratory turn duration.

    while (ros::ok() && current_state != FINISHED) {
        ros::spinOnce(); // Process incoming messages (e.g., laser scans).

        // Wait for first scan unless in exploratory turn.
        if (!scan_received && current_state != EXPLORATORY_TURN && current_state != INITIALIZING) {
            ROS_DEBUG_THROTTLE(1.0, "MainLoop: No scan received yet, waiting...");
            loop_rate.sleep();
            continue;
        }

        // Main state machine logic.
        switch (current_state) {
            case INITIALIZING:
                ROS_INFO("State: INITIALIZING -> FINDING_WALLS");
                current_state = FINDING_WALLS;
                consecutive_pose_failures = 0;
                break;

            case FINDING_WALLS:
                ROS_INFO_THROTTLE(1.0, "State: FINDING_WALLS. Attempting RANSAC.");
                if (current_scan_points_robot_frame.empty()) {
                    ROS_WARN_THROTTLE(1.0, "FINDING_WALLS: No scan points available yet.");
                    break; // Wait for scan data.
                }
                {
                    std::vector<Line> lines = find_lines_ransac(current_scan_points_robot_frame);
                    ROS_INFO("FINDING_WALLS: RANSAC found %zu lines.", lines.size());
                    // Need at least one line to attempt pose estimation. More is better.
                    if (lines.size() >= 1) {
                        ROS_INFO("FINDING_WALLS: Found lines. -> ESTIMATING_POSE");
                        current_state = ESTIMATING_POSE;
                        // Pose estimation will use these lines in the next state's iteration.
                    } else {
                        ROS_WARN("FINDING_WALLS: Not enough lines detected by RANSAC.");
                        consecutive_pose_failures++;
                        if (consecutive_pose_failures >= MAX_CONSECUTIVE_POSE_FAILURES) {
                            ROS_WARN("FINDING_WALLS: Max RANSAC failures. -> EXPLORATORY_TURN");
                            current_state = EXPLORATORY_TURN;
                            exploratory_turn_start_time = ros::Time::now();
                            stopRobot();
                        } else {
                            stopRobot(); // Stop and wait for more scans / better view.
                        }
                    }
                }
                break;

            case ESTIMATING_POSE:
                ROS_INFO_THROTTLE(1.0, "State: ESTIMATING_POSE");
                if (current_scan_points_robot_frame.empty()) {
                    ROS_WARN("ESTIMATING_POSE: No scan points. -> FINDING_WALLS");
                    current_state = FINDING_WALLS;
                    consecutive_pose_failures++;
                    break;
                }
                {
                    // Re-run RANSAC for fresh lines before pose estimation.
                    std::vector<Line> lines = find_lines_ransac(current_scan_points_robot_frame);
                     if (lines.empty()) {
                        ROS_WARN("ESTIMATING_POSE: No lines from RANSAC. -> FINDING_WALLS");
                        current_state = FINDING_WALLS;
                        consecutive_pose_failures++;
                        break;
                    }
                    pose_successfully_estimated = perform_pose_estimation(lines,
                                                                        estimated_x_offset_global,
                                                                        estimated_y_offset_global,
                                                                        estimated_angle_error_global);
                    if (pose_successfully_estimated) {
                        ROS_INFO("ESTIMATING_POSE: Success. Est Pose: X_off=%.2f, Y_off=%.2f, Angle_err=%.2f. -> MOVING_TO_CENTER",
                                estimated_x_offset_global, estimated_y_offset_global, estimated_angle_error_global);
                        current_state = MOVING_TO_CENTER;
                        consecutive_pose_failures = 0; // Reset failure counter.
                        execute_control(); // Issue first control command for MOVING_TO_CENTER.
                    } else {
                        ROS_WARN("ESTIMATING_POSE: Pose estimation failed.");
                        consecutive_pose_failures++;
                        if (consecutive_pose_failures >= MAX_CONSECUTIVE_POSE_FAILURES) {
                            ROS_WARN("ESTIMATING_POSE: Max pose estimation failures. -> EXPLORATORY_TURN");
                            current_state = EXPLORATORY_TURN;
                            exploratory_turn_start_time = ros::Time::now();
                            stopRobot();
                        } else {
                             ROS_INFO("ESTIMATING_POSE: Failure. -> FINDING_WALLS to retry.");
                             current_state = FINDING_WALLS;
                             stopRobot();
                        }
                    }
                }
                break;

            case MOVING_TO_CENTER:
            case ALIGNING_ANGLE: // Both states continuously re-estimate pose and control.
                ROS_INFO_THROTTLE(0.5, "State: %s. Re-estimating pose and controlling.",
                                (current_state == MOVING_TO_CENTER ? "MOVING_TO_CENTER" : "ALIGNING_ANGLE"));
                if (current_scan_points_robot_frame.empty()) {
                    ROS_WARN("State %s: No scan points. -> FINDING_WALLS",
                             (current_state == MOVING_TO_CENTER ? "MOVING_TO_CENTER" : "ALIGNING_ANGLE"));
                    current_state = FINDING_WALLS; // Lost track of environment.
                    stopRobot();
                    break;
                }
                {
                    std::vector<Line> lines = find_lines_ransac(current_scan_points_robot_frame);
                    if (lines.empty()) {
                        ROS_WARN("State %s: Lost lines during RANSAC. -> FINDING_WALLS",
                                 (current_state == MOVING_TO_CENTER ? "MOVING_TO_CENTER" : "ALIGNING_ANGLE"));
                        current_state = FINDING_WALLS;
                        stopRobot();
                        break;
                    }
                    pose_successfully_estimated = perform_pose_estimation(lines,
                                                                        estimated_x_offset_global,
                                                                        estimated_y_offset_global,
                                                                        estimated_angle_error_global);
                    if (pose_successfully_estimated) {
                        execute_control(); // Continue controlling based on new pose.
                    } else {
                        ROS_WARN("State %s: Lost pose during movement/alignment. -> FINDING_WALLS",
                                 (current_state == MOVING_TO_CENTER ? "MOVING_TO_CENTER" : "ALIGNING_ANGLE"));
                        current_state = FINDING_WALLS;
                        stopRobot();
                    }
                }
                break;

            case EXPLORATORY_TURN:
                ROS_INFO_THROTTLE(1.0, "State: EXPLORATORY_TURN. Executing turn.");
                execute_control(); // Publishes the turn command.
                // Check if exploratory turn duration has elapsed.
                if (ros::Time::now() - exploratory_turn_start_time > ros::Duration(5.0)) { // Turn for 5 seconds.
                    ROS_INFO("EXPLORATORY_TURN: Turn finished. -> FINDING_WALLS");
                    current_state = FINDING_WALLS;
                    consecutive_pose_failures = 0; // Reset failure counter before retrying.
                    stopRobot();
                }
                break;

            case FINISHED:
                // Alignment complete, do nothing further in the loop.
                ROS_INFO_ONCE("State: FINISHED. Alignment process complete.");
                break;
        }

        scan_received = false; // Reset scan flag for the next loop iteration, ensuring fresh scan is used.
        loop_rate.sleep();     // Maintain the desired loop frequency.
    }

    ROS_INFO("Alignment node shutting down.");
    stopRobot(); // Ensure robot is stopped when node exits.
    return 0;
}
/********* END MODIFIED align_improved_commented.cpp *********/
