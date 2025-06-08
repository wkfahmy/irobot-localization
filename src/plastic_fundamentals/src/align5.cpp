#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <cmath>
#include <vector>
#include <cstdlib>

ros::ServiceClient* diff_drive_client;

float wheel_separation = 0.266;
const int MIN_INLIERS = 30; // minimum inliers to consider a line valid
const int MAX_ITERATIONS = 100; // maximum iterations for RANSAC
const float DISTANCE_THRESHOLD = 0.05; // distance threshold for inliers
const float DISTANCE_FROM_LIDAR = 0.28; // distance from the lidar to the wall
constexpr double WHEEL_RADIUS_M   = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M    = 0.263;    // 26.3 cm
int callback_count = 0; // Counter for the number of callbacks

bool processing_done = false; // Flag to indicate if processing is done
int start_index;
int end_index;

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M;
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
}

void moveLinear(ros::ServiceClient& diffDrive,
                double distance_m,
                double wheel_speed_rad_s)
{
    const double linear_speed = WHEEL_RADIUS_M * wheel_speed_rad_s;
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

struct Line {
    float a_dash, b_dash, c_dash; // ax + by + c = 0
    std::vector<int> inliers;
};


void correctNormalDirection(Line& line, const std::vector<float>& x, const std::vector<float>& y) {
    // Calculate centroid of inliers
    float cx = 0.0, cy = 0.0;
    for (int idx : line.inliers) {
        cx += x[idx];
        cy += y[idx];
    }
    cx /= line.inliers.size();
    cy /= line.inliers.size();

    // Vector from centroid to origin (LiDAR position)
    float vec_to_origin_x = -cx;
    float vec_to_origin_y = -cy;

    // Dot product between normal and centroid-to-origin vector
    float dot = line.a_dash * vec_to_origin_x + line.b_dash * vec_to_origin_y;

    // If dot product is positive, normal points TOWARD origin (flip it)
    if (dot > 0) {
        line.a_dash *= -1;
        line.b_dash *= -1;
        line.c_dash *= -1;
    }
}

Line ransacLineFit(const std::vector<float>& x, const std::vector<float>& y, std::vector<bool>& used, float threshold = 0.05, int iterations = 100) {
    ROS_INFO("currently in ransac");
    int best_inliers = 0;
    Line best_line = {0, 0, 0, {}};
    size_t N = x.size();  // number of points

    for (int i = 0; i < iterations; ++i) {
        // Collect unused indices
        std::vector<int> unused_indices;
        for (size_t j = 0; j < N; ++j) {
            if (!used[j]) unused_indices.push_back(j);
        }

        // If fewer than 2 points left, break early
        if (unused_indices.size() < 2) {
            ROS_INFO("Not enough points left for RANSAC, exiting");
            return best_line;
        }

        // Randomly pick two different unused indices
        int i1_idx = rand() % unused_indices.size();
        int i1 = unused_indices[i1_idx];
        int i2;
        do {
            i2 = unused_indices[rand() % unused_indices.size()];
        } while (i2 == i1);

        float x1 = x[i1], y1 = y[i1], x2 = x[i2], y2 = y[i2];  // two random points
        // Calculate line parameters
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float norm = sqrt(a * a + b * b);

        if (norm == 0) continue;

        int count = 0;
        std::vector<int> current_inliers;

        // Count inliers
        for (size_t j = 0; j < N; ++j) {
            if (used[j]) continue;
            float dist = fabs(a * x[j] + b * y[j] + c) / norm;
            if (dist < threshold) {
                count++;
                current_inliers.push_back(j);
            }
        }

        if (count > best_inliers) {
            best_inliers = count;
            best_line = {a / norm, b / norm, c / norm, current_inliers};
        }
    }

    for (int idx : best_line.inliers) used[idx] = true;   // mark inliers found as used
    ROS_INFO("found a line");
    return best_line;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    callback_count++;
    ros::Duration(0.5).sleep();

    if (processing_done) return;  // Ignore further callbacks

    ROS_INFO("Processing scan data...");

    std::vector<float> x, y;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (!isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment;
            x.push_back(r * cos(angle));
            y.push_back(r * sin(angle));
        }
    }

    std::vector<bool> used(x.size(), false);
    std::vector<Line> lines;

    while (true) {
        Line line = ransacLineFit(x, y, used);
        if (line.inliers.size() < MIN_INLIERS) break;
        correctNormalDirection(line, x, y);
        lines.push_back(line);
    }

    if (lines.empty()) return;

    std::vector<Line> perpendicular_lines;
    float min_total_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            float dot = lines[i].a_dash * lines[j].a_dash + lines[i].b_dash * lines[j].b_dash;
            if (fabs(dot) < 0.3) {
                float distance1 = fabs(lines[i].c_dash);
                float distance2 = fabs(lines[j].c_dash);
                float total_distance = distance1 + distance2;

                if (total_distance < min_total_distance) {
                    // Check if lines are connected
                    float min_point_distance = std::numeric_limits<float>::max();
                    for (int idx1 : lines[i].inliers) {
                        for (int idx2 : lines[j].inliers) {
                            float dx = x[idx1] - x[idx2];
                            float dy = y[idx1] - y[idx2];
                            float d = std::sqrt(dx * dx + dy * dy);
                            if (d < min_point_distance) min_point_distance = d;
                        }
                    }

                    if (min_point_distance < 0.1) { // threshold for "connected"
                        min_total_distance = total_distance;
                        perpendicular_lines.clear();
                        perpendicular_lines.push_back(lines[i]);
                        perpendicular_lines.push_back(lines[j]);
                    } else {
                        ROS_INFO("Lines are perpendicular but not connected (min_dist = %.2f)", min_point_distance);
                    }
                }
            }
        }
    }

    bool found_perpendicular = !perpendicular_lines.empty();

    if (found_perpendicular) {
        float distance1 = fabs(perpendicular_lines[0].c_dash);
        float distance2 = fabs(perpendicular_lines[1].c_dash);
        if (distance1 > 0.8 || distance2 > 0.8) {
            ROS_INFO("Distance to wall is too far, rotating 180 degrees");
            spinInPlace(*diff_drive_client, M_PI, 5.0);
            ros::Duration(0.5).sleep();
            return;
        }

        float angle = atan2(perpendicular_lines[0].b_dash, perpendicular_lines[0].a_dash);
        float center_distance1 = distance1 + 0.16 * cos(angle);
        float center_distance2 = distance2 - 0.16 * sin(angle);

        spinInPlace(*diff_drive_client, angle, 3.0);
        ros::Duration(0.5).sleep();
        moveLinear(*diff_drive_client, center_distance1 - 0.4, 3.0);

        float turn_angle = (perpendicular_lines[0].a_dash * perpendicular_lines[1].b_dash - 
                            perpendicular_lines[0].b_dash * perpendicular_lines[1].a_dash) > 0 ? -M_PI_2 : M_PI_2;
        spinInPlace(*diff_drive_client, turn_angle, 3.0);
        ros::Duration(0.5).sleep();

        moveLinear(*diff_drive_client, -(center_distance2 - 0.4), 3.0);
        if (callback_count > 2) {
            processing_done = true;
            ros::shutdown();
        }
    } else {
        ROS_INFO("No connected perpendicular lines found. Rotating 180 degrees...");
        spinInPlace(*diff_drive_client, M_PI, 5.0);
        ros::Duration(0.5).sleep();
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle n;
    ROS_INFO("Align node started");

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, scanCallback);
    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

     // Keep spinning until we're done
    ros::Rate rate(10);
    while (ros::ok() && !processing_done) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}