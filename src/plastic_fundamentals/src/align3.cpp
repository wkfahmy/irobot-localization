#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <cstdlib>

ros::ServiceClient* diff_drive_client;
ros::Publisher marker_pub;
float wheel_separation = 0.266;
const int MIN_INLIERS = 30; // minimum inliers to consider a line valid
const int MAX_ITERATIONS = 100; // maximum iterations for RANSAC
const float DISTANCE_THRESHOLD = 0.05; // distance threshold for inliers
const float DISTANCE_FROM_LIDAR = 0.28; // distance from the lidar to the wall
constexpr double WHEEL_RADIUS_M   = 0.0325;   // 6.5 cm / 2
constexpr double TRACK_WIDTH_M    = 0.263;    // 26.3 cm
int callback_count = 0; // Counter for the number of callbacks
int rotation_180 = 0; // Counter for the number of 180-degree rotations

bool processing_done = false; // Flag to indicate if processing is done

int start_index;
int end_index;

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    // 1. Work out how long we must turn
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M; // rad/s
    const double duration_s = std::fabs(angle_rad) / std::fabs(omega_robot);

    // 2. Build DiffDrive request (sign decides direction)
    create_fundamentals::DiffDrive srv;
    srv.request.right = (angle_rad >= 0.0 ? wheel_speed_rad_s : -wheel_speed_rad_s);
    srv.request.left = -(srv.request.right); // opposite direction for pure spin

    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send spin command!");

    ros::Duration(duration_s).sleep(); // 3. Wait while wheels spin

    // 4. Stop
    srv.request.left = 0.0;
    srv.request.right = 0.0;
    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send stop command!");
}

void moveLinear(ros::ServiceClient& diffDrive,
                double distance_m,
                double wheel_speed_rad_s)
{
    // 1. Work out how long we must move
    const double linear_speed = WHEEL_RADIUS_M * wheel_speed_rad_s; // m/s
    const double duration_s = std::fabs(distance_m) / std::fabs(linear_speed);

    // 2. Build DiffDrive request (sign decides direction)
    create_fundamentals::DiffDrive srv;
    srv.request.left = (distance_m >= 0.0 ? wheel_speed_rad_s : -wheel_speed_rad_s);
    srv.request.right = srv.request.left; // same direction for linear motion

    if (!diffDrive.call(srv))
        ROS_ERROR("Failed to send move command!");

    ros::Duration(duration_s).sleep(); // 3. Wait while wheels move

    // 4. Stop
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



//considering lidar sensor as the origin


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
            ROS_INFO("Not enough points left for RANSAC");
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

void publishLineMarker(const Line& line, const std::vector<float>& x, const std::vector<float>& y, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lines";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    for (int idx : line.inliers) {
        if (p1.x == 0 && p1.y == 0) {
            p1.x = x[idx]; p1.y = y[idx];
        } else {
            p2.x = x[idx]; p2.y = y[idx];
        }
    }
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker_pub.publish(marker);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    callback_count++;

    if (processing_done) return;  // Ignore further callbacks

    ROS_INFO("Processing scan data...");


    std::vector<float> x, y;
    //convert lidar polar coordinates to cartesian
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (!isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment;
            x.push_back(r * cos(angle));
            y.push_back(r * sin(angle));
        }
    }


    std::vector<bool> used(x.size(), false);   // to track used points
    std::vector<Line> lines; // to store detected lines by RANSAC
    int line_id = 0;

    while (true) {
        // RANSAC to find lines
        Line line = ransacLineFit(x, y, used);
        if (line.inliers.size() < MIN_INLIERS) break;  // stop if not enough inliers
        correctNormalDirection(line, x, y); // Correct the normal direction
        lines.push_back(line);
        publishLineMarker(line, x, y, line_id++);
    }

    if (lines.empty()) return; // no lines found

    // Find the pair of perpendicular lines closest to the LiDAR (origin)
    std::vector<Line> perpendicular_lines;
    float min_total_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            float dot = lines[i].a_dash * lines[j].a_dash + lines[i].b_dash * lines[j].b_dash;
            if (fabs(dot) < 0.2) { // approx. perpendicular

                float distance1 = fabs(lines[i].c_dash);  // distance from origin to line 1
                float distance2 = fabs(lines[j].c_dash);  // distance from origin to line 2
                float total_distance = distance1 + distance2;

                if (total_distance < min_total_distance) {
                    min_total_distance = total_distance;
                    perpendicular_lines.clear();
                    perpendicular_lines.push_back(lines[i]);
                    perpendicular_lines.push_back(lines[j]);
                }
            }
        }
    }

    bool found_perpendicular = !perpendicular_lines.empty();

    if (found_perpendicular) {
        // Calculate distances and angles
        float distance1 = fabs(perpendicular_lines[0].c_dash);
        float distance2 = fabs(perpendicular_lines[1].c_dash);
        if(distance1 >= 0.8 || distance2 >= 0.8) {
            ROS_INFO("Distance to wall is too far, rotating 180 degrees");
            spinInPlace(*diff_drive_client, M_PI, 3.0); // Turn 180 degrees
            return;
        }
        float angle = atan2(perpendicular_lines[0].b_dash, perpendicular_lines[0].a_dash);

        // Adjust distances to center
        float center_distance1 = distance1 + 0.16 * cos(angle);
        float center_distance2 = distance2 - 0.16 * sin(angle);

        // Align with the first line
        spinInPlace(*diff_drive_client, angle, 3.0);

        ros::Duration(0.5).sleep(); // Wait for a moment
        moveLinear(*diff_drive_client, center_distance1 - 0.4, 3.0);

        // Determine turn direction and align with the second line
        float turn_angle = (perpendicular_lines[0].a_dash * perpendicular_lines[1].b_dash - 
                            perpendicular_lines[0].b_dash * perpendicular_lines[1].a_dash) > 0 ? -M_PI_2 : M_PI_2;
        spinInPlace(*diff_drive_client, turn_angle, 3.0);

        ros::Duration(0.5).sleep(); // Wait for a moment

        // Move to the center of the cell
        moveLinear(*diff_drive_client, -(center_distance2 - 0.4), 3.0);
            processing_done = true;  // Mark that we're done
            ros::shutdown();         // Exit the node cleanly
        }
    }
    else {
        spinInPlace(*diff_drive_client, M_PI, 3.0); // Turn 180 degrees if no lines found
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle n;
    ROS_INFO("Align node started");

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, scanCallback);
    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

    marker_pub = n.advertise<visualization_msgs::Marker>("/lines", 10);

     // Keep spinning until we're done
    ros::Rate rate(10); // 10 Hz loop
    while (ros::ok() && !processing_done) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
