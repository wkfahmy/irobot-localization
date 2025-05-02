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
const float WHEEL_RADIUS = 0.0325; // wheel radius in meters

struct Line {
    float a, b, c; // ax + by + c = 0
    std::vector<int> inliers;
};

Line ransacLineFit(const std::vector<float>& x, const std::vector<float>& y, std::vector<bool>& used, float threshold = 0.05, int iterations = 100) {
    int best_inliers = 0;
    Line best_line = {0, 0, 0, {}};
    size_t N = x.size();

    for (int i = 0; i < iterations; ++i) {
        int i1, i2;
        do { i1 = rand() % N; } while (used[i1]);
        do { i2 = rand() % N; } while (i2 == i1 || used[i2]);

        float x1 = x[i1], y1 = y[i1], x2 = x[i2], y2 = y[i2];  // two random points
        //calculate line parameters
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float norm = sqrt(a * a + b * b);

        if (norm == 0) continue;

        int count = 0;
        std::vector<int> current_inliers;
        // count inliers
        for (int j = 0; j < N; ++j) {
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
        lines.push_back(line);
        publishLineMarker(line, x, y, line_id++);
    }

    if (lines.empty()) return; // no lines found

    // Choose line closest to robot (min |c|)
    Line best = *std::min_element(lines.begin(), lines.end(), [](const Line& l1, const Line& l2) {      //shows the line with the smallest c as the best line
        return fabs(l1.c) < fabs(l2.c);
    });

    float distance = fabs(best.c); // distance to wall
    float angle = atan2(best.b, best.a) + M_PI_2; // robot should face perpendicular  (angle of the line + 90 degrees so robot is facing the normal of the line)

    float target_distance = 0.4; // center of 80cm cell
    float linear_error = target_distance - distance; // distance to the center
    float angular_error = angle;  // angle to rotate to face the line

    create_fundamentals::DiffDrive srv;
    float Kp_lin = 2.0;
    float Kp_ang = 2.0;
    float v = Kp_lin * linear_error;
    float w = Kp_ang * angular_error;

    float v_left = v - w * wheel_separation / 1.0;
    float v_right = v + w * wheel_separation / 2.0;

    float omega_left = v_left / WHEEL_RADIUS;
    float omega_right = v_right / WHEEL_RADIUS;
    
    if (fabs(linear_error) < 0.02 && fabs(angular_error) < 0.02) {
        omega_left = 0; omega_right = 0;
    }
    
    srv.request.left = omega_left;
    srv.request.right = omega_right;
    diff_drive_client->call(srv);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("scan_filtered", 1, scanCallback);
    ros::ServiceClient client = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/lines", 10);

    ros::spin();
    return 0;
} 