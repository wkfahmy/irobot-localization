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

bool processing_done = false; // Flag to indicate if processing is done

float min_distance;
int min_index;
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


typedef struct {
    double x;
    double y;
} Point;

struct Line {
    float a_dash, b_dash, c_dash; // ax + by + c = 0
    std::vector<int> inliers;
};

Line* ransacLineFit(const std::vector<Point>& points, std::vector<bool>& used, float threshold = 0.05, int iterations = 100) {
    int best_inliers = 0;
    Line best_line = {0, 0, 0, {}};
    size_t N = points.size();  // number of points

    for (int i = 0; i < iterations; ++i) {
        int i1, i2;
        int count = 0;
        do {
            i1 = rand() % N;
            count++;
            if (count > points.size() / 2)
                return nullptr;
        } while (used[i1]);
        count = 0;
        do { i2 = rand() % N;
            count++;
            if (count > points.size() / 2)
                return nullptr;
        } while (i2 == i1 || used[i2]);

        float x1 = points[i1].x, y1 = points[i1].y, x2 = points[i2].x, y2 = points[i2].y;  // two random points
        //calculate line parameters
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float norm = sqrt(a * a + b * b);

        if (norm == 0) continue;

        count = 0;
        std::vector<int> current_inliers;
        // count inliers
        for (int j = 0; j < N; ++j) {
            if (used[j]) continue;
            float dist = fabs(a * points[j].x + b * points[j].y + c) / norm;
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
    return *best_line;
}

const double fov_deg = 240.0;
const double R     = 0.17;
const double dL    = 0.16;



std::vector<Point> obstaclePoints;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    if (processing_done) return;  // Ignore further callbacks

    min_distance = std::numeric_limits<float>::infinity();
    min_index = -1;

    //convert lidar polar coordinates to cartesian
    for (int i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (!isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            double theta = msg->angle_min + i * msg->angle_increment;
            double sin_t = sin(theta);
            double cos_t = cos(theta);

            obstaclePoints[i].x = dL + r * cos_t;
            obstaclePoints[i].y = r * sin_t;

            // Check if the value is less than the current minimum distance
            if (r < min_distance) {
                min_distance = r;
                min_index = i;
            }
        } else {
            obstaclePoints[i].x = std::numeric_limits<double>::infinity();
            obstaclePoints[i].y = std::numeric_limits<double>::infinity();
        }
    }

    std::vector<bool> used(obstaclePoints.size(), false);   // to track used points
    std::vector<Line> lines; // to store detected lines by RANSAC
    int line_id = 0;

    while (true) {
        // RANSAC to find lines
        Line* line = ransacLineFit(obstaclePoints, used);
        if (line == nullptr || line->inliers.size() < MIN_INLIERS) break;  // stop if not enough inliers
        lines.push_back(*line);
    }

    ROS_INFO("Found %zu lines", lines.size());

    if (lines.empty()) return; // no lines found

    // Find the pair of perpendicular lines closest to the LiDAR (origin)
    std::vector<Line> perpendicular_lines;
    float min_total_distance = std::numeric_limits<float>::max();
    Point intersectionPoint;
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {

            float dot = lines[i].a_dash * lines[j].a_dash + lines[i].b_dash * lines[j].b_dash;
            if (fabs(dot) < 0.1) { // approx. perpendicular
                ROS_INFO("Found perpendicular lines");
                float a = - lines[i].a_dash / lines[i].b_dash;
                float c = - lines[i].c_dash / lines[i].b_dash;
                float b = - lines[j].a_dash / lines[j].b_dash;
                float d = - lines[j].c_dash / lines[j].b_dash;

                float x = (d - c) / (a - b);
                float y = a * x + c;

                float distance = sqrt(x * x + y * y);
                if (distance < min_total_distance && distance < 0.80) {
                    min_total_distance = distance;
                    perpendicular_lines.clear();
                    intersectionPoint.x = x;
                    intersectionPoint.y = y;
                    perpendicular_lines.push_back(lines[i]);
                    perpendicular_lines.push_back(lines[j]);
                }
            }
        }
    }


    if (!perpendicular_lines.empty()) {
        // Calculate distances and angles
        float x = (intersectionPoint.x > 0.4 ? intersectionPoint.x - 0.4 : intersectionPoint.x + 0.4);
        float y = (intersectionPoint.y > 0.4 ? intersectionPoint.y - 0.4 : intersectionPoint.y + 0.4);

        float angle = atan2(intersectionPoint.x - 0.4, (intersectionPoint.y - 0.4));
        float distance = sqrt(x * x + y * y);
        // Align with the first line
        spinInPlace(*diff_drive_client, angle, 3.0);
        moveLinear(*diff_drive_client, -distance - 0.4, 3.0);

        processing_done = true;  // Mark that we're done
        ros::shutdown();         // Exit the node cleanly
    }
    else {
        spinInPlace(*diff_drive_client, M_PI, 3.0); // Turn 180 degrees if no lines found
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, scanCallback);
    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

    //marker_pub = n.advertise<visualization_msgs::Marker>("/lines", 10);

     // Keep spinning until we're done
    ros::Rate rate(10); // 10 Hz loop
    while (ros::ok() && !processing_done) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
