#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <optional>
#include <ctime>
#include <numeric>
#include <random>
#include <algorithm>

ros::ServiceClient* diff_drive_client;
ros::Publisher marker_pub;
float wheel_separation = 0.266;
const int MIN_INLIERS = 50; // minimum inliers to consider a line valid
const int MAX_ITERATIONS = 200; // maximum iterations for RANSAC
const float DISTANCE_THRESHOLD = 0.01; // distance threshold for inliers
constexpr double WHEEL_RADIUS_M   = 0.0325;
constexpr double TRACK_WIDTH_M    = 0.263;

int align_passes = 2;

const double fov_deg = 240.0;
const double R     = 0.17;
const double dL    = 0.16;


bool processing_done = false;

float min_distance;
int min_index;
int start_index;
int end_index;

void spinInPlace(ros::ServiceClient& diffDrive,
                 double angle_rad,
                 double wheel_speed_rad_s)
{
    const double omega_robot = 2.0 * WHEEL_RADIUS_M * wheel_speed_rad_s / TRACK_WIDTH_M; // rad/s
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



std::vector<Point> obstaclePoints;
std::vector<AngularPoint> angledObstaclePoints;

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    double angle;
    double distance;
} AngularPoint;

//considering lidar sensor as the origin

struct Line {
    float a_dash, b_dash, c_dash; // ax + by + c = 0
    std::vector<int> inliers;
};

Line fitLineToInliers(const std::vector<Point>& points, const std::vector<int>& inliers) {
    size_t N = inliers.size();
    if (N < 2) return {0, 0, 0, {}};

    double sum_x = 0, sum_y = 0;
    for (int idx : inliers) {
        sum_x += points[idx].x;
        sum_y += points[idx].y;
    }
    double mean_x = sum_x / N;
    double mean_y = sum_y / N;

    double Sxx = 0, Sxy = 0, Syy = 0;
    for (int idx : inliers) {
        double dx = points[idx].x - mean_x;
        double dy = points[idx].y - mean_y;
        Sxx += dx * dx;
        Sxy += dx * dy;
        Syy += dy * dy;
    }

    double theta = 0.5 * std::atan2(2 * Sxy, Sxx - Syy);
    double a = std::sin(theta);
    double b = -std::cos(theta);
    double c = -(a * mean_x + b * mean_y);

    return {static_cast<float>(a), static_cast<float>(b), static_cast<float>(c), inliers};
}

bool ransacLineFit(const std::vector<Point>& points, std::vector<bool>& used, Line& out_line) {
    const size_t N = points.size();
    if (N < 2) return false;

    Line best_line = {0, 0, 0, {}};
    int best_inliers = 0;

    srand(static_cast<unsigned>(time(0)));

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        int i1 = rand() % N;
        int i2 = rand() % N;

        if (i1 == i2 || used[i1] || used[i2]) continue;

        float x1 = points[i1].x, y1 = points[i1].y;
        float x2 = points[i2].x, y2 = points[i2].y;
        float dx = x2 - x1;
        float dy = y2 - y1;

        float norm = std::sqrt(dx * dx + dy * dy);
        if (norm < 1e-3) continue;

        float a = dy / norm;
        float b = -dx / norm;
        float c = -(a * x1 + b * y1);

        if (std::fabs(c) < R) continue;

        std::vector<int> current_inliers;
        for (size_t j = 0; j < N; ++j) {
            if (used[j]) continue;
            float x = points[j].x;
            float y = points[j].y;
            float dist = std::fabs(a * x + b * y + c);
            if (dist < DISTANCE_THRESHOLD)
                current_inliers.push_back(j);
        }

        if ((int)current_inliers.size() > best_inliers) {
            best_inliers = current_inliers.size();
            best_line = {a, b, c, current_inliers};

            if (best_inliers > 0.8 * N) break;
        }
    }

    if (best_inliers < MIN_INLIERS) return false;

    out_line = fitLineToInliers(points, best_line.inliers);

    for (int idx : out_line.inliers)
        used[idx] = true;

    return true;
}

bool insideRobot(Point& p) {
  return p.x * p.x + p.y * p.y < R * R;
}

std::vector<Point> obstaclePoints;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (processing_done) return;  // Ignore further callbacks

    if (obstaclePoints.size() != msg->ranges.size()) {
        obstaclePoints.resize(msg->ranges.size());
    }

    for (int i = 0; i < msg->ranges.size(); ++i) {
		float r = msg->ranges[i];
		if (i < 16 ||  i > msg->ranges.size() - 16 || isnan(r) || r < msg->range_min || r > msg->range_max) {
			continue;
		}
        double theta = msg->angle_min + i * msg->angle_increment;
        double sin_t = sin(theta);
        double cos_t = cos(theta);

		Point obstaclePoint;
		obstaclePoint.x = r * cos_t + dL;
		obstaclePoint.y = r * sin_t;

		if (insideRobot(obstaclePoint)) continue;
        obstaclePoints.push_back(obstaclePoint);
    }

    std::vector<Line> lines;

	std::vector<bool> used(obstaclePoints.size(), false);
    while (true) {
        // RANSAC to find lines
        Line line;
        if (!ransacLineFit(obstaclePoints, used, line) || line.inliers.size() < MIN_INLIERS) break;
        lines.push_back(line);
    }

    if (lines.empty()) return; // no lines found

    ROS_INFO("Found %zu lines", lines.size());
	for (size_t i = 0; i < lines.size(); ++i) {
		ROS_INFO("Line %ld y = %f x + %f", i, - lines[i].a_dash / lines[i].b_dash, - lines[i].c_dash / lines[i].b_dash);
	}

    std::vector<Line> perpendicular_lines;
    float min_total_distance = std::numeric_limits<float>::max();
    Point intersectionPoint;
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {

 			float A1 = lines[i].a_dash;
			float B1 = lines[i].b_dash;
			float C1 = lines[i].c_dash;
			float A2 = lines[j].a_dash;
			float B2 = lines[j].b_dash;
			float C2 = lines[j].c_dash;

			float dot = A1 * A2 + B1 * B2;
			if (fabs(dot) > 0.2) continue;

			float det = A1 * B2 - A2 * B1;
			if (fabs(det) > 1e-6) {
            ROS_INFO("Found perpendicular lines");
    			float x = (B1 * C2 - B2 * C1) / det;
    			float y = (A2 * C1 - A1 * C2) / det;
				float distance = std::sqrt(x * x + y * y);

				if (min_total_distance > distance) {
                    perpendicular_lines.clear();
					min_total_distance = distance;
                    intersectionPoint.x = x;
                    intersectionPoint.y = y;
                    perpendicular_lines.push_back(lines[i]);
                    perpendicular_lines.push_back(lines[j]);
                }
            }
        }
    }


    if (!perpendicular_lines.empty()) {
		ROS_INFO("Found an intersection point %f %f", intersectionPoint.x, intersectionPoint.y);
		ROS_INFO("Intersection (robot frame): x = %.3f, y = %.3f, distance = %.3f",
          intersectionPoint.x,
          intersectionPoint.y,
          sqrt(intersectionPoint.x * intersectionPoint.x + intersectionPoint.y * intersectionPoint.y));
		float diag_half_cell = 0.8 / std::sqrt(2);
		float dx = intersectionPoint.x;
		float dy = intersectionPoint.y;

		float norm = std::sqrt(dx*dx + dy*dy);
		float dir_x = -dx / norm;
		float dir_y = -dy / norm;

		float center_x = intersectionPoint.x + diag_half_cell * dir_x;
		float center_y = intersectionPoint.y + diag_half_cell * dir_y;

		float angle = atan2(center_y, center_x);
		float distance = sqrt(center_x * center_x + center_y * center_y);

		ROS_INFO("Direction to center: (%f, %f)", -dir_x, -dir_y);
		ROS_INFO("Computed center: (%f, %f), distance = %.3f", center_x, center_y, distance);

		spinInPlace(*diff_drive_client, angle, 3.0);
		moveLinear(*diff_drive_client, distance, 3.0);



		align_passes--;
		if (align_passes == 0) {
			float align_angle = atan2(perpendicular_lines[0].b_dash, perpendicular_lines[0].a_dash);
			float align_wall = align_angle - angle;

			spinInPlace(*diff_drive_client, align_wall, 3.0);

        	processing_done = true;
        	ros::shutdown();
		}
    }
    else {
        spinInPlace(*diff_drive_client, 2 * M_PI / 3, 3.0); // Turn 180 degrees if no lines found
    }
}

int main(int argc, char** argv) {
	srand(static_cast<unsigned>(time(0)));

    ros::init(argc, argv, "align_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, scanCallback);
    ros::ServiceClient client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diff_drive_client = &client;

    ros::Rate rate(20); // 20 Hz loop
    while (ros::ok() && !processing_done) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
