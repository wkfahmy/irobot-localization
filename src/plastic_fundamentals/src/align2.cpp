#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <optional>

ros::ServiceClient* diff_drive_client;
ros::Publisher marker_pub;
float wheel_separation = 0.266;
const int MIN_INLIERS = 30; // minimum inliers to consider a line valid
const int MAX_ITERATIONS = 100; // maximum iterations for RANSAC
const float DISTANCE_THRESHOLD = 0.05; // distance threshold for inliers
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

Line ransacLineFit(const std::vector<Point>& points, std::vector<bool>& used, float threshold = 0.05, int iterations = 100) {
    int best_inliers = 0;
    Line best_line = {0, 0, 0, {}};
    size_t N = points.size();  // number of points

    for (int i = 0; i < 100; ++i) {
        int i1, i2;
        int count = 0;
        do {
            i1 = rand() % N;
            count++;
            if (count > points.size() / 2)
                return false;
        } while (used[i1]);

        float x1 = points[i1].x, y1 = points[i1].y, x2 = points[i2].x, y2 = points[i2].y;  // two random points
        //calculate line parameters
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float norm = sqrt(a * a + b * b);
        if (norm == 0) continue;

        count = 0;
        std::vector<int> current_inliers;
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

    if (best_inliers == 0) return false;

    for (int idx : best_line.inliers) used[idx] = true;
    out_line = best_line;
    return true;
}

struct PolarLine {
    double theta;
    double rho;
};

std::vector<Line> accurateRansacLineFit(const std::vector<Point>& points) {
	const size_t M = 10;
    const double dThetaThresh = 2.0 * M_PI/180.0;  // 2°
    const double dRhoThresh   = 0.01; // 1 cm
    const size_t minSupport   = 5;

	std::vector<PolarLine> samples;
    samples.reserve(M * 5);

	for (size_t run = 0; run < M; ++run) {
        std::vector<bool> used(points.size(), false);
        while (true) {
            Line L;
            if (!ransacLineFit(points, used, L)
                || L.inliers.size() < MIN_INLIERS)
                break;

            // convert to normalized (θ, ρ)
            double theta = std::atan2(L.b_dash, L.a_dash);
            if (theta < 0) theta += 2*M_PI;
            double rho = -L.c_dash;

            samples.push_back({theta, rho});
        }
	}

	if (samples.empty())
        return {};

    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < (int)samples.size(); ++i) {
        bool placed = false;
        for (auto& C : clusters) {
            auto& rep = samples[C[0]];
            // circular difference
            double dtheta = fabs(samples[i].theta - rep.theta);
            dtheta = fmod(dtheta, 2*M_PI);
            if (dtheta > M_PI) dtheta = 2*M_PI - dtheta;
            double drho = fabs(samples[i].rho - rep.rho);

            if (dtheta < dThetaThresh && drho < dRhoThresh) {
                C.push_back(i);
                placed = true;
                break;
            }
        }
        if (!placed) {
            clusters.push_back({i});
        }
    }

	const double dtheta_thresh = 5.0 * M_PI/180.0;  // 5°
	const double drho_thresh = 0.01;              // 1 cm

	std::vector<Line> result;
    result.reserve(clusters.size());
    for (auto& C : clusters) {
        if (C.size() < minSupport)
            continue;

        // circular‐mean of θ and arithmetic‐mean of ρ
        double sum_sin = 0, sum_cos = 0, sum_rho = 0;
        for (int idx : C) {
            sum_cos +=  cos(samples[idx].theta);
            sum_sin +=  sin(samples[idx].theta);
            sum_rho +=  samples[idx].rho;
        }
        double theta_avg = std::atan2(sum_sin, sum_cos);
        if (theta_avg < 0) theta_avg += 2*M_PI;
        double rho_avg = sum_rho / C.size();

        // reconstruct line in Hesse normal form
        Line avg;
        avg.a_dash =  std::cos(theta_avg);
        avg.b_dash =  std::sin(theta_avg);
        avg.c_dash = -rho_avg;
        avg.inliers.clear();

        // (Re‐classify inliers on the averaged model, optional)
        for (int i = 0; i < (int)points.size(); ++i) {
            double dist = fabs(avg.a_dash * points[i].x
                             + avg.b_dash * points[i].y
                             + avg.c_dash);
            if (dist < 0.05)
                avg.inliers.push_back(i);
        }

        if (avg.inliers.size() >= MIN_INLIERS)
            result.push_back(std::move(avg));
    }


	return result;
}

const double fov_deg = 240.0;
const double R     = 0.17;
const double dL    = 0.16;



std::vector<Point> obstaclePoints;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    if (processing_done) return;  // Ignore further callbacks

    if (obstaclePoints.size() != msg->ranges.size()) {
        obstaclePoints.resize(msg->ranges.size());
    }

    min_distance = std::numeric_limits<float>::infinity();
    min_index = -1;

    float rays = msg->ranges.size();

    if (obstaclePoints.size() != rays) {
        obstaclePoints.resize(rays);
    }

    float step = (fov_deg / rays) * M_PI / 180.0;
    float start_angle = (-fov_deg / 2.0) * M_PI / 180.0;

    std::vector<float> x, y;
    //convert lidar polar coordinates to cartesian
    for (size_t i = 0; i < msg->ranges.size(); ++i) {


        float distance = msg->ranges[i];
        if (!isnan(distance) && distance >= msg->range_min && distance <= msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment;
            double sin_t = sin(theta);
            double cos_t = cos(theta);

            obstaclePoints[i].x = r * cos_t + dL;
            obstaclePoints[i].y = r * sin_t;

            // Check if the value is less than the current minimum distance
            obstaclePoints[i].x = dL + distance * cos_t;
            obstaclePoints[i].y = distance * sin_t;

            angledObstaclePoints[i].angle = atan2(obstaclePoints[i].x, obstaclePoints[i].y) - M_PI / 2.0;
            angledObstaclePoints[i].distance = std::sqrt(pow(obstaclePoints[i].x,2) + pow(obstaclePoints[i].y,2));

            if (r < min_distance) {
                min_distance = r;
                min_index = i;
            }
        } else {
            obstaclePoints[i].x = std::numeric_limits<double>::infinity();
            obstaclePoints[i].y = std::numeric_limits<double>::infinity();

            angledObstaclePoints[i].angle = M_PI;
            angledObstaclePoints[i].distance = std::numeric_limits<double>::infinity();
        }
    }

    std::vector<bool> used(obstaclePoints.size(), false);   // to track used points
    std::vector<Line> lines = accurateRansacLineFit(obstaclePoints); // to store detected lines by RANSAC

    if (lines.empty()) return; // no lines found

    ROS_INFO("Found %zu lines", lines.size());
	ROS_INFO("Line 1 y = %f x + %f", - lines[0].a_dash / lines[0].b_dash, - lines[0].c_dash / lines[0].b_dash);
	ROS_INFO("Line 2 y = %f x + %f", - lines[1].a_dash / lines[0].b_dash, - lines[1].c_dash / lines[1].b_dash);
    // Find the pair of perpendicular lines closest to the LiDAR (origin)
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
		ROS_INFO("Angle: %f", angle);

		spinInPlace(*diff_drive_client, angle, 3.0);
		moveLinear(*diff_drive_client, distance, 3.0);

        processing_done = true;
        ros::shutdown();
    } else {
        spinInPlace(*diff_drive_client, 2 * M_PI / 3, 3.0); // Turn 180 degrees if no lines found
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
