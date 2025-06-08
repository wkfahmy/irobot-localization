#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <plastic_fundamentals/Point.h>

#include <cmath>
#include <vector>
#include <cstdlib>
#include <optional>

ros::ServiceClient* resetEncodersClient;
create_fundamentals::DiffDrive diffDriveSrv;
ros::ServiceClient* diffDriveClient;

ros::ServiceClient marker;

double WHEEL_RADIUS_M = 0.0325;
double TRACK_WIDTH_M = 0.263;

double RANSAC_DIST_THRESHOLD = 0.01;
int    RANSAC_MIN_INLIERS    = 40;
int    RANSAC_MAX_ITER       = 1000;

double leftTicks = 0;
double rightTicks = 0;

bool processing_done = false; // Flag to indicate if processing is done

bool centered = false;

bool moving = false;

float min_distance;
int min_index;
int start_index;
int end_index;

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;
}

double getRotationTicks(double angle_rad) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    double wheel_travel_distance = (TRACK_WIDTH_M * angle_rad) / 2;
    return (wheel_travel_distance / wheel_circumference) * ticks_per_revolution;
}

double getTranslationTicks(double distance) {
    double wheel_circumference = 2 * M_PI * WHEEL_RADIUS_M;
    double ticks_per_revolution = 6;
    return (distance / wheel_circumference) * ticks_per_revolution;
}


void rotate(double angle_rad, double speed) {
    moving = true;

    double ticks = getRotationTicks(fabs(angle_rad));


    resetEncoders();
    ros::spinOnce();

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    diffDriveSrv.request.left = -side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;


    while (ticks * 0.98 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 4)) {
            error_based_speed = (speed / 4);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = -side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();

    moving = false;
}


void translate(double distance, double speed) {
    moving = true;

    double ticks = getTranslationTicks(distance);

    resetEncoders();
    ros::spinOnce();

    double side = (distance > 0) ? 1.0 : -1.0;
    diffDriveSrv.request.left = side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;

    while (ticks * 0.99 > (abs(leftTicks) + abs(rightTicks)) / 2) {
        double correction = 0.0;

        double angular_error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2);
        double error_based_speed = speed * (angular_error / ticks);

        if (error_based_speed < (speed / 2)) {
            error_based_speed = (speed / 2);
        }

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
            diffDriveSrv.request.left = side * error_based_speed * (1 + correction / 2);
            diffDriveSrv.request.right = side * error_based_speed * (1 - correction / 2);
        }

        diffDriveClient->call(diffDriveSrv);

        last_left_ticks = leftTicks;
        last_right_ticks = rightTicks;

        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();

    moving = false;
}

struct Line {
    float a_dash, b_dash, c_dash; // ax + by + c = 0
    std::vector<int> inliers;
};

bool ransacLineFit(const std::vector<plastic_fundamentals::Point>& points, std::vector<bool>& used, Line& out_line,
                   double threshold = 0.03, int min_inliers = 15, int max_iter = 100) {
    int best_inliers = 0;
    Line best_line = {0, 0, 0, {}};
    size_t N = points.size();
    if (N < 2) return false;

    for (int iter = 0; iter < max_iter; ++iter)  {
        int i1, i2;

        int tries = 0;
        do { i1 = rand() % N; } while (used[i1] && ++tries < N*2);
        tries = 0;
        do { i2 = rand() % N; } while ((i2 == i1 || used[i2]) && ++tries < N*2);
        if (i1 == i2) continue;

        float x1 = points[i1].x, y1 = points[i1].y, x2 = points[i2].x, y2 = points[i2].y;
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float norm = sqrt(a * a + b * b);
        if (norm == 0) continue;

        std::vector<int> current_inliers;
        for (int j = 0; j < N; ++j) {
            if (used[j]) continue;
            float dist = fabs(a * points[j].x + b * points[j].y + c) / norm;
            if (dist < threshold) current_inliers.push_back(j);
        }

        if ((int)current_inliers.size() > best_inliers) {
            best_inliers = current_inliers.size();
            best_line = {a / norm, b / norm, c / norm, current_inliers};
        }
    }

    if (best_inliers < min_inliers) return false;

    // Refit line using all inliers (simple least-squares)
    float sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (int idx : best_line.inliers) {
        float x = points[idx].x, y = points[idx].y;
        sx += x; sy += y;
        sxx += x*x; syy += y*y; sxy += x*y;
    }
    int K = best_inliers;
    float mean_x = sx / K, mean_y = sy / K;
    float num = 2 * (sxy - sx*sy/K);
    float den = (sxx - sx*sx/K) - (syy - sy*sy/K);
    float theta = 0.5 * atan2(num, den);
    float a = sin(theta), b = -cos(theta);
    float c = -(a * mean_x + b * mean_y);

    best_line.a_dash = a; best_line.b_dash = b; best_line.c_dash = c;

    for (int idx : best_line.inliers) used[idx] = true;
    out_line = best_line;
    return true;
}

const double fov_deg = 240.0;
const double R     = 0.17;
const double dL    = 0.16;



std::vector<plastic_fundamentals::Point> obstaclePoints;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (centered || moving) return;
    if (processing_done) return;  // Ignore further callbacks

    if (obstaclePoints.size() != msg->ranges.size()) {
        obstaclePoints.resize(msg->ranges.size());
    }

    min_distance = std::numeric_limits<float>::infinity();
    min_index = -1;

    for (int i = 0; i < msg->ranges.size(); ++i) {
		if (i < 16 ||  i > msg->ranges.size() - 16) {
			obstaclePoints[i].x = std::numeric_limits<double>::infinity();
            obstaclePoints[i].y = std::numeric_limits<double>::infinity();
			continue;
		}
        float r = msg->ranges[i];
        if (!isnan(r) && r >= msg->range_min && r <= msg->range_max) {
            double theta = msg->angle_min + i * msg->angle_increment;
            double sin_t = sin(theta);
            double cos_t = cos(theta);

            obstaclePoints[i].x = r * cos_t + dL;
            obstaclePoints[i].y = r * sin_t;

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
        Line line;
        if (!ransacLineFit(obstaclePoints, used, line, RANSAC_DIST_THRESHOLD, RANSAC_MIN_INLIERS, RANSAC_MAX_ITER)) break;
        lines.push_back(line);
    }


    if (lines.empty()) return; // no lines found

    plastic_fundamentals::PublishMarker vis_srv;
    vis_srv.request.marker_type = "LineMarker";
    vis_srv.request.lines.clear();

    // For each line found by RANSAC
    for (const auto& l : lines) {
        if (l.inliers.size() < 2) continue;

        int idx1 = l.inliers.front();
        int idx2 = l.inliers.back();
        if (idx1 < 0 || idx1 >= obstaclePoints.size() || idx2 < 0 || idx2 >= obstaclePoints.size())
            continue;

        plastic_fundamentals::Line seg;
        seg.x1 = obstaclePoints[idx1].x;
        seg.y1 = obstaclePoints[idx1].y;
        seg.x2 = obstaclePoints[idx2].x;
        seg.y2 = obstaclePoints[idx2].y;

        vis_srv.request.lines.push_back(seg);
    }

    // Publish the detected lines
    if (!marker.call(vis_srv)) {
        //ROS_ERROR("Failed to call marker_service for found lines!");
    }

    // Find the pair of perpendicular lines closest to the LiDAR (origin)
    std::vector<Line> perpendicular_lines;
    float min_total_distance = 0.8;
    plastic_fundamentals::Point intersectionPoint;

    plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointMarker";

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
    				float x = (B1 * C2 - B2 * C1) / det;
    				float y = (A2 * C1 - A1 * C2) / det;
					float distance = std::sqrt(x * x + y * y);

					if (min_total_distance > distance) {
                    	perpendicular_lines.clear();
						min_total_distance = distance;
                    	intersectionPoint.x = x;
                    	intersectionPoint.y = y;
                    	intersectionPoint.w = 0.0;
                    	perpendicular_lines.push_back(lines[i]);
                    	perpendicular_lines.push_back(lines[j]);

					    srv.request.points.push_back(intersectionPoint);


                	}
            }
        }
    }

    plastic_fundamentals::Point centerPoint;
    centerPoint.x = 0.0;
    centerPoint.y = 0.0;
    srv.request.points.push_back(centerPoint);


    if (!marker.call(srv)) {
        ROS_ERROR("Failed to call service marker_service");
    }


    if (!perpendicular_lines.empty()) {
        auto compute_inward_normal = [](float a, float b, float px, float py, float robot_x, float robot_y) {
            float norm = std::sqrt(a * a + b * b);
            float na = a / norm;
            float nb = b / norm;

            float robot_vec_x = robot_x - px;
            float robot_vec_y = robot_y - py;

            float dot = na * robot_vec_x + nb * robot_vec_y;
            if (dot < 0) {
                na = -na;
                nb = -nb;
            }
            return std::make_pair(na, nb);
        };

        auto n1 = compute_inward_normal(perpendicular_lines[0].a_dash, perpendicular_lines[0].b_dash, intersectionPoint.x, intersectionPoint.y, 0, 0);
        float n1_x = n1.first;
        float n1_y = n1.second;
        auto n2 = compute_inward_normal(perpendicular_lines[1].a_dash, perpendicular_lines[1].b_dash, intersectionPoint.x, intersectionPoint.y, 0, 0);
        float n2_x = n2.first;
        float n2_y = n2.second;
        float bis_x = (n1_x + n2_x) / 2.0f;
        float bis_y = (n1_y + n2_y) / 2.0f;

        float center_x = intersectionPoint.x + 0.4 * (n1_x + n2_x);
        float center_y = intersectionPoint.y + 0.4 * (n1_y + n2_y);

        srv.request.marker_type = "LineMarker";
        plastic_fundamentals::Line bis_line;
        bis_line.x1 = intersectionPoint.x;
        bis_line.y1 = intersectionPoint.y;
        bis_line.x2 = center_x;
        bis_line.y2 = center_y;
        vis_srv.request.lines.push_back(bis_line);

        float angle = atan2(center_y, center_x);
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;

        float distance = sqrt(center_x * center_x + center_y * center_y);

        ROS_INFO("Computed center: (%f, %f), distance = %.3f", center_x, center_y, distance);

        rotate(angle, 5.0);
        translate(distance, 5.0);

        float wall_angle = atan2(n1_y, n1_x);

        float correction = wall_angle - angle;
        while (correction > M_PI) correction -= 2*M_PI;
        while (correction < -M_PI) correction += 2*M_PI;

        rotate(correction, 5.0);

        centered = true;
        resetEncoders();
    } else {
        rotate(2 * M_PI / 6, 5.0); // Turn 180 degrees if no lines found
        ros::Duration(0.5).sleep();
    }

    if (!marker.call(vis_srv)) {
        //ROS_ERROR("Failed to call marker_service for found lines!");
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "align_node");
    ros::NodeHandle nh;

    marker = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::Subscriber sub = nh.subscribe("/sensor_packet", 1, sensorCallback);

    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, scanCallback);

    ros::ServiceClient diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diffDriveClient = &diffDrive;

    ros::ServiceClient resetEncoders = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    ros::Rate rate(100);
    while (ros::ok() && !processing_done && !centered) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
