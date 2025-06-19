#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <create_fundamentals/DiffDrive.h>
#include "create_fundamentals/SensorPacket.h"
#include <create_fundamentals/ResetEncoders.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <plastic_fundamentals/Line.h>
#include <plastic_fundamentals/Grid.h>
#include <cmath>
#include <vector>

class RobotSimulator {
public:
    RobotSimulator(const ros::NodeHandle& nh, const plastic_fundamentals::Grid::ConstPtr& map_data,
                       double start_x, double start_y, double start_theta)
        : nh_(nh), map_data_(map_data), robot_x_(start_x), robot_y_(start_y), robot_theta_(start_theta + M_PI),
          left_speed_(0), right_speed_(0), encoder_left_(0), encoder_right_(0), line_segments_(preprocessMap(map_data_)) {

        sensor_packet_pub_ = nh_.advertise<create_fundamentals::SensorPacket>("sensor_packet", 1);
        lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
        diff_drive_service_ = nh_.advertiseService("diff_drive", &RobotSimulator::diffDriveCallback, this);
        reset_encoders_service_ = nh_.advertiseService("reset_encoders", &RobotSimulator::resetEncodersCallback, this);

        client = nh_.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");
    }

    void publishSensorPacket() {
        create_fundamentals::SensorPacket sensor_packet;

        sensor_packet.encoderLeft = encoder_left_;
        sensor_packet.encoderRight = encoder_right_;



        sensor_packet_pub_.publish(sensor_packet);
    }


    void simulateLidar() {
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "map";

        scan.angle_min = -M_PI * 2 / 3; // -120 degrees
        scan.angle_max = M_PI * 2 / 3; // 120 degrees
        scan.angle_increment = M_PI / 300;
        scan.range_min = 0.0;
        scan.range_max = 1.0;

        int num_ranges = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
        scan.ranges.resize(num_ranges);


        std::vector<plastic_fundamentals::Line> lines;

        double lidar_origin_x = robot_x_ + LIDAR_OFFSET_M * cos(robot_theta_);
        double lidar_origin_y = robot_y_ + LIDAR_OFFSET_M * sin(robot_theta_);

        for (int i = 0; i < num_ranges; ++i) {
            double angle = scan.angle_min + i * scan.angle_increment;
            double distance = getDistanceToWall(angle, lidar_origin_x, lidar_origin_y);

            if (distance > scan.range_max || std::isinf(distance)) {
                scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            } else {
                scan.ranges[i] = distance;

                double end_x = lidar_origin_x + distance * cos(robot_theta_ + angle);
                double end_y = lidar_origin_y + distance * sin(robot_theta_ + angle);

                plastic_fundamentals::Line line;
                line.x1 = lidar_origin_x;
                line.y1 = lidar_origin_y;
                line.x2 = end_x;
                line.y2 = end_y;

                lines.push_back(line);
            }
        }

        lidar_pub_.publish(scan);

        plastic_fundamentals::PublishMarker srv;
        srv.request.marker_type = "RayMarker";

        srv.request.lines = lines;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call marker service");
        }

        srv.request.marker_type = "RobotMarker";
        srv.request.robot.x = robot_x_;
        srv.request.robot.y = robot_y_;
        srv.request.robot.w = robot_theta_;

        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service publish_marker_service");
        }

        srv.request.marker_type = "MapMarker"; // For walls
        srv.request.lines = line_segments_;

        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service marker_service");
        }
    }

    void updatePose(double dt) {
        if (left_speed_ == 0 && right_speed_ == 0) {
            return;
        }

        double ticks_per_meter = 1 / WHEEL_RADIUS_M;

        double left_linear_velocity = left_speed_ / ticks_per_meter;
        double right_linear_velocity = right_speed_ / ticks_per_meter;

        double linear_velocity = (left_linear_velocity + right_linear_velocity) / 2.0;

        double angular_velocity = (right_linear_velocity - left_linear_velocity) / TRACK_WIDTH_M;

        double delta_theta = angular_velocity * dt;
        robot_theta_ += delta_theta;

        if (robot_theta_ >= 2 * M_PI) {
            robot_theta_ -= 2 * M_PI;
        } else if (robot_theta_ < 0) {
            robot_theta_ += 2 * M_PI;
        }

        double delta_x = linear_velocity * cos(robot_theta_) * dt;
        double delta_y = linear_velocity * sin(robot_theta_) * dt;

        robot_x_ += delta_x;
        robot_y_ += delta_y;

        double left_distance = left_linear_velocity * dt;
        double right_distance = right_linear_velocity * dt;

        double left_ticks = left_distance * ticks_per_meter;
        double right_ticks = right_distance * ticks_per_meter;

        encoder_left_ += left_ticks;
        encoder_right_ += right_ticks;

        //ROS_INFO("Angle = %f , X = %f, Y = %f, Left Encoder = %f, Right Encoder = %f",
        //         robot_theta_ / M_PI * 180, robot_x_, robot_y_, encoder_left_, encoder_right_);
    }

    bool diffDriveCallback(create_fundamentals::DiffDrive::Request &req, create_fundamentals::DiffDrive::Response &res) {
        double left_velocity = req.left;
        double right_velocity = req.right;

        left_speed_ = left_velocity;
        right_speed_ = right_velocity;

        //ROS_INFO("Received wheel velocities: Left = %f, Right = %f", left_velocity, right_velocity);

        res.success = true;
        return true;
    }

    bool resetEncodersCallback(create_fundamentals::ResetEncoders::Request &req, create_fundamentals::ResetEncoders::Response &res) {
        encoder_left_ = 0.0;
        encoder_right_ = 0.0;

        //ROS_INFO("Encoders reset successfully.");

        res.success = true;
        return true;
    }

private:

    bool checkIntersection(const plastic_fundamentals::Line& ray, const plastic_fundamentals::Line& segment, double& intersect_x, double& intersect_y) {
        double det = (ray.x1 - ray.x2) * (segment.y1 - segment.y2) - (ray.y1 - ray.y2) * (segment.x1 - segment.x2);

        const double epsilon = 1e-6;
        if (std::fabs(det) < epsilon) {
            return false;
        }

        double t = ((ray.x1 - segment.x1) * (segment.y1 - segment.y2) - (ray.y1 - segment.y1) * (segment.x1 - segment.x2)) / det;
        double s = ((ray.x1 - segment.x1) * (ray.y1 - ray.y2) - (ray.y1 - segment.y1) * (ray.x1 - ray.x2)) / det;

        if (t >= 0 && s >= 0 && s <= 1) {
            intersect_x = ray.x1 + t * (ray.x2 - ray.x1);
            intersect_y = ray.y1 + t * (ray.y2 - ray.y1);
            return true;
        }

        return false;
    }

    double getDistanceToWall(double angle, double lidar_origin_x, double lidar_origin_y) {
        double lidar_angle = robot_theta_ + angle;

        plastic_fundamentals::Line ray;
        ray.x1 = lidar_origin_x;
        ray.y1 = lidar_origin_y;

        ray.x2 = lidar_origin_x + cos(lidar_angle);
        ray.y2 = lidar_origin_y + sin(lidar_angle);

        double closest_distance = 1.05;
        double closest_intersect_x, closest_intersect_y;

        for (const auto& wall : line_segments_) {
            double intersect_x, intersect_y;
            if (checkIntersection(ray, wall, intersect_x, intersect_y)) {
                double distance = std::sqrt(std::pow(intersect_x - lidar_origin_x, 2) + std::pow(intersect_y - lidar_origin_y, 2));
                if (distance < closest_distance) {
                    closest_distance = distance;
                }
            }
        }

        return closest_distance;
    }

    std::vector<plastic_fundamentals::Line> preprocessMap(const plastic_fundamentals::Grid::ConstPtr& msg) {
        std::vector<plastic_fundamentals::Line> line_segments;

        int rows = msg->rows.size();
        int cols = msg->rows[0].cells.size();

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                const auto& cell = msg->rows[i].cells[j];
                double x = i * CELL_SIZE;
                double y = j * CELL_SIZE;

                if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::LEFT) != cell.walls.end()) {
                    plastic_fundamentals::Line line;
                    line.x1 = x;
                    line.y1 = y;
                    line.x2 = x + CELL_SIZE;
                    line.y2 = y;
                    line_segments.push_back(line);
                }

                if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::BOTTOM) != cell.walls.end()) {
                    plastic_fundamentals::Line line;
                    line.x1 = x + CELL_SIZE;
                    line.y1 = y;
                    line.x2 = x + CELL_SIZE;
                    line.y2 = y + CELL_SIZE;
                    line_segments.push_back(line);
                }

                if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::RIGHT) != cell.walls.end()) {
                    plastic_fundamentals::Line line;
                    line.x1 = x;
                    line.y1 = y + CELL_SIZE;
                    line.x2 = x + CELL_SIZE;
                    line.y2 = y + CELL_SIZE;
                    line_segments.push_back(line);
                }

                if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::TOP) != cell.walls.end()) {
                    plastic_fundamentals::Line line;
                    line.x1 = x;
                    line.y1 = y;
                    line.x2 = x;
                    line.y2 = y + CELL_SIZE;
                    line_segments.push_back(line);
                }
            }
        }

        return line_segments;
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher sensor_packet_pub_;
    ros::Publisher lidar_pub_;
    ros::ServiceServer diff_drive_service_;
    ros::ServiceServer reset_encoders_service_;
    plastic_fundamentals::Grid::ConstPtr map_data_;  // Maze map data

    double robot_x_;  // Robot's x position
    double robot_y_;  // Robot's y position
    double robot_theta_; // Robot's orientation
    double left_speed_;  // Left encoder value
    double right_speed_; // Right encoder value
    double encoder_left_;  // Left encoder value
    double encoder_right_; // Right encoder value

    double WHEEL_RADIUS_M = 0.0325;
    double TRACK_WIDTH_M = 0.263;
    double LIDAR_OFFSET_M = 0.16;
    double CELL_SIZE = 0.8;

    ros::ServiceClient client;

    std::vector<plastic_fundamentals::Line> line_segments_;


};

plastic_fundamentals::Grid::ConstPtr map_data;

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    if(map_data) {
        return;
    }

    ROS_INFO("Received map data");
    map_data = msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);

    ros::spinOnce();

    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(2).sleep();

        ros::spinOnce();
    }

    double cell_size = 0.8;

    int x = 0;
    int y = 0;

    int direction = 1; // 0: right, 1: up, 2: left, 3: down

    double initial_x = x * cell_size + 0.4;
    double initial_y = y * cell_size + 0.4;
    double initial_theta = (direction - 1) % 4 * M_PI / 2;

    RobotSimulator robot(nh, map_data, initial_x, initial_y, initial_theta);

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        double dt = loop_rate.expectedCycleTime().toSec() * 50;  // Time difference between updates

        robot.updatePose(dt);
        robot.publishSensorPacket();
        robot.simulateLidar();

        ros::spinOnce();
        loop_rate.sleep();
    }
}