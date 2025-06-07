#include <ros/ros.h>
#include <plastic_fundamentals/PublishMarker.h>  // Custom service header (make sure your message types are set correctly)
#include <plastic_fundamentals/Grid.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>

#include <cmath>
#include <random>
#include <algorithm>

ros::ServiceClient client;

plastic_fundamentals::Grid::ConstPtr map_data;

std::vector<plastic_fundamentals::Line> map_lines;

constexpr double WHEEL_RADIUS_M = 0.0325;
constexpr double TRACK_WIDTH_M = 0.263;
constexpr double LIDAR_OFFSET_M = 0.16;
constexpr double CELL_SIZE = 0.8;



constexpr int NUM_PARTICLES = 1000;
constexpr double INIT_X_NOISE = 0.1;
constexpr double INIT_Y_NOISE = 0.1;
constexpr double INIT_THETA_NOISE = 0.2;

double leftTicks = 0;
double rightTicks = 0;

std::default_random_engine random_generator;


create_fundamentals::DiffDrive diffDriveSrv;

ros::ServiceClient* resetEncodersClient;
ros::ServiceClient* diffDriveClient;

void resetEncoders() {
    ROS_INFO("Resetting the encoders");
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
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

    // Create distributions for random noise
    std::uniform_real_distribution<double> x_noise_dist(-INIT_X_NOISE, INIT_X_NOISE);
    std::uniform_real_distribution<double> y_noise_dist(-INIT_Y_NOISE, INIT_Y_NOISE);
    std::uniform_real_distribution<double> theta_noise_dist(-INIT_THETA_NOISE, INIT_THETA_NOISE);

    int num_rows = map->rows.size();
    int num_cols = map->rows[0].cells.size();
    int total_cells = num_rows * num_cols;

    int particles_per_cell = NUM_PARTICLES / (total_cells * 4);
    if (particles_per_cell < 1) particles_per_cell = 1;

    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_cols; ++col) {
            double cell_center_x = (col + 0.5) * CELL_SIZE;
            double cell_center_y = (row + 0.5) * CELL_SIZE;

            for (int orientation = 0; orientation < 4; ++orientation) {
                double base_theta = orientation * M_PI / 2.0;

                for (int i = 0; i < particles_per_cell; ++i) {
                    if (particles.size() >= NUM_PARTICLES) break;

                    Particle p;
                    p.x = cell_center_x + x_noise_dist(random_generator);
                    p.y = cell_center_y + y_noise_dist(random_generator);
                    p.theta = base_theta + theta_noise_dist(random_generator);
                    p.weight = 1.0 / NUM_PARTICLES;
                    p.row = row;
                    p.col = col;
                    p.orientation = orientation;

                    particles.push_back(p);
                }
            }
        }
    }

    while (particles.size() < NUM_PARTICLES) {
        int idx = rand() % particles.size();
        particles.push_back(particles[idx]);
    }

    ROS_INFO("Initialized %zu particles", particles.size());

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
    ROS_INFO("Rotation: %f", angle_rad);

    // Calculate the number of encoder ticks required for the desired rotation angle
    double ticks = getRotationTicks(angle_rad);  // Get the required encoder ticks for the rotation

    resetEncoders();
    ros::spinOnce();

    double side = (angle_rad > 0) ? 1.0 : -1.0;

    diffDriveSrv.request.left = -side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(100);
    ros::spinOnce();

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
        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();


}


void translate(double distance, double speed) {
    ROS_INFO("Translation: %f", distance);

    double ticks = getTranslationTicks(distance);

    resetEncoders();
    ros::spinOnce();

    double side = (distance > 0) ? 1.0 : -1.0;
    diffDriveSrv.request.left = side * speed;
    diffDriveSrv.request.right = side * speed;

    ros::Rate rate(10);

    ros::spinOnce();

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
        rate.sleep();
        ros::spinOnce();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient->call(diffDriveSrv);

    resetEncoders();
}

void getMapLines(const plastic_fundamentals::Grid::ConstPtr& msg) {
    map_lines.clear();
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
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::BOTTOM) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x + CELL_SIZE;
                line.y1 = y;
                line.x2 = x + CELL_SIZE;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::RIGHT) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x;
                line.y1 = y + CELL_SIZE;
                line.x2 = x + CELL_SIZE;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }

            if (std::find(cell.walls.begin(), cell.walls.end(), plastic_fundamentals::Cell::TOP) != cell.walls.end()) {
                plastic_fundamentals::Line line;
                line.x1 = x;
                line.y1 = y;
                line.x2 = x;
                line.y2 = y + CELL_SIZE;
                map_lines.push_back(line);
            }
        }
    }

}

void mapCallback(const plastic_fundamentals::Grid::ConstPtr& msg) {
    if(map_data) {
        return;
    }

    ROS_INFO("Received map data");

    getMapLines(msg);
    map_data = msg;

    initializeParticles(map_data);

}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker_test");
    ros::NodeHandle nh;

    client = nh.serviceClient<plastic_fundamentals::PublishMarker>("marker_service");

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);

    ros::Subscriber sub = nh.subscribe("sensor_packet", 1, sensorCallback);

    ros::ServiceClient diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diffDriveClient = &diffDrive;
    ros::ServiceClient resetEncoders = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    ros::spinOnce();

    while (!map_data) {
        ROS_INFO("Waiting for map data to start..");
        ros::Duration(2).sleep();

        ros::spinOnce();
    }


    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Service marker_service not available!");
        return 1;
    }

    rotate(M_PI / 2, 20.0);

    translate(0.8, 20.0);

    /*ros::Rate rate(100);

    while (ros::ok()) {
        spinInPlace(diff_drive_client, M_PI / 2, 5.0);


        ros::spinOnce();
        rate.sleep();
    }*/

    /*plastic_fundamentals::PublishMarker srv;

    srv.request.marker_type = "PointMarker";

    for (const auto& particle : particles) {
        plastic_fundamentals::Point pt;
        pt.x = particle.x;
        pt.y = particle.y;
        srv.request.points.push_back(pt);
    }

    if (client.call(srv)) {
        ROS_INFO("PointMarker published successfully.");
    } else {
        ROS_ERROR("Failed to call service marker_service");
    }*/

    return 0;
}