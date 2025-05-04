#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cmath>

// PID controller class
class PID {
public:
    double kp, ki, kd;
    double prev_error;
    double integral;

    PID(double p = 0.0, double i = 0.0, double d = 0.0)
        : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0) {}

    void reset() {
        prev_error = 0.0;
        integral = 0.0;
    }

    double compute(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        integral += error * dt;
        double derivative = (dt > 0.0 ? (error - prev_error) / dt : 0.0);
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

// Robot parameters
static const double radius = 0.0325;        // wheel radius (m)
static const double error_factor = 1.0/0.81;
static const double ticksPerRevolution = 5.0 * error_factor;
static const double angle_error_factor = 1.0/0.96;
static const double ticksPerRevolutionRot = 6.4;
static const double track_width  = 0.263;

// Encoder readings
volatile double leftTicks = 0.0;
volatile double rightTicks = 0.0;

// ROS clients and services
ros::ServiceClient* diffDriveClient;
ros::ServiceClient* resetEncodersClient;
create_fundamentals::DiffDrive srv;

// PID controllers (to be initialized in main)
PID pidTrans, pidRot;

// Reset encoders
void resetEncoders() {
    create_fundamentals::ResetEncoders srv_reset;
    if (resetEncodersClient->call(srv_reset)) {
        leftTicks = 0.0;
        rightTicks = 0.0;
    }
}

// Convert distance (m) to encoder ticks
inline double getTranslationTicks(double distance) {
    return ticksPerRevolution * distance / (2.0 * M_PI * radius);
}

// Convert rotation (rad) to encoder ticks (average per wheel)
inline double getRotationTicks(double angle_rad) {
    return ticksPerRevolutionRot * angle_rad * track_width / (4.0 * M_PI * radius);
}

// Sensor callback to update encoder values
void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    leftTicks  = msg->encoderLeft;
    rightTicks = msg->encoderRight;
}

// Translate by a distance (m) using PID control
void translate(double distance, double max_speed) {
    double targetTicks = getTranslationTicks(distance);
    double side = (distance >= 0.0 ? 1.0 : -1.0);

    resetEncoders();
    pidTrans.reset();
    ros::Time lastTime = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        double now = ros::Time::now().toSec();
        double dt  = (ros::Time::now() - lastTime).toSec();
        lastTime = ros::Time::now();

        double currentTicks = (std::abs(leftTicks) + std::abs(rightTicks)) / 2.0;
        double controlSignal = pidTrans.compute(targetTicks, currentTicks, dt);
        // Clamp to max speed
        controlSignal = std::max(-max_speed, std::min(max_speed, controlSignal));

        srv.request.left  = side * controlSignal;
        srv.request.right = side * controlSignal;
        diffDriveClient->call(srv);

        // Check if within tolerance (1 tick)
        if (std::abs(targetTicks - currentTicks) < 0.3) break;
        rate.sleep();
    }
    // Stop
    srv.request.left = srv.request.right = 0.0;
    diffDriveClient->call(srv);
    resetEncoders();
}

// Rotate by an angle (rad) using PID control
void rotate(double angle_rad, double max_speed) {
    double targetTicks = getRotationTicks(angle_rad);
    double side = (angle_rad >= 0.0 ? 1.0 : -1.0);

    resetEncoders();
    pidRot.reset();
    ros::Time lastTime = ros::Time::now();
    double correction = 0.0;
    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        double now = ros::Time::now().toSec();
        double dt  = (ros::Time::now() - lastTime).toSec();
        lastTime = ros::Time::now();

        double currentTicks = (std::abs(leftTicks) + std::abs(rightTicks)) / 2.0;
        double controlSignal = pidRot.compute(targetTicks, currentTicks, dt);
        // Clamp
        controlSignal = std::max(-max_speed, std::min(max_speed, controlSignal));

        // One wheel forward, one backward for rotation

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
        }
        srv.request.left  = -side * controlSignal;// * (1 + correction / 4);
        srv.request.right =  side * controlSignal;// * (1 - correction / 4);
        diffDriveClient->call(srv);

        if (std::abs(targetTicks - currentTicks) < 1.0) break;
        rate.sleep();
    }
    srv.request.left = srv.request.right = 0.0;
    diffDriveClient->call(srv);
    resetEncoders();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "square_with_pid");
    ros::NodeHandle nh("~");

    // Subscribe and service clients
    ros::Subscriber sub = nh.subscribe("/sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("/diff_drive");
    diffDriveClient = &diffDrive;
    ros::ServiceClient resetEnc = nh.serviceClient<create_fundamentals::ResetEncoders>("/reset_encoders");
    resetEncodersClient = &resetEnc;

    // PID parameters (tunable via ROS params)
    double kp_t, ki_t, kd_t;
    nh.param("kp_trans", kp_t, 1.0);
    nh.param("ki_trans", ki_t, 0.0);
    nh.param("kd_trans", kd_t, 0.0);
    pidTrans = PID(kp_t, ki_t, kd_t);

    double kp_r, ki_r, kd_r;
    nh.param("kp_rot", kp_r, 1.2);
    nh.param("ki_rot", ki_r, 0.0);
    nh.param("kd_rot", kd_r, 0.0);
    pidRot = PID(kp_r, ki_r, kd_r);

    // Execute a square path
    double speed = 3.0;
    for (int i = 0; i < 4 && ros::ok(); ++i) {
        //translate(1.0, speed);
        //ros::Duration(1.0).sleep();
        rotate(M_PI / 2, speed);
        ros::Duration(0.5).sleep();
    }
    return 0;
}