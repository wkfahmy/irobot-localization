#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cmath>

// Measured robot-specific physical parameters
double radius = 0.0325;          // Wheel radius in meters
double track_width = 0.263;      // Distance between wheels

// Correction factors
double translation_error_factor = 1/0.81;  // 81cm actually moved vs 100cm target
double rotation_error_factor = 1/0.96;     // 96% of target rotation achieved

// Encoder
double base_ticks_per_rev = 5.0;          // Measured ticks/revolution
double ticksPerRevolution = base_ticks_per_rev * translation_error_factor;
double ticksPerRevolutionRot = 8.36;//base_ticks_per_rev * rotation_error_factor;

// PID parameters
double Kp_trans = 0.8, Ki_trans = 0.05, Kd_trans = 0.2;
double Kp_rot = 1.0, Ki_rot = 0.0, Kd_rot = 0.0;

/*Tuning Steps:
1. Start with P Only:
    - Set Ki=0, Kd=0
    - Increase Kp until robot starts oscillating
    - Reduce to 50% of this value

2. Add Derivative (D):
    - Start with Kd = Kp/10
    - Increase to reduce overshoot
    - Too much D causes slow response

3. Add Integral (I):
    - Start with Ki = Kp/100
    - Increase to eliminate steady-state error
    - Too much I causes windup and oscillations

Tuning Scenarios:
    - Overshooting: Increase D, decrease P
    - Slow Response: Increase P, decrease D
    - Steady-State Error: Increase I
    - Oscillations: Decrease P and I, increase D
*/

double tolerance = 3.0;  // Tolerance in ticks. Adjust based on results: large final error: Decrease tolerance.
// Oscillations/overshoots: Increase tolerance or retune PID gains.

ros::ServiceClient* diffDriveClient;
create_fundamentals::DiffDrive srv;
ros::ServiceClient* resetEncodersClient;

double leftTicks = 0;
double rightTicks = 0;

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if(resetEncodersClient->call(srv)) {
        leftTicks = 0;
        rightTicks = 0;
    }
}

double getTranslationTicks(double distance) {
    // Returns encoder ticks needed to travel given distance (meters)
    // Includes translation error compensation
    double circumference = 2 * M_PI * radius;
    double revolutions = distance / circumference;
    return revolutions * ticksPerRevolution;
}

double getRotationTicks(double angle_rad) {
    // Returns encoder ticks needed to rotate given angle (radians)
    // Includes rotation error compensation
    double wheel_distance = (angle_rad * track_width) / 2.0;
    double revolutions = wheel_distance / (2 * M_PI * radius);
    return revolutions * ticksPerRevolutionRot;
}

void rotate(double angle_rad, double max_speed) {
    ROS_INFO("Commanded rotation: %.2f radians (%.1f°)", angle_rad, angle_rad*180/M_PI);
    double target_ticks = getRotationTicks(fabs(angle_rad));
    resetEncoders();

    double error = 0, integral = 0, derivative = 0, prev_error = 0;
    ros::Rate rate(20);  // 20 Hz control loop
    int direction = (angle_rad > 0) ? 1 : -1;
    double correction = 0.0;

    do {
        ros::spinOnce();
        double current_ticks = (fabs(leftTicks) + fabs(rightTicks)) / 2.0;
        error = target_ticks - current_ticks;

        integral += error * 0.05;  // dt = 1/20Hz = 0.05s
        derivative = (error - prev_error) / 0.05;

        double output = Kp_rot*error + Ki_rot*integral + Kd_rot*derivative;
        output = fmin(fmax(output, -max_speed), max_speed);

        if (rightTicks != 0.0) {
            correction = 1 - abs(leftTicks) / abs(rightTicks);
        }
        srv.request.left = - direction * output * (1 + correction / 2);
        srv.request.right = direction * output * (1 - correction / 2);
        diffDriveClient->call(srv);

        prev_error = error;
        rate.sleep();
    } while(fabs(error) > tolerance);

    // Final stop and reset
    srv.request.left = srv.request.right = 0;
    diffDriveClient->call(srv);
    resetEncoders();
}

void translate(double distance, double max_speed) {
    ROS_INFO("Commanded translation: %.2f meters", distance);
    double target_ticks = getTranslationTicks(fabs(distance));
    resetEncoders();

    double error = 0, integral = 0, derivative = 0, prev_error = 0;
    ros::Rate rate(20);  // 20 Hz control loop
    int direction = (distance > 0) ? 1 : -1;

    do {
        ros::spinOnce();
        double current_ticks = (leftTicks + rightTicks) / 2.0;
        error = target_ticks - current_ticks;

        integral += error * 0.05;
        derivative = (error - prev_error) / 0.05;

        double output = Kp_trans*error + Ki_trans*integral + Kd_trans*derivative;
        output = fmin(fmax(output, -max_speed), max_speed);

        srv.request.left = srv.request.right = direction * output;
        diffDriveClient->call(srv);

        prev_error = error;
        rate.sleep();
    } while(fabs(error) > tolerance);

    // Final stop and reset
    srv.request.left = srv.request.right = 0;
    diffDriveClient->call(srv);
    resetEncoders();
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_controlled_square");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    diffDriveClient = &diffDrive;
    ros::ServiceClient resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    resetEncodersClient = &resetEncoders;

    double speed = 3.0;  // Start with moderate speed

    for (int i = 0; i < 4; i++) {
        //translate(1.0, speed);
        //ros::Duration(1.0).sleep();
        rotate(2 * M_PI, speed);
        ros::Duration(1.0).sleep();
    }

    return 0;
}