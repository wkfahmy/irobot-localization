#include <ros/ros.h>
#include <plastic_fundamentals/Move.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <create_fundamentals/SensorPacket.h>
#include <std_msgs/Empty.h>
#include <plastic_fundamentals/AbsEncoder.h>


double absLeftTicks = 0.0;
double absRightTicks = 0.0;

double leftTicks = 0;
double rightTicks = 0;

ros::ServiceClient diffDriveClient;
ros::ServiceClient resetEncodersClient;

ros::Publisher absEncoderPub;

template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg) {
    leftTicks = msg->encoderLeft;
    rightTicks = msg->encoderRight;

    plastic_fundamentals::AbsEncoder absEncoderMsg;
    absEncoderMsg.abs_left = absLeftTicks + leftTicks;
    absEncoderMsg.abs_right = absRightTicks + rightTicks;
    absEncoderPub.publish(absEncoderMsg);
}

void resetEncoders() {
    create_fundamentals::ResetEncoders srv;
    if (resetEncodersClient.call(srv)) {
        absLeftTicks += leftTicks;
        absRightTicks += rightTicks;

        leftTicks = 0.0;
        rightTicks = 0.0;
    } else {
        ROS_WARN("Failed to reset encoders.");
    }
}

double getRotationTicks(double angle_rad) {
    constexpr double TRACK_WIDTH_M = 0.263;
    constexpr double WHEEL_RADIUS_M = 0.0325;
    return (TRACK_WIDTH_M * angle_rad) / (2.0 * WHEEL_RADIUS_M);
}

double getTranslationTicks(double distance) {
    constexpr double WHEEL_RADIUS_M = 0.0325;
    return (distance / WHEEL_RADIUS_M);
}

bool handleRotate(plastic_fundamentals::Move::Request &req,
                  plastic_fundamentals::Move::Response &res)
{
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

    double ticks = getRotationTicks(fabs(req.angle));
    double side = (req.angle > 0) ? 1.0 : -1.0;

    resetEncoders();

    double small_margin = ticks * 0.01;

    while ((abs(leftTicks) + abs(rightTicks)) / 2.0 < ticks - small_margin && ros::ok()) {
        double error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2.0);
        double error_based_speed = req.speed * (error / ticks);

        if (error_based_speed < req.speed / 4.0)
            error_based_speed = req.speed / 4.0;

        double correction = 0.0;
        if (fabs(rightTicks) > 1e-3) {
            correction = clamp(1.0 - fabs(leftTicks) / fabs(rightTicks), -0.5, 0.5);
        }

        diffDriveSrv.request.left = -side * error_based_speed * (1.0 + correction / 2.0);
        diffDriveSrv.request.right =  side * error_based_speed * (1.0 - correction / 2.0);

        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = absLeftTicks + leftTicks;
        msg.abs_right = absRightTicks + rightTicks;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient.call(diffDriveSrv);

    resetEncoders();

    res.success = true;
    return true;
}

bool handleTranslate(plastic_fundamentals::Move::Request &req,
                     plastic_fundamentals::Move::Response &res)
{
    ros::Rate rate(1000);
    create_fundamentals::DiffDrive diffDriveSrv;

    double ticks = getTranslationTicks(fabs(req.distance));
    double side = (req.distance > 0) ? 1.0 : -1.0;

    resetEncoders();

    double small_margin = ticks * 0.01;

    while ((abs(leftTicks) + abs(rightTicks)) / 2.0 < ticks - small_margin && ros::ok()) {
        double error = fabs(ticks - (abs(leftTicks) + abs(rightTicks)) / 2.0);
        double error_based_speed = req.speed * (error / ticks);

        if (error_based_speed < req.speed / 2.0)
            error_based_speed = req.speed / 2.0;

        double correction = 0.0;
        if (fabs(rightTicks) > 1e-3) {
            correction = clamp(1.0 - fabs(leftTicks) / fabs(rightTicks), -0.5, 0.5);
        }

        diffDriveSrv.request.left = side * error_based_speed * (1.0 + correction / 2.0);
        diffDriveSrv.request.right =  side * error_based_speed * (1.0 - correction / 2.0);

        diffDriveClient.call(diffDriveSrv);

        plastic_fundamentals::AbsEncoder msg;
        msg.abs_left = absLeftTicks + leftTicks;
        msg.abs_right = absRightTicks + rightTicks;
        absEncoderPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    diffDriveSrv.request.left = 0;
    diffDriveSrv.request.right = 0;
    diffDriveClient.call(diffDriveSrv);

    resetEncoders();

    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("sensor_packet", 1, sensorCallback);
    diffDriveClient = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    resetEncodersClient = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    absEncoderPub = nh.advertise<plastic_fundamentals::AbsEncoder>("absolute_encoders", 10);

    ros::ServiceServer rotateSrv = nh.advertiseService("perform_rotation", handleRotate);
    ros::ServiceServer translateSrv = nh.advertiseService("perform_translation", handleTranslate);
    ROS_INFO("Encoder Mover ready.");
    ros::spin();
    return 0;
}
