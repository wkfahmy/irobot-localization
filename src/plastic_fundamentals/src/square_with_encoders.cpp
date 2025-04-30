/**************************************************************
 *  square_with_encoders.cpp
 *  (Robotics Fundamentals – SoSe 2025)
 *
 *  Executes a 1 m × 1 m square using wheel-encoder feedback.
 *  No run-time tuning: all numbers fixed at compile time.
 *
 *  Topics  : /sensor_packet   (create_fundamentals/SensorPacket)
 *  Service : /diff_drive      (create_fundamentals/DiffDrive)
 *************************************************************/

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cmath>

/* ───────── robot geometry (measure once, then leave) ────────────────────── */
constexpr double WHEEL_RADIUS_M = 0.0325;   //  65 mm Ø  →  32.5 mm radius
constexpr double TRACK_WIDTH_M  = 0.263;    //  26.3 cm  wheel-to-wheel

/* ───────── chosen wheel speeds (rad s⁻¹) ────────────────────────────────── */
constexpr double FWD_WHEEL_SPEED_RAD_S  = 2.5;   // forward legs
constexpr double TURN_WHEEL_SPEED_RAD_S = 2.5;   // in-place spin

/* ───────── globals filled by sensor callback ────────────────────────────── */
static double enc_left  = 0.0;      // cumulative wheel radians
static double enc_right = 0.0;      // "
static bool   enc_ok    = false;

void sensorCB(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    enc_left  = msg->encoderLeft;
    enc_right = msg->encoderRight;
    enc_ok    = true;
}

/* helper: set wheel speeds via diff_drive */
void setWheels(ros::ServiceClient& cli, double left, double right)
{
    create_fundamentals::DiffDrive srv;
    srv.request.left  = left;
    srv.request.right = right;
    if (!cli.call(srv))
        ROS_ERROR("diff_drive call failed");
}

/* helper: compute Δs, Δθ between two encoder snapshots                      *
 * enc units = wheel radians (already scaled by driver)                      */
inline void deltas(double L0,double R0,double L1,double R1,
                   double& ds,double& dtheta)
{
    double dL = (L1 - L0) * WHEEL_RADIUS_M;            // metres
    double dR = (R1 - R0) * WHEEL_RADIUS_M;            // metres
    ds     = 0.5 * (dL + dR);                          // centre travel
    dtheta = (dR - dL) / TRACK_WIDTH_M;                // body rotation
}

/* ======================================================================== */
int main(int argc,char** argv)
{
    ros::init(argc, argv, "square_with_encoders");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("sensor_packet",1,sensorCB);
    ros::ServiceClient diff =
        nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::service::waitForService("diff_drive");

    /* wait for first encoder packet */
    ROS_INFO("Waiting for encoders …");
    ros::Rate wait_r(20);
    while (ros::ok() && !enc_ok) { ros::spinOnce(); wait_r.sleep(); }
    ROS_INFO("Encoders online – starting square.");

    ros::Rate loop(50);                      // 50 Hz control loop

    for (int edge=0; edge<4 && ros::ok(); ++edge)
    {
        /* ── 1 m straight ─────────────────────────────────────────────── */
        double L0 = enc_left, R0 = enc_right;
        setWheels(diff,  FWD_WHEEL_SPEED_RAD_S,  FWD_WHEEL_SPEED_RAD_S);

        while (ros::ok())
        {
            ros::spinOnce();
            double ds,dth; deltas(L0,R0,enc_left,enc_right,ds,dth);
            if (ds >= 1.0) break;
            loop.sleep();
        }
        setWheels(diff, 0.0, 0.0);
        ros::Duration(0.3).sleep();          // damp residual motion

        /* ── 90 ° CCW turn ────────────────────────────────────────────── */
        L0 = enc_left; R0 = enc_right;
        setWheels(diff, -TURN_WHEEL_SPEED_RAD_S,  TURN_WHEEL_SPEED_RAD_S);

        while (ros::ok())
        {
            ros::spinOnce();
            double ds,dth; deltas(L0,R0,enc_left,enc_right,ds,dth);
            if (dth >= M_PI/2.0) break;      // reached +90 °
            loop.sleep();
        }
        setWheels(diff, 0.0, 0.0);
        ros::Duration(0.3).sleep();
    }

    ROS_INFO("Encoder-closed square complete.");
    setWheels(diff, 0.0, 0.0);               // final safety stop
    return 0;
}
