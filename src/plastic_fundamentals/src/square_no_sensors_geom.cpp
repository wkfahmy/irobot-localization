/**************************************************************
 * square_no_sensors_geom.cpp
 * with geometry-derived timing and manual correction factors
 *
 * How to run: rosrun plastic_fundamentals square_no_sensors_geom   [omega_fwd] [omega_turn] [N]
 * 
 *     omega_fwd   : wheel speed straight (rad/s)   (default 3.0)
 *     omega_turn  : wheel speed spin     (rad/s)   (default 3.0)
 *     N           : how many squares to draw       (default 1)
 *************************************************************/

 #include "ros/ros.h"
 #include "create_fundamentals/DiffDrive.h"
 #include <cstdlib>     // std::atof, std::atoi
 #include <cmath>       // M_PI, std::fabs
 
 /* ---------- plastic geometry (measured with a caliper gauge and a ruler) ------------------------------- */
 constexpr double R_Wheel = 0.03235;       // 64.7 mm Ø  → 32.35 mm radius
 constexpr double Track_Width  = 0.263;    // wheel-to-wheel (m)
 
 /* ---------- correction/scaling factors (tuned based on real floor measurements) -------------- */
 constexpr double K_L = 1.00 / 0.905;    // K_L = 1.0m / actual_d_measured 
 constexpr double K_R  = 1.00; //90/85.0;   // K_R = 90∘/ actual_α_measured
 
 /* ---------- helper: low-level command ----------------------------------- */
 void setWheels(ros::ServiceClient& cli, double left, double right)
 {
     create_fundamentals::DiffDrive srv;
     srv.request.left  = left;
     srv.request.right = right;
     if (!cli.call(srv)) ROS_ERROR("diff_drive call failed");
 }
 
 /* ---------- drive exactly 1 m straight (open-loop) ------------------------------- */
 void driveOneMetre(ros::ServiceClient& diff, double omega_fwd)
 {
     const double v = omega_fwd * R_Wheel;              // m/s
     const double t = 1.0 / v * K_L;                    // adjusted time for linear movement in sec
 
     setWheels(diff, omega_fwd, omega_fwd);
     ros::Duration(t).sleep();
     setWheels(diff, 0.0, 0.0);
 }
 
 /* ---------- spin +90 ° CCW in place ------------------------------------- */
 void turnLeft90(ros::ServiceClient& diff, double omega_turn)
 {
     const double omega_robot = 2.0 * R_Wheel * omega_turn / Track_Width;       // rad/s
     const double t = (M_PI/2.0) / std::fabs(omega_robot) * K_R;                // adjusted time for rotating in sec
 
     setWheels(diff, -omega_turn, omega_turn);
     ros::Duration(t).sleep();
     setWheels(diff, 0.0, 0.0);
 }
 
 /* ======================================================================== */
 int main(int argc,char** argv)
 {
     ros::init(argc, argv, "square_no_sensors_geom");
     ros::NodeHandle n;
 
     ros::ServiceClient diff = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
     ros::service::waitForService("diff_drive");
 
     /* ---- CLI: wheel speeds & number of squares -------------------------- */
     double omega_fwd  = (argc > 1) ? std::atof(argv[1]) : 3.0;  // rad/s
     double omega_turn = (argc > 2) ? std::atof(argv[2]) : 3.0;  // rad/s
     int    squares    = (argc > 3) ? std::atoi(argv[3]) : 1;    // ≥1
     if (squares < 1) squares = 1;
 
     ROS_INFO_STREAM("Squares=" << squares
         << "  ω_fwd="  << omega_fwd  << " rad/s"
         << "  ω_turn=" << omega_turn << " rad/s"
         << "  K_Linear=" << K_L << "  K_Rotation=" << K_R);
 
     /* ---- main pattern --------------------------------------------------- */
     for (int s = 0; s < squares && ros::ok(); ++s)
     {
         ROS_INFO_STREAM("*** Square " << (s+1) << " ***");
         for (int e = 0; e < 4 && ros::ok(); ++e)
         {
             driveOneMetre(diff, omega_fwd);
             ros::Duration(0.3).sleep();
             turnLeft90(diff, omega_turn);
             ros::Duration(0.3).sleep();
         }
     }
 
     setWheels(diff, 0.0, 0.0);   // final brake
     ROS_INFO("We made it. Lets go plastic!");
     return 0;
 } 
