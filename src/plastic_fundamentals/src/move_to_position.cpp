#include <ros/ros.h>
#include <plastic_fundamentals/moveToPosition.h>
#include <plastic_fundamentals/ExecutePlan.h>
#include <plastic_fundamentals/Pose.h>
#include "graph_utils.hpp"
#include <mutex>

// Mutex for thread-safe pose access
std::mutex pose_mutex;

// Latest pose stored here
int current_pose_row = -1;
int current_pose_col = -1;
bool pose_received = false;

ros::ServiceClient execute_plan_client;

// Callback for /pose topic subscription
void poseCallback(const plastic_fundamentals::Pose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    current_pose_row = msg->row;
    current_pose_col = msg->column;
    pose_received = true;
}


bool moveToPosition(plastic_fundamentals::moveToPosition::Request &req, plastic_fundamentals::moveToPosition::Response &res) {
    int goal_row = req.row;
    int goal_col = req.column;

    int start_row, start_col;
    {
        std::lock_guard<std::mutex> lock(pose_mutex);
        if (!pose_received) {
            ROS_ERROR("Current pose not received yet. Cannot plan path.");
            res.success = false;
            return true;
        }
        start_row = current_pose_row;
        start_col = current_pose_col;
    }

    std::pair<int, int> start = {start_row, start_col};
    std::pair<int, int> goal = {goal_row, goal_col};

    ROS_INFO("Planning path from (%d, %d) to (%d, %d)", start.first, start.second, goal.first, goal.second);

    // Create graph
    auto graph = createGraph();

    // Find shortest path as list of positions
    std::vector<std::pair<int, int>> path_positions = findShortestPath(graph, start, goal);

    if (path_positions.empty()) {
        ROS_WARN("No valid path found to the goal (%d, %d).", goal_row, goal_col);
        res.success = false;
        return true;
    }

    // Convert path to direction enum values
    std::vector<int> path_directions = pathToDirections(path_positions);

    // Prepare the ExecutePlan service request
    plastic_fundamentals::ExecutePlan execute_req;
    execute_req.request.plan = path_directions;

    if (execute_plan_client.call(execute_req)) {
        res.success = execute_req.response.success;
        if (res.success) {
            ROS_INFO("Successfully executed plan to (%d, %d).", goal_row, goal_col);
        } else {
            ROS_WARN("Failed to execute plan even though path was found.");
        }
    } else {
        ROS_ERROR("Failed to call /execute_plan service.");
        res.success = false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_position_node");
    ros::NodeHandle nh;

    // Subscribe to /pose topic to get current pose updates
    ros::Subscriber pose_sub = nh.subscribe("/pose", 10, poseCallback);

    // Create service client for execute_plan
    execute_plan_client = nh.serviceClient<plastic_fundamentals::ExecutePlan>("execute_plan");

    // Advertise the move_to_position service
    ros::ServiceServer move_to_position_srv = nh.advertiseService("move_to_position", moveToPosition);

    ROS_INFO("move_to_position service is ready and listening for goals.");

    ros::spin();
    return 0;
}
