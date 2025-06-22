#include <ros/ros.h>
#include <plastic_fundamentals/MoveToPosition.h>
#include <plastic_fundamentals/Pose.h>
#include <create_fundamentals/StoreSong.h>
#include <create_fundamentals/PlaySong.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <utility>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <mutex>

enum ChallengeState {
    INITIALIZING,
    PLANNING,
    MOVING_TO_GOLD,
    COLLECTING_GOLD,
    MOVING_TO_PICKUP,
    EXITING,
    COMPLETED,
    FAILED
};

class ChallengeExecutor {
private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Publisher status_pub;
    ros::ServiceClient move_to_position_client;
    ros::ServiceClient store_song_client;
    ros::ServiceClient play_song_client;
    
    // State management
    ChallengeState current_state;
    std::mutex pose_mutex;
    std::mutex state_mutex;
    
    // Robot pose
    int current_row;
    int current_col;
    bool pose_received;
    
    // Challenge data
    std::vector<std::pair<int, int>> gold_locations;
    std::vector<std::pair<int, int>> pickup_locations;
    std::vector<bool> gold_collected;
    int current_gold_target;
    int selected_pickup;
    
    // Timing
    ros::Time collection_start_time;
    ros::Time exit_start_time;
    bool timing_active;
    
    // Retry logic
    int movement_retry_count;
    int max_retries;
    ros::Time last_movement_attempt;
    
    // Configuration
    std::string gold_file_path;
    std::string pickup_file_path;
    
public:
    ChallengeExecutor() : 
        nh("~"),
        current_state(INITIALIZING),
        current_row(-1),
        current_col(-1),
        pose_received(false),
        current_gold_target(-1),
        selected_pickup(-1),
        timing_active(false),
        movement_retry_count(0),
        max_retries(3)
    {
        // Get file paths from parameters
        nh.param<std::string>("gold_file", gold_file_path, "gold.txt");
        nh.param<std::string>("pickup_file", pickup_file_path, "pickup.txt");
        
        // Initialize ROS connections
        pose_sub = nh.subscribe("/pose", 10, &ChallengeExecutor::poseCallback, this);
        status_pub = nh.advertise<std_msgs::String>("/challenge_status", 10);
        
        move_to_position_client = nh.serviceClient<plastic_fundamentals::MoveToPosition>("/move_to_position");
        store_song_client = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
        play_song_client = nh.serviceClient<create_fundamentals::PlaySong>("play_song");
        
        ROS_INFO("Challenge Executor initialized");
        ROS_INFO("Gold file: %s", gold_file_path.c_str());
        ROS_INFO("Pickup file: %s", pickup_file_path.c_str());
    }
    
    void poseCallback(const plastic_fundamentals::Pose::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        current_row = msg->row;
        current_col = msg->column;
        pose_received = true;
    }
    
    bool parseLocationFile(const std::string& filename, std::vector<std::pair<int, int>>& locations) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open file: %s", filename.c_str());
            return false;
        }
        
        std::string line;
        std::getline(file, line);
        file.close();
        
        // Remove whitespace and parse JSON-like format: [[0,1], [3,1], ...]
        locations.clear();
        
        // Find all [x,y] pairs
        size_t pos = 0;
        while ((pos = line.find('[', pos)) != std::string::npos) {
            size_t end = line.find(']', pos);
            if (end == std::string::npos) break;
            
            std::string pair_str = line.substr(pos + 1, end - pos - 1);
            size_t comma = pair_str.find(',');
            if (comma != std::string::npos) {
                try {
                    int row = std::stoi(pair_str.substr(0, comma));
                    int col = std::stoi(pair_str.substr(comma + 1));
                    locations.push_back({row, col});
                } catch (const std::exception& e) {
                    ROS_ERROR("Error parsing location: %s", e.what());
                }
            }
            pos = end + 1;
        }
        
        ROS_INFO("Parsed %zu locations from %s", locations.size(), filename.c_str());
        for (size_t i = 0; i < locations.size(); ++i) {
            ROS_INFO("  Location %zu: (%d, %d)", i, locations[i].first, locations[i].second);
        }
        
        return !locations.empty();
    }
    
    void initializeSongs() {
        ROS_INFO("Initializing songs...");
        
        // Wait for services to be available
        if (!store_song_client.waitForExistence(ros::Duration(10.0))) {
            ROS_ERROR("store_song service not available");
            return;
        }
        
        // Gold collection song (song #2) - Upbeat melody
        std::vector<unsigned int> gold_song = {
            72, 16,  // C5
            76, 16,  // E5
            79, 16,  // G5
            84, 32,  // C6
            79, 16,  // G5
            76, 16,  // E5
            72, 32   // C5
        };
        
        create_fundamentals::StoreSong store_req;
        store_req.request.number = 2;
        store_req.request.song = gold_song;
        if (store_song_client.call(store_req)) {
            ROS_INFO("Gold collection song stored (song #2)");
        } else {
            ROS_ERROR("Failed to store gold collection song");
        }
        
        // Exit song (song #3) - Victory melody
        std::vector<unsigned int> exit_song = {
            60, 16,  // C4
            64, 16,  // E4
            67, 16,  // G4
            72, 16,  // C5
            76, 16,  // E5
            79, 16,  // G5
            84, 32,  // C6
            84, 32   // C6
        };
        
        store_req.request.number = 3;
        store_req.request.song = exit_song;
        if (store_song_client.call(store_req)) {
            ROS_INFO("Exit song stored (song #3)");
        } else {
            ROS_ERROR("Failed to store exit song");
        }
        
        // Success song (song #4) - Celebration
        std::vector<unsigned int> success_song = {
            72, 8, 76, 8, 79, 8, 84, 16,
            84, 8, 79, 8, 76, 8, 72, 16,
            72, 8, 76, 8, 79, 8, 84, 32
        };
        
        store_req.request.number = 4;
        store_req.request.song = success_song;
        if (store_song_client.call(store_req)) {
            ROS_INFO("Success song stored (song #4)");
        } else {
            ROS_ERROR("Failed to store success song");
        }
        
        // Failure song (song #5) - Sad melody
        std::vector<unsigned int> failure_song = {
            60, 32,  // C4
            58, 32,  // A#3
            56, 32,  // G#3
            55, 64   // G3
        };
        
        store_req.request.number = 5;
        store_req.request.song = failure_song;
        if (store_song_client.call(store_req)) {
            ROS_INFO("Failure song stored (song #5)");
        } else {
            ROS_ERROR("Failed to store failure song");
        }
        
        ROS_INFO("All songs initialized successfully");
    }
    
    void playSong(int song_number) {
        if (!play_song_client.waitForExistence(ros::Duration(1.0))) {
            ROS_ERROR("play_song service not available");
            return;
        }
        
        create_fundamentals::PlaySong play_req;
        play_req.request.number = song_number;
        if (play_song_client.call(play_req)) {
            ROS_INFO("Playing song %d", song_number);
        } else {
            ROS_ERROR("Failed to play song %d", song_number);
        }
    }
    
    double calculateDistance(const std::pair<int, int>& pos1, const std::pair<int, int>& pos2) {
        int dr = pos1.first - pos2.first;
        int dc = pos1.second - pos2.second;
        return std::sqrt(dr * dr + dc * dc);
    }
    
    int findNearestGold() {
        if (gold_locations.empty()) return -1;
        
        std::pair<int, int> current_pos;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            current_pos = {current_row, current_col};
        }
        
        int nearest_idx = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < gold_locations.size(); ++i) {
            if (!gold_collected[i]) {
                double dist = calculateDistance(current_pos, gold_locations[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_idx = i;
                }
            }
        }
        
        return nearest_idx;
    }
    
    int selectPickupLocation() {
        if (pickup_locations.empty()) return -1;
        
        std::pair<int, int> current_pos;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            current_pos = {current_row, current_col};
        }
        
        double dist1 = calculateDistance(current_pos, pickup_locations[0]);
        double dist2 = (pickup_locations.size() > 1) ? 
                      calculateDistance(current_pos, pickup_locations[1]) : 
                      std::numeric_limits<double>::max();
        
        return (dist1 <= dist2) ? 0 : 1;
    }
    
    bool moveToLocation(int row, int col) {
        if (!move_to_position_client.waitForExistence(ros::Duration(1.0))) {
            ROS_ERROR("move_to_position service not available");
            return false;
        }
        
        plastic_fundamentals::MoveToPosition move_req;
        move_req.request.row = row;
        move_req.request.column = col;
        
        ROS_INFO("Attempting to move to location (%d, %d) [attempt %d/%d]", 
                 row, col, movement_retry_count + 1, max_retries);
        
        last_movement_attempt = ros::Time::now();
        
        if (move_to_position_client.call(move_req)) {
            if (move_req.response.success) {
                movement_retry_count = 0; // Reset retry count on success
                return true;
            } else {
                ROS_WARN("Move to position service returned failure");
                return false;
            }
        } else {
            ROS_ERROR("Failed to call move_to_position service");
            return false;
        }
    }
    
    bool isAtLocation(int target_row, int target_col) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        return (current_row == target_row && current_col == target_col);
    }
    
    bool allGoldCollected() {
        for (bool collected : gold_collected) {
            if (!collected) return false;
        }
        return true;
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub.publish(msg);
        ROS_INFO("Status: %s", status.c_str());
    }
    
    void setState(ChallengeState new_state) {
        std::lock_guard<std::mutex> lock(state_mutex);
        current_state = new_state;
    }
    
    ChallengeState getState() {
        std::lock_guard<std::mutex> lock(state_mutex);
        return current_state;
    }
    
    void executeStateMachine() {
        ros::Rate rate(10); // 10 Hz
        
        while (ros::ok()) {
            ChallengeState state = getState();
            
            switch (state) {
                case INITIALIZING:
                    handleInitializing();
                    break;
                    
                case PLANNING:
                    handlePlanning();
                    break;
                    
                case MOVING_TO_GOLD:
                    handleMovingToGold();
                    break;
                    
                case COLLECTING_GOLD:
                    handleCollectingGold();
                    break;
                    
                case MOVING_TO_PICKUP:
                    handleMovingToPickup();
                    break;
                    
                case EXITING:
                    handleExiting();
                    break;
                    
                case COMPLETED:
                    handleCompleted();
                    return; // Exit the loop
                    
                case FAILED:
                    handleFailed();
                    return; // Exit the loop
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void handleInitializing() {
        publishStatus("Initializing challenge executor");
        
        // Wait for pose
        if (!pose_received) {
            ROS_INFO_THROTTLE(5, "Waiting for robot pose...");
            return;
        }
        
        ROS_INFO("Robot pose received: (%d, %d)", current_row, current_col);
        
        // Parse location files
        if (!parseLocationFile(gold_file_path, gold_locations)) {
            publishStatus("Failed to parse gold locations file");
            setState(FAILED);
            return;
        }
        
        if (!parseLocationFile(pickup_file_path, pickup_locations)) {
            publishStatus("Failed to parse pickup locations file");
            setState(FAILED);
            return;
        }
        
        // Validate that we have valid locations
        if (gold_locations.empty()) {
            publishStatus("No gold locations found");
            setState(FAILED);
            return;
        }
        
        if (pickup_locations.empty()) {
            publishStatus("No pickup locations found");
            setState(FAILED);
            return;
        }
        
        // Initialize gold collection tracking
        gold_collected.resize(gold_locations.size(), false);
        
        // Initialize songs
        initializeSongs();
        
        publishStatus("Initialization complete - Starting challenge!");
        setState(PLANNING);
    }
    
    void handlePlanning() {
        publishStatus("Planning next move");
        
        // Find nearest uncollected gold
        current_gold_target = findNearestGold();
        
        if (current_gold_target == -1) {
            // No more gold to collect, plan exit
            publishStatus("All gold collected! Planning exit route");
            selected_pickup = selectPickupLocation();
            if (selected_pickup == -1 || selected_pickup >= pickup_locations.size()) {
                publishStatus("No valid pickup location found");
                setState(FAILED);
                return;
            }
            
            ROS_INFO("Selected pickup location %d: (%d, %d)", 
                     selected_pickup, 
                     pickup_locations[selected_pickup].first,
                     pickup_locations[selected_pickup].second);
            
            setState(MOVING_TO_PICKUP);
        } else {
            ROS_INFO("Next gold target %d: (%d, %d)", 
                     current_gold_target,
                     gold_locations[current_gold_target].first,
                     gold_locations[current_gold_target].second);
            setState(MOVING_TO_GOLD);
        }
        
        movement_retry_count = 0; // Reset retry count for new movement
    }
    
    void handleMovingToGold() {
        if (current_gold_target < 0 || current_gold_target >= gold_locations.size()) {
            publishStatus("Invalid gold target");
            setState(FAILED);
            return;
        }
        
        auto target = gold_locations[current_gold_target];
        
        if (isAtLocation(target.first, target.second)) {
            publishStatus("Arrived at gold location, starting collection");
            setState(COLLECTING_GOLD);
            collection_start_time = ros::Time::now();
            timing_active = true;
            playSong(2); // Play gold collection song
            return;
        }
        
        // Check if we should retry movement
        ros::Duration time_since_attempt = ros::Time::now() - last_movement_attempt;
        if (time_since_attempt.toSec() < 5.0) {
            // Wait a bit more for the movement to complete
            return;
        }
        
        // Try to move to the location
        if (!moveToLocation(target.first, target.second)) {
            movement_retry_count++;
            if (movement_retry_count >= max_retries) {
                publishStatus("Failed to reach gold location after maximum retries");
                setState(FAILED);
            } else {
                ROS_WARN("Movement failed, will retry (%d/%d)", movement_retry_count, max_retries);
            }
        }
    }
    
    void handleCollectingGold() {
        if (!timing_active) {
            setState(FAILED);
            return;
        }
        
        ros::Duration elapsed = ros::Time::now() - collection_start_time;
        
        if (elapsed.toSec() >= 5.0) {
            // Collection complete
            timing_active = false;
            gold_collected[current_gold_target] = true;
            publishStatus("Gold collected successfully!");
            
            // Count collected gold
            int collected_count = 0;
            for (bool collected : gold_collected) {
                if (collected) collected_count++;
            }
            
            ROS_INFO("Gold collected: %d/%zu", collected_count, gold_locations.size());
            
            if (allGoldCollected()) {
                publishStatus("All gold collected! Planning exit route");
                setState(PLANNING); // Will transition to MOVING_TO_PICKUP
            } else {
                setState(PLANNING); // Find next gold
            }
        } else {
            // Still collecting, ensure we stay in position
            auto target = gold_locations[current_gold_target];
            if (!isAtLocation(target.first, target.second)) {
                publishStatus("Robot moved during collection, repositioning");
                setState(MOVING_TO_GOLD);
                timing_active = false;
            } else {
                // Show progress
                double remaining = 5.0 - elapsed.toSec();
                ROS_INFO_THROTTLE(1, "Collecting gold... %.1f seconds remaining", remaining);
            }
        }
    }
    
    void handleMovingToPickup() {
        if (selected_pickup < 0 || selected_pickup >= pickup_locations.size()) {
            publishStatus("Invalid pickup target");
            setState(FAILED);
            return;
        }
        
        auto target = pickup_locations[selected_pickup];
        
        if (isAtLocation(target.first, target.second)) {
            publishStatus("Arrived at pickup location, starting exit sequence");
            setState(EXITING);
            exit_start_time = ros::Time::now();
            timing_active = true;
            playSong(3); // Play exit song
            return;
        }
        
        // Check if we should retry movement
        ros::Duration time_since_attempt = ros::Time::now() - last_movement_attempt;
        if (time_since_attempt.toSec() < 5.0) {
            // Wait a bit more for the movement to complete
            return;
        }
        
        // Try to move to the pickup location
        if (!moveToLocation(target.first, target.second)) {
            movement_retry_count++;
            if (movement_retry_count >= max_retries) {
                // Try the other pickup location if available
                if (pickup_locations.size() > 1) {
                    selected_pickup = (selected_pickup == 0) ? 1 : 0;
                    movement_retry_count = 0;
                    ROS_WARN("Switching to alternative pickup location: (%d, %d)",
                             pickup_locations[selected_pickup].first,
                             pickup_locations[selected_pickup].second);
                } else {
                    publishStatus("Failed to reach pickup location after maximum retries");
                    setState(FAILED);
                }
            } else {
                ROS_WARN("Movement to pickup failed, will retry (%d/%d)", movement_retry_count, max_retries);
            }
        }
    }
    
    void handleExiting() {
        if (!timing_active) {
            setState(FAILED);
            return;
        }
        
        ros::Duration elapsed = ros::Time::now() - exit_start_time;
        
        if (elapsed.toSec() >= 5.0) {
            // Exit complete
            timing_active = false;
            publishStatus("Exit sequence complete - Challenge successful!");
            setState(COMPLETED);
        } else {
            // Still exiting, ensure we stay in position
            auto target = pickup_locations[selected_pickup];
            if (!isAtLocation(target.first, target.second)) {
                publishStatus("Robot moved during exit, repositioning");
                setState(MOVING_TO_PICKUP);
                timing_active = false;
            } else {
                // Show progress
                double remaining = 5.0 - elapsed.toSec();
                ROS_INFO_THROTTLE(1, "Exiting maze... %.1f seconds remaining", remaining);
            }
        }
    }
    
    void handleCompleted() {
        publishStatus("Challenge completed successfully!");
        playSong(4); // Play success song
        ROS_INFO("=== CHALLENGE COMPLETED SUCCESSFULLY ===");
        
        // Print summary
        ROS_INFO("Gold locations visited: %zu", gold_locations.size());
        ROS_INFO("Pickup location used: (%d, %d)", 
                 pickup_locations[selected_pickup].first,
                 pickup_locations[selected_pickup].second);
    }
    
    void handleFailed() {
        publishStatus("Challenge execution failed");
        playSong(5); // Play failure song
        ROS_ERROR("=== CHALLENGE EXECUTION FAILED ===");
        
        // Print failure summary
        int collected_count = 0;
        for (bool collected : gold_collected) {
            if (collected) collected_count++;
        }
        ROS_ERROR("Gold collected before failure: %d/%zu", collected_count, gold_locations.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "challenge_dry");
    
    ChallengeExecutor executor;
    
    ROS_INFO("=== SARTING DRY CHALLENGE===");
    ROS_INFO("Ready to execute gold collection challenge");
    
    executor.executeStateMachine();
    
    return 0;
}

