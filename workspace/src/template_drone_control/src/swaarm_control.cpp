#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <map>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <thread> // For threading

using namespace std::chrono_literals;

class MultiDroneControl : public rclcpp::Node
{
public:

    struct Coords {
        double x;
        double y;
        double z;
        double angle; // in degrees
    };

    MultiDroneControl() : Node("multi_drone_control_node"), mission_aborted_(false)
    {
        // Define namespaces for the drones
        drone_namespaces_ = {"drone1", "drone2", "drone3"};

        // Declare parameters for flexibility
        this->declare_parameter<int>("max_missed_state_count", 3);
        this->declare_parameter<int>("max_missed_pose_count", 3);
        this->declare_parameter<double>("state_timeout_duration", 1.0); // seconds
        this->declare_parameter<double>("pose_timeout_duration", 1.0);  // seconds
        this->declare_parameter<double>("movement_delay", 4.0);         // seconds

        // Get parameter values
        this->get_parameter("max_missed_state_count", MAX_MISSED_STATE_COUNT);
        this->get_parameter("max_missed_pose_count", MAX_MISSED_POSE_COUNT);
        this->get_parameter("state_timeout_duration", STATE_TIMEOUT_DURATION);
        this->get_parameter("pose_timeout_duration", POSE_TIMEOUT_DURATION);
        this->get_parameter("movement_delay", MOVEMENT_DELAY);

        for (const auto &ns : drone_namespaces_)
        {
            RCLCPP_INFO(this->get_logger(), "Setting up drone with namespace: %s", ns.c_str());

            // State Topic Subscription
            std::string state_topic = "/" + ns + "/state";
            auto state_sub = this->create_subscription<mavros_msgs::msg::State>(
                state_topic, 10,
                [this, ns](mavros_msgs::msg::State::SharedPtr msg) {
                    state_callback(msg, ns);
                });
            state_subs_.push_back(state_sub);

            // Local Position Topic Subscription with SensorDataQoS
            std::string position_topic = "/" + ns + "/local_position/pose";
            auto pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                position_topic, rclcpp::SensorDataQoS(),
                [this, ns](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    local_pos_cb(msg, ns);
                });
            local_pos_subs_.push_back(pose_sub);

            // Setpoint Position Publisher
            std::string setpoint_topic = "/" + ns + "/setpoint_position/local";
            auto pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(setpoint_topic, 10);
            local_pos_pubs_.push_back(pose_pub);

            // Service Clients
            arming_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandBool>("/" + ns + "/cmd/arming"));

            set_mode_clients_.push_back(
                this->create_client<mavros_msgs::srv::SetMode>("/" + ns + "/set_mode"));

            takeoff_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandTOL>("/" + ns + "/cmd/takeoff"));
        }

        // Initialize connection and pose maps
        for (const auto &ns : drone_namespaces_) {
            connection_status_[ns] = false;
            last_state_time_[ns] = this->now();
            last_pose_time_[ns] = this->now();
            missed_state_counts_[ns] = 0;
            missed_pose_counts_[ns] = 0;
        }

        // Launch the mission execution in a separate thread
        std::thread(&MultiDroneControl::execute_mission, this).detach();

        // Timer to check for state and pose message timeouts
        timeout_timer_ = this->create_wall_timer(
            1s, std::bind(&MultiDroneControl::check_timeouts, this));
    }

private:

    // Constants for maximum allowed missed messages
    int MAX_MISSED_STATE_COUNT;
    int MAX_MISSED_POSE_COUNT;
    double STATE_TIMEOUT_DURATION; // seconds
    double POSE_TIMEOUT_DURATION;  // seconds
    double MOVEMENT_DELAY;         // seconds between movements

    std::vector<std::string> drone_namespaces_;
    std::vector<rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr> state_subs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> local_pos_pubs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> local_pos_subs_;
    std::vector<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr> arming_clients_;
    std::vector<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr> set_mode_clients_;
    std::vector<rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr> takeoff_clients_;
    std::map<std::string, mavros_msgs::msg::State> current_states_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> current_positions_;
    std::map<std::string, bool> connection_status_;
    std::map<std::string, rclcpp::Time> last_state_time_;
    std::map<std::string, rclcpp::Time> last_pose_time_;
    std::map<std::string, int> missed_state_counts_;
    std::map<std::string, int> missed_pose_counts_;
    std::mutex mutex_; // Mutex for thread-safe access
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    bool mission_aborted_; // Flag to indicate if the mission has been aborted

    // Callback function for state messages
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg, const std::string &ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_states_[ns] = *msg;
        connection_status_[ns] = msg->connected;
        last_state_time_[ns] = this->now();

        // Reset missed state counter since a state message was received
        missed_state_counts_[ns] = 0;

        if (!msg->connected && !mission_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Drone %s has disconnected!", ns.c_str());
            mission_aborted_ = true;
            handle_disconnection(ns);
        } else if (msg->connected && mission_aborted_) {
            RCLCPP_INFO(this->get_logger(), "Drone %s has reconnected.", ns.c_str());
            // Optionally, you can decide whether to reintegrate the drone into the mission
        }
    }

    // Callback function for local position
    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &ns) 
    {   
        RCLCPP_INFO(this->get_logger(), "Updated position for %s: x=%.2f, y=%.2f, z=%.2f", 
            ns.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // Lock the mutex to ensure thread-safe access to current_positions_
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Update the position for the given namespace
        current_positions_[ns] = *msg;
        last_pose_time_[ns] = this->now();

        // Reset missed pose counter since a message was received
        missed_pose_counts_[ns] = 0;
    }

    // Function to execute mission steps
    void execute_mission()
    {
        // Connect to all drones, set mode, arm, and take off
        for (const auto &ns : drone_namespaces_)
        {
            if (mission_aborted_) {
                RCLCPP_ERROR(this->get_logger(), "Mission has been aborted. Skipping initialization for drone: %s", ns.c_str());
                continue;
            }

            wait_for_connection(ns);
            if (mission_aborted_) break; // Check if mission was aborted during connection

            set_mode(ns, "GUIDED");
            if (mission_aborted_) break;

            arm_drone(ns);
            if (mission_aborted_) break;

            takeoff(ns, 3.0, 0.0); // Takeoff to 3 meters
            if (mission_aborted_) break;
        }

        if (mission_aborted_) {
            RCLCPP_ERROR(this->get_logger(), "Mission aborted during initialization.");
            return;
        }

        // Allow some time for drones to stabilize
        std::this_thread::sleep_for(6s);

        // Define coordinates for mission steps
        Coords coordsDrone1;

        // List of mission steps
        std::vector<Coords> mission_steps = {
            {0.0, 1.0, 3.0, 0.0},    // First movement
            {0.0, 6.0, 3.0, 0.0},    // Second movement
            {3.0, 6.0, 3.0, -90.0},  // Third movement
            {3.0, 6.0, 3.0, -180.0}, // Fourth movement
            {3.0, 0.0, 3.0, -90.0}   // Fifth movement
        };

        for (size_t i = 0; i < mission_steps.size(); ++i)
        {
            if (mission_aborted_) {
                RCLCPP_ERROR(this->get_logger(), "Mission aborted before executing movement step %zu.", i+1);
                return;
            }

            coordsDrone1 = mission_steps[i];
            RCLCPP_INFO(this->get_logger(), "Executing movement step %zu.", i+1);
            if (!move_drones(coordsDrone1)) {
                RCLCPP_ERROR(this->get_logger(), "Mission aborted during movement step %zu.", i+1);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Completed movement step %zu.", i+1);

            // Sleep longer to allow drones to reach the waypoint and send pose updates
            std::this_thread::sleep_for(MOVEMENT_DELAY * 1s);
        }

        // Land all drones
        land_all_drones();

        RCLCPP_INFO(this->get_logger(), "Mission completed successfully.");
    }

    // Function to wait for a drone to connect
    void wait_for_connection(const std::string &ns)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for connection to drone: %s", ns.c_str());
        while (rclcpp::ok())
        {
            bool connected = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (current_states_.find(ns) != current_states_.end())
                {
                    connected = current_states_[ns].connected;
                }
            }

            if (connected)
            {
                RCLCPP_INFO(this->get_logger(), "Connected to drone: %s", ns.c_str());
                break;
            }

            std::this_thread::sleep_for(100ms);
        }
    }

    // Function to set mode for a drone
    void set_mode(const std::string &ns, const std::string &mode)
    {
        auto idx = index_for_namespace(ns);
        if (idx >= set_mode_clients_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid index for set_mode_clients_ for drone: %s", ns.c_str());
            return;
        }

        auto client = set_mode_clients_[idx];
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok() || mission_aborted_)
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set_mode service for drone: %s. Exiting.", ns.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service for drone: %s...", ns.c_str());
        }

        auto future = client->async_send_request(request);

        // Wait for the result without spinning
        if (future.wait_for(5s) == std::future_status::ready)
        {
            auto response = future.get();
            // You can add additional checks on the response if needed
            if (response->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Mode set to %s for drone: %s", mode.c_str(), ns.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set mode to %s for drone: %s", mode.c_str(), ns.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode for drone: %s", ns.c_str());
            mission_aborted_ = true;
            handle_disconnection(ns);
        }
    }


    bool move_drones(Coords coordsLeadingDrone)
    {
        // If the mission has been aborted, do not proceed
        if (mission_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Mission has been aborted. Skipping movement.");
            return false;
        }

        // Check if all drones are active before proceeding
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto &ns : drone_namespaces_) {
                if (!connection_status_[ns]) {
                    RCLCPP_WARN(this->get_logger(), "Drone %s is inactive. Aborting mission.", ns.c_str());
                    mission_aborted_ = true;
                    handle_disconnection(ns);
                    return false;
                }
            }
        }

        // Proceed to move all drones
        int active_drones = 0;
        Coords coordsDrone2, coordsDrone3;

        // Move leading drone
        move_drone(drone_namespaces_[0], coordsLeadingDrone);
        active_drones++;

        // Calculate follower positions based on leading drone's coordinates
        std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsLeadingDrone);

        // Move drone2
        move_drone(drone_namespaces_[1], coordsDrone2);
        active_drones++;

        // Move drone3
        move_drone(drone_namespaces_[2], coordsDrone3);
        active_drones++;

        // Implement movement confirmation or synchronization if necessary
        // For simplicity, we'll assume movement commands are sent successfully

        return true;
    }

    // Function to arm a drone
    void arm_drone(const std::string &ns)
    {
        auto idx = index_for_namespace(ns);
        if (idx >= arming_clients_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid index for arming_clients_ for drone: %s", ns.c_str());
            return;
        }

        auto client = arming_clients_[idx];
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok() || mission_aborted_)
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arming service for drone: %s. Exiting.", ns.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service for drone: %s...", ns.c_str());
        }

        auto future = client->async_send_request(request);

        // Wait for the result without spinning
        if (future.wait_for(5s) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Drone armed: %s", ns.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm drone: %s", ns.c_str());
                mission_aborted_ = true;
                handle_disconnection(ns);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone: %s", ns.c_str());
            mission_aborted_ = true;
            handle_disconnection(ns);
        }
    }

    // Function to take off a drone
    void takeoff(const std::string &ns, double altitude, double yaw)
    {
        auto idx = index_for_namespace(ns);
        if (idx >= takeoff_clients_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid index for takeoff_clients_ for drone: %s", ns.c_str());
            return;
        }

        auto client = takeoff_clients_[idx];
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude;
        request->yaw = yaw;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok() || mission_aborted_)
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for takeoff service for drone: %s. Exiting.", ns.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service for drone: %s...", ns.c_str());
        }

        auto future = client->async_send_request(request);

        // Wait for the result without spinning
        if (future.wait_for(5s) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff command sent for drone: %s", ns.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff for drone: %s", ns.c_str());
                mission_aborted_ = true;
                handle_disconnection(ns);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff for drone: %s", ns.c_str());
            mission_aborted_ = true;
            handle_disconnection(ns);
        }
    }

    // Function to land a drone
    void land_drone(const std::string &ns)
    {
        auto idx = index_for_namespace(ns);
        if (idx >= set_mode_clients_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid index for set_mode_clients_ for drone: %s", ns.c_str());
            return;
        }

        auto client = set_mode_clients_[idx];
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "LAND";

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set_mode service for drone: %s. Exiting.", ns.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service for drone: %s...", ns.c_str());
        }

        auto future = client->async_send_request(request);

        // Wait for the result without spinning
        if (future.wait_for(5s) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Landing command sent for drone: %s", ns.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to land drone: %s", ns.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to land drone: %s", ns.c_str());
        }
    }

    // Function to handle disconnection
    void handle_disconnection(const std::string &ns) {
        RCLCPP_ERROR(this->get_logger(), "Handling disconnection for drone: %s", ns.c_str());
        
        // Stop all drones
        land_all_drones();

        // Optionally, implement additional logic such as alerting operators or attempting reconnections
    }

    // Function to handle reconnection
    void handle_reconnection(const std::string &ns) {
        RCLCPP_INFO(this->get_logger(), "Handling reconnection for drone: %s", ns.c_str());

        // Optionally, implement logic to resume missions or notify operators
        // For this implementation, we do not reintegrate the drone into the mission automatically
    }

    // Function to check for state and pose message timeouts
    void check_timeouts() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &ns : drone_namespaces_) {
            // Check state message timeout
            auto time_since_last_state = this->now() - last_state_time_[ns];
            if (time_since_last_state.seconds() > STATE_TIMEOUT_DURATION) {
                missed_state_counts_[ns]++;
                RCLCPP_DEBUG(this->get_logger(), "Missed state message from %s (%d/%d)", ns.c_str(), missed_state_counts_[ns], MAX_MISSED_STATE_COUNT);
                if (missed_state_counts_[ns] >= MAX_MISSED_STATE_COUNT && connection_status_[ns] && !mission_aborted_) {
                    RCLCPP_WARN(this->get_logger(), "Drone %s state update timed out!", ns.c_str());
                    mission_aborted_ = true;
                    handle_disconnection(ns);
                    return;
                }
            } else {
                // Reset state message counter if message is recent
                missed_state_counts_[ns] = 0;
            }

            // Check pose message timeout
            auto time_since_last_pose = this->now() - last_pose_time_[ns];
            if (time_since_last_pose.seconds() > POSE_TIMEOUT_DURATION) {
                missed_pose_counts_[ns]++;
                RCLCPP_DEBUG(this->get_logger(), "Missed pose message from %s (%d/%d)", ns.c_str(), missed_pose_counts_[ns], MAX_MISSED_POSE_COUNT);
                if (missed_pose_counts_[ns] >= MAX_MISSED_POSE_COUNT && !mission_aborted_) {
                    RCLCPP_WARN(this->get_logger(), "Drone %s pose update timed out!", ns.c_str());
                    mission_aborted_ = true;
                    handle_disconnection(ns);
                    return;
                }
            } else {
                // Reset pose message counter if message is recent
                missed_pose_counts_[ns] = 0;
            }
        }
    }

    // Function to land all drones
    void land_all_drones()
    {
        RCLCPP_INFO(this->get_logger(), "Landing all drones due to detected disconnection.");
        for (const auto &ns : drone_namespaces_) {
            if (connection_status_[ns]) {
                land_drone(ns);
            } else {
                RCLCPP_WARN(this->get_logger(), "Drone %s is already disconnected. Skipping landing.", ns.c_str());
            }
        }
    }

    // Function to get the index of a drone based on its namespace
    size_t index_for_namespace(const std::string &ns)
    {
        auto it = std::find(drone_namespaces_.begin(), drone_namespaces_.end(), ns);
        if (it != drone_namespaces_.end())
        {
            return std::distance(drone_namespaces_.begin(), it);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Namespace not found: %s", ns.c_str());
            return drone_namespaces_.size(); // Return an invalid index
        }
    }

    // Function to calculate follower drone positions
    std::pair<Coords, Coords> calculateFollowerPositions(const Coords &coords) {
        // Circle radius (half of the diameter)
        const double radius = 1;
        double angle = coords.angle;

        // Angles for the follower drones in degrees
        const double drone1AngleOffset = 225.0 + 90; // Drone 1 offset angle
        const double drone2AngleOffset = 135.0 + 90; // Drone 2 offset angle

        // Convert angles from degrees to radians
        double drone1AngleRad = (angle + drone1AngleOffset) * M_PI / 180.0;
        double drone2AngleRad = (angle + drone2AngleOffset) * M_PI / 180.0;

        // Calculate coordinates for drone 1
        Coords drone1;
        drone1.x = coords.x + radius * cos(drone1AngleRad);
        drone1.y = coords.y + 1 + radius * sin(drone1AngleRad);
        drone1.z = coords.z; // Same altitude as the leading drone
        drone1.angle = coords.angle;

        // Calculate coordinates for drone 2
        Coords drone2;
        drone2.x = coords.x + radius * cos(drone2AngleRad);
        drone2.y = coords.y + 2 + radius * sin(drone2AngleRad);
        drone2.z = coords.z; // Same altitude as the leading drone
        drone2.angle = coords.angle;

        // Return the pair of follower drone coordinates
        return std::make_pair(drone1, drone2);
    }

    // Function to move a single drone
    void move_drone(const std::string &ns, const Coords &coords)
    {
        RCLCPP_INFO(this->get_logger(), "Moving drone %s to coordinates: x=%.2f, y=%.2f, z=%.2f", 
                    ns.c_str(), coords.x, coords.y, coords.z);

        size_t idx = index_for_namespace(ns);

        if (idx >= local_pos_pubs_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid namespace index for drone: %s", ns.c_str());
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, coords.angle * M_PI / 180.0); // Convert degrees to radians

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = coords.x;
        pose_msg.pose.position.y = coords.y;
        pose_msg.pose.position.z = coords.z;
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish the desired pose
        local_pos_pubs_[idx]->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Command sent to move drone %s to x=%.2f, y=%.2f, z=%.2f", 
                    ns.c_str(), coords.x, coords.y, coords.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiDroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
