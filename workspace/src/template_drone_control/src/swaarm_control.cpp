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


    MultiDroneControl() : Node("multi_drone_control_node")
    {
        // Define namespaces for the drones
        drone_namespaces_ = {"drone1", "drone2", "drone3"};

        for (const auto &ns : drone_namespaces_)
        {
            RCLCPP_INFO(this->get_logger(), "Setting up drone with namespace: %s", ns.c_str());

            // State Topic Subscription
            std::string state_topic = "/" + ns + "/state";
            auto state_sub = this->create_subscription<mavros_msgs::msg::State>(
                state_topic, 10,
                [this, ns](mavros_msgs::msg::State::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    current_states_[ns] = *msg;
                });
            state_subs_.push_back(state_sub);

            // Local Position Topic Subscription
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

        // Launch the mission execution in a separate thread
        std::thread(&MultiDroneControl::execute_mission, this).detach();
    }

private:

    std::vector<std::string> drone_namespaces_;
    std::vector<rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr> state_subs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> local_pos_pubs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> local_pos_subs_;
    std::vector<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr> arming_clients_;
    std::vector<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr> set_mode_clients_;
    std::vector<rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr> takeoff_clients_;
    std::map<std::string, mavros_msgs::msg::State> current_states_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> current_positions_;
    std::mutex mutex_; // Mutex for thread-safe access

    // Callback function for local position
    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &ns) 
    {   
        // RCLCPP_INFO(this->get_logger(), "Updated position for %s: x=%.2f, y=%.2f, z=%.2f", 
        //     ns.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // Lock the mutex to ensure thread-safe access to current_positions_
        std::lock_guard<std::mutex> lock(mutex_);
    
        // Update the position for the given namespace
        current_positions_[ns] = *msg;
    }

    // Function to execute mission steps
    void execute_mission()
    {
        // Connect to all drones, set mode, arm, and take off
        for (const auto &ns : drone_namespaces_)
        {
            wait_for_connection(ns);
            set_mode(ns, "GUIDED");
            arm_drone(ns);
            takeoff(ns, 3.0, 0.0); // Takeoff to 3 meters
        }

        // Allow some time for drones to stabilize
        std::this_thread::sleep_for(6s);

        // Define coordinates for mission steps
        Coords coordsDrone1;
        Coords coordsDrone2;
        Coords coordsDrone3;

        // First movement
        coordsDrone1.x = 0;
        coordsDrone1.y = 1;
        coordsDrone1.z = 3;
        coordsDrone1.angle = 0;

        move_drone(drone_namespaces_[0], coordsDrone1);
        std::this_thread::sleep_for(2s);
        std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsDrone1);
        move_drone(drone_namespaces_[1], coordsDrone2);
        std::this_thread::sleep_for(2s);
        move_drone(drone_namespaces_[2], coordsDrone3);
        std::this_thread::sleep_for(2s);

        // Second movement
        coordsDrone1.x = 0;
        coordsDrone1.y = 3;
        coordsDrone1.z = 3;
        coordsDrone1.angle = 0;

        move_drones(coordsDrone1);

        // Third movement
        coordsDrone1.x = 3;
        coordsDrone1.y = 3;
        coordsDrone1.z = 3;
        coordsDrone1.angle = -90;

        // move_drone(drone_namespaces_[0], coordsDrone1);
        // std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsDrone1);
        // move_drone(drone_namespaces_[1], coordsDrone2);
        // move_drone(drone_namespaces_[2], coordsDrone3);
        // std::this_thread::sleep_for(2s);
        move_drones(coordsDrone1);

        // Fourth movement
        coordsDrone1.x = 3;
        coordsDrone1.y = 3;
        coordsDrone1.z = 3;
        coordsDrone1.angle = -180;

        // move_drone(drone_namespaces_[0], coordsDrone1);
        // std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsDrone1);
        // move_drone(drone_namespaces_[1], coordsDrone2);
        // move_drone(drone_namespaces_[2], coordsDrone3);
        // std::this_thread::sleep_for(2s);
        move_drones(coordsDrone1);

        // Fifth movement
        coordsDrone1.x = 3;
        coordsDrone1.y = 0;
        coordsDrone1.z = 3;
        coordsDrone1.angle = -180;

        // move_drone(drone_namespaces_[0], coordsDrone1);
        // std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsDrone1);
        // move_drone(drone_namespaces_[1], coordsDrone2);
        // move_drone(drone_namespaces_[2], coordsDrone3);
        // std::this_thread::sleep_for(2s);
        move_drones(coordsDrone1);

        // Land all drones
        land_drone(drone_namespaces_[0]);
        land_drone(drone_namespaces_[1]);
        land_drone(drone_namespaces_[2]);
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
            // You can add additional checks on the response if needed
            RCLCPP_INFO(this->get_logger(), "Mode set to %s for drone: %s", mode.c_str(), ns.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode for drone: %s", ns.c_str());
        }
    }


    void move_drones(Coords coordsLeadingDrone)
    {
        // Access the position of the leading drone (drone1)
        const auto &drone1 = drone_namespaces_[0];
        const auto &drone2 = drone_namespaces_[1];
        const auto &drone3 = drone_namespaces_[2];
        geometry_msgs::msg::PoseStamped posDrone1;
        geometry_msgs::msg::PoseStamped posDrone2;
        geometry_msgs::msg::PoseStamped posDrone3;
        double tolerance = 0.1;



        // posDrone1 = current_positions_[drone1];

        // RCLCPP_INFO(this->get_logger(), "Current position for %s: x=%.2f, y=%.2f, z=%.2f", 
        //             drone1.c_str(), posDrone1.pose.position.x, posDrone1.pose.position.y, posDrone1.pose.position.z);

        // Move the leading drone
        move_drone(drone1, coordsLeadingDrone);

        // Calculate follower positions based on the leading drone's position
        Coords coordsDrone2, coordsDrone3;
        std::tie(coordsDrone2, coordsDrone3) = calculateFollowerPositions(coordsLeadingDrone);

        // Move follower drones
        move_drone(drone_namespaces_[1], coordsDrone2);
        move_drone(drone_namespaces_[2], coordsDrone3);
        RCLCPP_INFO(this->get_logger(), "This 1");
        rclcpp::Rate rate(15.0);
        while(rclcpp::ok())
        {
        RCLCPP_INFO(this->get_logger(), "This 2");
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        RCLCPP_INFO(this->get_logger(), "This 3");
            posDrone1 = current_positions_[drone1];
        RCLCPP_INFO(this->get_logger(), "This 4");
            RCLCPP_INFO(this->get_logger(), "Position of leading drone : x=%.2f, y=%.2f, z=%.2f", 
                        posDrone1.pose.position.x, posDrone1.pose.position.y, posDrone1.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "This 5");
            // Calculate distance to target
            double dx = posDrone1.pose.position.x - coordsLeadingDrone.x;
            double dy = posDrone1.pose.position.y - coordsLeadingDrone.y;                   
            double distance = std::sqrt(dx * dx + dy * dy);
        RCLCPP_INFO(this->get_logger(), "This 6");

            
            if (distance <= tolerance) {
                break;
            }
        }



        // std::this_thread::sleep_for(2s); // Allow time for movement
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
            if (!rclcpp::ok())
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
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone: %s", ns.c_str());
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
            if (!rclcpp::ok())
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
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff for drone: %s", ns.c_str());
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
    rclcpp::spin(node); // Spin the node once
    rclcpp::shutdown();
    return 0;
}
