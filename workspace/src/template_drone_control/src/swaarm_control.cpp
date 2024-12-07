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
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class MultiDroneControl : public rclcpp::Node
{
public:

    struct Coords {
            double x;
            double y;
            double z;
            double angle;
    };

    MultiDroneControl() : Node("multi_drone_control_node")
    {
        // Define namespaces for the drones
        drone_namespaces_ = {"drone1", "drone2", "drone3"};

        for (const auto &ns : drone_namespaces_)
        {
            RCLCPP_INFO(this->get_logger(), "Setting up drone with namespace: %s", ns.c_str());

            // Create subscribers, publishers, and clients for each drone
            state_subs_.push_back(
                this->create_subscription<mavros_msgs::msg::State>(
                    "/" + ns + "/state", 10,
                    [this, ns](mavros_msgs::msg::State::SharedPtr msg) {
                        current_states_[ns] = *msg;
                    }));

            local_pos_pubs_.push_back(
                this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + ns + "/setpoint_position/local", 10));

            local_pos_subs_.push_back(
                this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/" + ns + "/local_position/pose", 10,
                    [this, ns](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        local_pos_cb(msg, ns);
                    }));

            arming_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandBool>("/" + ns + "/cmd/arming"));

            set_mode_clients_.push_back(
                this->create_client<mavros_msgs::srv::SetMode>("/" + ns + "/set_mode"));

            takeoff_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandTOL>("/" + ns + "/cmd/takeoff"));
        }


        Coords droneCoords;

        // Connect to all drones, set mode, arm, and take off
        for (const auto &ns : drone_namespaces_)
        {
            wait_for_connection(ns);
            set_mode(ns, "GUIDED");
            arm_drone(ns);
            takeoff(ns, 3.0, 0.0); // Takeoff to 3 meters

        }

        // wait_for_positions();

        std::this_thread::sleep_for(6000ms);
        Coords coordsDrone1;
        // Coords coordsDrone2;
        // Coords coordsDrone3;
        coordsDrone1.x = 0;
        coordsDrone1.y = 1;
        coordsDrone1.z = 3;
        coordsDrone1.angle = 0;

        move_drone(drone_namespaces_[0],coordsDrone1);
        std::this_thread::sleep_for(2000ms);
        auto [coordsDrone2, coordsDrone3] = calculateFollowerPositions(coordsDrone1, coordsDrone1.angle);
        move_drone(drone_namespaces_[1],coordsDrone2);
        std::this_thread::sleep_for(2000ms);
        move_drone(drone_namespaces_[2],coordsDrone3);
        // coordsDrone2.x = 2;
        // coordsDrone2.y = 2;
        // coordsDrone2.z = 3;

        // coordsDrone3.x = 3;
        // coordsDrone3.y = 2;
        // coordsDrone3.z = 3;
        // move_drone(drone_namespaces_[0],coordsDrone1);
        // move_drone(drone_namespaces_[1],coordsDrone2);
        // move_drone(drone_namespaces_[2],coordsDrone3);
        // for (const auto &ns : drone_namespaces_)
        // {
        //     // droneCoords = get_coords(ns);

        // }
        

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

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &ns) {
        current_positions_[ns] = *msg;
        RCLCPP_INFO(this->get_logger(), "Updated position for %s: x=%.2f, y=%.2f, z=%.2f", 
                    ns.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }


    void wait_for_positions()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for position updates...");
        while (rclcpp::ok())
        {
            bool all_positions_received = true;
            for (const auto &ns : drone_namespaces_)
            {
                if (current_positions_.find(ns) == current_positions_.end())
                {
                    all_positions_received = false;
                    break;
                }
            }

            if (all_positions_received)
            {
                RCLCPP_INFO(this->get_logger(), "All drone positions received.");
                break;
            }

            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
    }
    

    std::pair<Coords, Coords> calculateFollowerPositions(const Coords &coords, double angle) {
        // Circle radius (half of the diameter)
        const double radius = 1;

        // Angles for the follower drones in degrees
        const double drone1AngleOffset = 225.0 +90; // Drone 1 offset angle
        const double drone2AngleOffset = 135.0 +90; // Drone 2 offset angle

        // Convert angles from degrees to radians
        double angleRad = angle * M_PI / 180.0;
        double drone1AngleRad = (angle + drone1AngleOffset) * M_PI / 180.0;
        double drone2AngleRad = (angle + drone2AngleOffset) * M_PI / 180.0;

        // Calculate coordinates for drone 1
        Coords drone1;
        drone1.x = coords.x + radius * cos(drone1AngleRad);
        drone1.y = coords.y + radius * sin(drone1AngleRad);
        drone1.z = coords.z; // Same altitude as the leading drone
        drone1.angle = coords.angle;

        // Calculate coordinates for drone 2
        Coords drone2;
        drone2.x = coords.x + radius * cos(drone2AngleRad);
        drone2.y = coords.y + radius * sin(drone2AngleRad);
        drone2.z = coords.z; // Same altitude as the leading drone
        drone2.angle = coords.angle;

        // Return the pair of follower drone coordinates
        return std::make_pair(drone1, drone2);
    }


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
        q.setRPY(0, 0, coords.angle);

        auto pose_msg = geometry_msgs::msg::PoseStamped();
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

        // Optionally, wait for the drone to reach the target position
        // rclcpp::Rate rate(5); // 5 Hz
        // while (rclcpp::ok()) {
        //     rclcpp::spin_some(this->get_node_base_interface());

        //     // Check if the drone is at the desired position
        //     const auto &current_pos = current_positions_[ns].pose.position;
        //     RCLCPP_INFO(this->get_logger(), "Drone %s is on coordinates: x=%.2f, y=%.2f, z=%.2f", 
        //         ns.c_str(), current_pos.x, current_pos.y, current_pos.z);

        //     if (std::fabs(current_pos.x - coords.x) < 0.2 &&
        //         std::fabs(current_pos.y - coords.y) < 0.2 &&
        //         std::fabs(current_pos.z - coords.z) < 0.2) {
        //         RCLCPP_INFO(this->get_logger(), "Drone %s reached the target position.", ns.c_str());
        //         break;
        //     }

        //     rate.sleep();
        // }
    }



    Coords get_coords(const std::string &ns){
        Coords droneCoords;
        int i = 0;
        rclcpp::Rate rate(5.0);


        // while(rclcpp::ok()) {
        //     rclcpp::spin_some(this->get_node_base_interface());
        //     // rate.sleep();
        //     auto drone = current_positions_[index_for_namespace(ns)];
        //     droneCoords.x = drone.pose.position.x;
        //     droneCoords.y = drone.pose.position.y;
        //     droneCoords.z = drone.pose.position.z;
        //     // droneCoords.x = current_positions_[ns].pose.position.x;
        //     // droneCoords.y = current_positions_[ns].pose.position.y;
        //     // droneCoords.z = current_positions_[ns].pose.position.z;
        //     RCLCPP_INFO(this->get_logger(), "Drone %s coords are: %.6f, %.6f, %.6f", ns.c_str(),current_positions_[index_for_namespace(ns)].pose.position.x,current_positions_[index_for_namespace(ns)].pose.position.y,current_positions_[index_for_namespace(ns)].pose.position.z);
        //     RCLCPP_INFO(this->get_logger(), "Drone %s coords are: %.6f, %.6f, %.6f", ns.c_str(),droneCoords.x,droneCoords.y,droneCoords.z);
        //     // RCLCPP_INFO(this->get_logger(), "Drone %s coords are: %.6f, %.6f, %.6f", ns.c_str(),droneCoords.x,droneCoords.y,droneCoords.z);
        //     // RCLCPP_INFO(this->get_logger(), "Drone %s coords are: %.6f, %.6f, %.6f", ns.c_str(),current_positions_[ns].pose.position.x,current_positions_[ns].pose.position.y,current_positions_[ns].pose.position.z);
        
        //     if (i == 20) break;
        //     i += 1;
        // }

        return droneCoords;


    }


    void wait_for_connection(const std::string &ns)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for connection to drone: %s", ns.c_str());
        while (rclcpp::ok() && !current_states_[ns].connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Connected to drone: %s", ns.c_str());
    }

    void set_mode(const std::string &ns, const std::string &mode)
    {
        auto client = set_mode_clients_[index_for_namespace(ns)];
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

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Mode set to %s for drone: %s", mode.c_str(), ns.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode for drone: %s", ns.c_str());
        }
    }

    void arm_drone(const std::string &ns)
    {
        auto client = arming_clients_[index_for_namespace(ns)];
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

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Drone armed: %s", ns.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone: %s", ns.c_str());
        }
    }

    void takeoff(const std::string &ns, double altitude, double yaw)
    {
        auto client = takeoff_clients_[index_for_namespace(ns)];
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

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff command sent for drone: %s", ns.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff for drone: %s", ns.c_str());
        }
    }

    void land_drone(const std::string &ns)
    {
        auto client = set_mode_clients_[index_for_namespace(ns)];
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

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Landing command sent for drone: %s", ns.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to land drone: %s", ns.c_str());
        }
    }

    size_t index_for_namespace(const std::string &ns)
    {
        auto it = std::find(drone_namespaces_.begin(), drone_namespaces_.end(), ns);
        return std::distance(drone_namespaces_.begin(), it);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiDroneControl>());
    rclcpp::shutdown();
    return 0;
}