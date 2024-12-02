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

using namespace std::chrono_literals;

class MultiDroneControl : public rclcpp::Node
{
public:

    struct Coords {
            double x;
            double y;
            double z;
    };

    MultiDroneControl() : Node("multi_drone_control_node")
    {
        // Define namespaces for the drones
        drone_namespaces_ = {"/drone1", "/drone2", "/drone3"};

        for (const auto &ns : drone_namespaces_)
        {
            RCLCPP_INFO(this->get_logger(), "Setting up drone with namespace: %s", ns.c_str());

            // Create subscribers, publishers, and clients for each drone
            state_subs_.push_back(
                this->create_subscription<mavros_msgs::msg::State>(
                    ns + "/state", 10,
                    [this, ns](mavros_msgs::msg::State::SharedPtr msg) {
                        current_states_[ns] = *msg;
                    }));

            local_pos_pubs_.push_back(
                this->create_publisher<geometry_msgs::msg::PoseStamped>(ns + "/setpoint_position/local", 10));

            local_pos_subs_.push_back(
                this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    ns + "/mavros/local_position/pose", 10,
                    [this, ns](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        local_pos_cb(msg, ns);
                    }));

            arming_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandBool>(ns + "/cmd/arming"));

            set_mode_clients_.push_back(
                this->create_client<mavros_msgs::srv::SetMode>(ns + "/set_mode"));

            takeoff_clients_.push_back(
                this->create_client<mavros_msgs::srv::CommandTOL>(ns + "/cmd/takeoff"));
        }

        // struct Coords {
        //     double x;
        //     double y;
        //     double z;
        // };

        Coords droneCoords;

        // Connect to all drones, set mode, arm, and take off
        for (const auto &ns : drone_namespaces_)
        {
            wait_for_connection(ns);
            set_mode(ns, "GUIDED");
            arm_drone(ns);
            takeoff(ns, 3.0, 0.0); // Takeoff to 3 meters

        }

        std::this_thread::sleep_for(2000ms);
        for (const auto &ns : drone_namespaces_)
        {
            droneCoords = get_coords(ns);
        }
        

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

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &ns)
    {
        current_positions_[ns] = *msg;
    }

    Coords get_coords(const std::string &ns){
        Coords droneCoords;
        droneCoords.x = current_positions_[ns].pose.position.x;
        droneCoords.y = current_positions_[ns].pose.position.y;
        droneCoords.z = current_positions_[ns].pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Drone %s coords are: %.2f, %.2f, %.2f", ns.c_str(),droneCoords.x,droneCoords.y,droneCoords.z);
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
