#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        // Set mode to GUIDED
        set_mode("GUIDED");

        // Arm the drone
        arm_drone();

        // Takeoff
        takeoff(2.0, 90.0);

        std::this_thread::sleep_for(9000ms);

        RCLCPP_INFO(this->get_logger(), "Sending position command");
        // TODO: Implement position controller and mission commands here
        move(-5,0,2,"soft");
    }

    void move(double x, double y, double z, const std::string& finish_type)
    {
        // Desired position
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        double tolerance = (finish_type == "hard") ? 0.2 : 0.5;

        rclcpp::Rate rate(10); // Publish at 10 Hz
        while (rclcpp::ok())
        {
            // Publish the target position
            local_pos_pub_->publish(target_pose);

            // Get the current position
            auto current_pos = current_position_; // Assume current_position_ is updated in local_pos_cb

            // Calculate distance to target
            double dx = current_pos.pose.position.x - x;
            double dy = current_pos.pose.position.y - y;
            double dz = current_pos.pose.position.z - z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Check if the drone is within tolerance of the target
            if (distance <= tolerance)
            {
                RCLCPP_INFO(this->get_logger(), "Reached target position with %s finish", finish_type.c_str());
                break;
            }

            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    // Function to set mode
    void set_mode(const std::string &mode)
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }

        auto result = set_mode_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Mode set to %s", mode.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode to %s", mode.c_str());
        }
    }

    // Function to arm the drone
    void arm_drone()
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }

        auto result = arming_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Drone armed successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone");
        }
    }

    // Function to take off
    void takeoff(double altitude, double yaw)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->min_pitch = 0;
        request->yaw = yaw;
        request->altitude = altitude;

        while (!takeoff_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the takeoff service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
        }

        auto result = takeoff_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff command sent");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff");
        }
    }

    // Callback for drone state updates
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    // Callback for local position updates
    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    // ROS2 subscribers, publishers, and clients
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_position_;
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemplateDroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
