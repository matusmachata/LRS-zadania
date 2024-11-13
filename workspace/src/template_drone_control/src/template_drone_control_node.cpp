#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <optional>
#include <cctype>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



// TODO
// flip map before generating path
// add path from drone [13,1] to first waypoint
// figure out wheere the fuck the drone going

// when going to given waypoint, go with given accuracy (soft/hard pp)
// when on given point check altitude and go up/down accordingly

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
        takeoff(2.0, 0);

        // std::this_thread::sleep_for(9000ms);

        // Example waypoints file
        std::string homeDir = std::getenv("HOME");
        // std::string filePath = homeDir + "/Documents/GitHub/LRS-zadania/pathfinder/path.txt";

        std::string path_file = homeDir + "/Documents/GitHub/LRS-zadania/pathfinder/all_paths.txt";
        std::string waypoints_file = homeDir + "/Documents/GitHub/LRS-zadania/pathfinder/waypoints.csv";
        move(path_file, waypoints_file);
        // geometry_msgs::msg::PoseStamped target_pose;

        // Point2D point;
        // point.x = 13.55;
        // point.y = 8.30;
        // point = transformPoint(point);
        // target_pose.pose.position.x = point.x;
        // target_pose.pose.position.y = point.y;
        // target_pose.pose.position.z = 2;
        // local_pos_pub_->publish(target_pose);
    }

    struct Point2D {
        double x;
        double y;
    };

    struct Waypoints {
        double x;
        double y;
        double z;
        std::string accuracy;
        std::string additionalCommand;
    };

    // Method to read waypoints from file
    std::vector<std::vector<Point2D>> readPath(const std::string& filename) {
        std::ifstream file(filename);
        std::vector<std::vector<Point2D>> waypoints;

        // Check if the file opened successfully
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return waypoints;
        }

        // Check if the file is empty
        file.seekg(0, std::ios::end);
        if (file.tellg() == 0) {
            std::cerr << "Error: File " << filename << " is empty" << std::endl;
            return waypoints;
        }
        file.seekg(0, std::ios::beg);

        std::vector<Point2D> current_path;
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty()) {
                if (!current_path.empty()) {
                    waypoints.push_back(current_path);
                    current_path.clear();
                }
            } else {
                std::istringstream ss(line);
                Point2D point;
                char comma;
                ss >> point.x >> comma >> point.y;
                current_path.push_back(point);
            }
        }
        if (!current_path.empty()) {
            waypoints.push_back(current_path);
        }
        return waypoints;
    }

    std::vector<Waypoints> readWaypointsCSV(const std::string& filename) {
        std::ifstream file(filename);
        std::vector<Waypoints> waypoints;

        // Check if the file opened successfully
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return waypoints;
        }

        // Check if the file is empty
        file.seekg(0, std::ios::end);
        if (file.tellg() == 0) {
            std::cerr << "Error: File " << filename << " is empty" << std::endl;
            return waypoints;
        }
        file.seekg(0, std::ios::beg);

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            Waypoints point;
            char comma;

            // Read x, y, z as doubles, followed by accuracy and additionalCommand as strings
            if ((ss >> point.x >> comma >> point.y >> comma >> point.z >> comma) &&
                std::getline(ss, point.accuracy, ',') &&
                std::getline(ss, point.additionalCommand)) {
                waypoints.push_back(point);
            } else {
                std::cerr << "Error: Incorrect file format in line: " << line << std::endl;
            }
        }

        return waypoints;
    }

    Point2D transformPoint(const Point2D& point) {

    double newX = point.x - 14.035;
    double newY = point.y - 1.514;

    Point2D transformedPoint;
    transformedPoint.x = -newY;
    transformedPoint.y = newX;

    return transformedPoint;
    }

    struct DecodedCommand {
        std::string command;
        double value; 
    };


    DecodedCommand decodeCommand(const std::string& input) {
        DecodedCommand result;

        // Check for the "-" case, which should return null for both command and value
        if (input == "-") {
            result.command = "-";
            result.value = 0;
        }
        // Type 1 commands
        else if (input == "takeoff" || input == "land" || input == "landtakeoff") {
            result.command = input;
            result.value = 0;
        } 
        // Type 2 commands
        else {
            // Find the first digit position in the string to separate command and value
            size_t numStart = 0;
            while (numStart < input.size() && !std::isdigit(input[numStart])) {
                ++numStart;
            }

            if (numStart < input.size()) {
                result.command = input.substr(0, numStart); // Command part (e.g., "yaw")
                result.value = std::stod(input.substr(numStart)); // Numeric part as double (e.g., 180)
            } else {
                // If no numeric part found, treat as an invalid command
                result.command = "Invalid";
                result.value = 0;
            }
        }

        return result;
    }

    void land_drone(){
            // Set the mode to LAND
            set_mode("LAND");

            // Wait until the drone has landed (current altitude reaches close to zero)
            rclcpp::Rate rate(1.0);
            while (rclcpp::ok() && current_position_.pose.position.z > 0.1)
            {
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
                RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_position_.pose.position.z);
            }

            RCLCPP_INFO(this->get_logger(), "Drone has successfully landed.");
    }

    void move(const std::string& pathFile, const std::string& waypointsFile) {
        auto wholePath = readPath(pathFile);
        double tolerance;
        auto allWaypoints = readWaypointsCSV(waypointsFile);
        bool isOnTarget = false;
        int waypointsIndex = 0;
        double z = 2;
        // RCLCPP_INFO(this->get_logger(), "Print no 0 (%.2f)", waypoints.);

        for (const auto& path : wholePath) {
            Waypoints curWaypoint = allWaypoints[waypointsIndex];
            tolerance = (curWaypoint.accuracy == "hard") ? 0.2 : 0.5;
            Point2D curWaypointTransformed;
            curWaypointTransformed.x = curWaypoint.x;
            curWaypointTransformed.y = curWaypoint.y;
            curWaypointTransformed = transformPoint(curWaypointTransformed);
            int waypointIndex = 0;
            for (const auto& waypoint : path) {
                rclcpp::Rate rate(15.0);

                geometry_msgs::msg::PoseStamped target_pose;
                Point2D transformedPoint;
                transformedPoint = transformPoint(waypoint);
                // transformedPoint = waypoint;

                target_pose.pose.position.x = transformedPoint.x;
                target_pose.pose.position.y = transformedPoint.y;
                target_pose.pose.position.z = z;

                // RCLCPP_INFO(this->get_logger(), "map orientation target (%.2f, %.2f)", waypoint.x, waypoint.y);
                // RCLCPP_INFO(this->get_logger(), "drone orientation target (%.2f, %.2f)", transformedPoint.x, transformedPoint.y);
                // RCLCPP_INFO(this->get_logger(), "current waypoint data (%.2f, %.2f, %.2f, %s, %s)", curWaypoint.x, curWaypoint.y, curWaypoint.z, curWaypoint.accuracy.c_str(), curWaypoint.additionalCommand.c_str());
                // RCLCPP_INFO(this->get_logger(), "waypoint target transformed (%.2f, %.2f)", curWaypointTransformed.x, curWaypointTransformed.y);
                local_pos_pub_->publish(target_pose);
                while(rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    rate.sleep();

                    auto current_pos = current_position_; 
                    
                    // RCLCPP_INFO(this->get_logger(), "ros curr position (%.2f, %.2f)", current_pos.pose.position.x, current_pos.pose.position.y);

                    // Calculate distance to target
                    double dx = current_pos.pose.position.x - transformedPoint.x;
                    double dy = current_pos.pose.position.y - transformedPoint.y;                   
                    double distance = std::sqrt(dx * dx + dy * dy);

                    double dxFinal = current_pos.pose.position.x - curWaypointTransformed.x;
                    double dyFinal = current_pos.pose.position.y - curWaypointTransformed.y;
                    double distanceFinal = std::sqrt(dxFinal * dxFinal + dyFinal * dyFinal);
                    // RCLCPP_INFO(this->get_logger(), "distance to waypoint (%.2f)", distanceFinal);
                    if (distanceFinal <= tolerance){
                        // RCLCPP_INFO(this->get_logger(), "isOnTarget je true =============");
                        isOnTarget = true;
                        break;                        
                    }
                    
                    if (distance <= tolerance) {
                        // RCLCPP_INFO(this->get_logger(), "Reached waypoint map orientation (%.2f, %.2f) with %s finish", curWaypoint.x, curWaypoint.y, curWaypoint.accuracy.c_str());
                        break;
                    }
                }

                if (waypointIndex == (path.size() - 1)) isOnTarget = true;

                RCLCPP_INFO(this->get_logger(), "waypointindex (%d) pathsize (%d)", waypointIndex, (path.size() - 1));

                if (isOnTarget){
                    RCLCPP_INFO(this->get_logger(), "podmienka na stupanie =============");
                    auto current_pos = current_position_; 
                    double zTemp = current_pos.pose.position.z;
                    target_pose.pose.position.x = current_pos.pose.position.x;
                    target_pose.pose.position.y = current_pos.pose.position.y;
                    double height;

                    for (int i = 0; i < 2; i++){
                        if (i == 0){
                            target_pose.pose.position.z = curWaypoint.z;
                            height = curWaypoint.z; 
                            RCLCPP_INFO(this->get_logger(), "tam ============="); 
                        }
                        else{
                            target_pose.pose.position.z = zTemp;
                            height = zTemp;
                            RCLCPP_INFO(this->get_logger(), "spat =============");
                        }
                        local_pos_pub_->publish(target_pose);
                        while(rclcpp::ok()){
                            rclcpp::spin_some(this->get_node_base_interface());
                            rate.sleep();

                            current_pos = current_position_;
                            double dz = current_pos.pose.position.z - height;
                            double distacneZ = std::sqrt(dz * dz);
                            // RCLCPP_INFO(this->get_logger(), "distance to waypoint (%.2f)", distacneZ);
                            if (distacneZ <=tolerance) break; 
                        }
                    }
                    DecodedCommand command = decodeCommand(curWaypoint.additionalCommand);
                    RCLCPP_INFO(this->get_logger(), "command from csv %s ", command.command.c_str());

                    if (command.command == "-"){
                        break;
                    }
                    else if (command.command == "land"){
                        RCLCPP_INFO(this->get_logger(), "land =============");
                        land_drone();
                        break;
                    }
                    else if (command.command == "landtakeoff"){
                        land_drone();
                        set_mode("GUIDED");
                        arm_drone();
                        takeoff(2,0);
                        std::this_thread::sleep_for(50ms);
                        break;
                    }
                    else{
                        turn(command.command,command.value);
                        break;
                    }



                }
                waypointIndex++;
            }
            RCLCPP_INFO(this->get_logger(), "Reached end of current path segment.");
            isOnTarget = false;
            waypointsIndex++;
        }
        RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
    }

    void turn(std::string turnType, double angle) {
        geometry_msgs::msg::PoseStamped target_pose;
        auto current_pos = current_position_;
        rclcpp::Rate rate(15.0);
        double angle_tolerance = 0.1;
        
        // Convert angle to radians
        angle = angle * M_PI / 180.0;
        double initial_angle = angle;

        // Set initial target position to hold current position
        target_pose.pose.position.x = current_pos.pose.position.x;
        target_pose.pose.position.y = current_pos.pose.position.y;
        target_pose.pose.position.z = current_pos.pose.position.z; 

        // Create a quaternion for orientation based on the specified axis and angle
        tf2::Quaternion q;
        if (turnType == "roll") {
            q.setRPY(angle, 0, 0);
        } else if (turnType == "pitch") {
            q.setRPY(0, angle, 0);
        } else if (turnType == "yaw") {
            q.setRPY(0, 0, angle);
        } else {
            q.setRPY(0, 0, 0);
        }

        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();

        // Publish the target orientation
        local_pos_pub_->publish(target_pose);

        // Wait until the drone has achieved the target orientation
        while(rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();

            current_pos = current_position_;

            // Get current orientation quaternion and convert to roll, pitch, yaw
            tf2::Quaternion q_current(
                current_pos.pose.orientation.x,
                current_pos.pose.orientation.y,
                current_pos.pose.orientation.z,
                current_pos.pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

            // Calculate the difference between the current and target angle for the specified axis
            double angle_diff;
            if (turnType == "roll") {
                angle_diff = std::fabs(roll - angle);
            } else if (turnType == "pitch") {
                angle_diff = std::fabs(pitch - angle);
            } else if (turnType == "yaw") {
                angle_diff = std::fabs(yaw - angle);
            }

            // Check if the difference is within an acceptable tolerance
            const double angle_tolerance = 0.01; // radians
            if (angle_diff <= angle_tolerance) break;
        }

        // Return to the initial orientation
        tf2::Quaternion q_return;
        q_return.setRPY(0, 0, 0);  // Reset to initial orientation (assuming it was 0)

        target_pose.pose.orientation.x = q_return.x();
        target_pose.pose.orientation.y = q_return.y();
        target_pose.pose.orientation.z = q_return.z();
        target_pose.pose.orientation.w = q_return.w();
        local_pos_pub_->publish(target_pose);

        // Wait until the drone has returned to the original orientation
        while(rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();

            current_pos = current_position_;
            tf2::Quaternion q_current(
                current_pos.pose.orientation.x,
                current_pos.pose.orientation.y,
                current_pos.pose.orientation.z,
                current_pos.pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

            // Calculate the angle difference to the initial orientation
            double angle_diff_return = std::fabs(roll) + std::fabs(pitch) + std::fabs(yaw);
            if (angle_diff_return <= angle_tolerance) break;
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
        double tolerance = 0.15;
        auto current_pos = current_position_; 
        rclcpp::Rate rate(15.0);

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
            while(rclcpp::ok()){
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();

                current_pos = current_position_;
                double dz = current_pos.pose.position.z - altitude;
                double distacneZ = std::sqrt(dz * dz);
                // RCLCPP_INFO(this->get_logger(), "distance to waypoint (%.2f)", distacneZ);
                if (distacneZ <=tolerance) break; 
            }
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}
