#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <queue>

class WaypointNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    WaypointNavigator() : Node("waypoint_navigator")
    {
        // Declare parameters
        this->declare_parameter("waypoints_file", "");
        this->declare_parameter("loop_waypoints", false);
        this->declare_parameter("waypoint_tolerance", 0.5);
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("retry_delay", 5.0);
        
        waypoints_file_ = this->get_parameter("waypoints_file").as_string();
        loop_waypoints_ = this->get_parameter("loop_waypoints").as_bool();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        max_retries_ = this->get_parameter("max_retries").as_int();
        retry_delay_ = this->get_parameter("retry_delay").as_double();

        // Initialize subscribers
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mars_rover/odom", 10,
            std::bind(&WaypointNavigator::odom_callback, this, std::placeholders::_1));
            
        waypoint_command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/mars_rover/waypoint_command", 10,
            std::bind(&WaypointNavigator::command_callback, this, std::placeholders::_1));

        // Initialize publishers
        waypoint_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/mars_rover/waypoint_status", 10);
            
        waypoints_viz_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/mars_rover/waypoints_viz", 10);

        // Initialize action clients
        navigate_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/navigate_to_pose");
            
        follow_waypoints_action_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "/follow_waypoints");

        // Load waypoints if file is provided
        if (!waypoints_file_.empty())
        {
            load_waypoints_from_file(waypoints_file_);
        }
        else
        {
            // Load default Mars exploration waypoints
            load_default_waypoints();
        }

        RCLCPP_INFO(this->get_logger(), "Waypoint Navigator initialized");
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Loop waypoints: %s", loop_waypoints_ ? "true" : "false");
        
        publish_waypoints_visualization();
    }

    void start_waypoint_navigation()
    {
        if (waypoints_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded!");
            return;
        }

        if (navigation_active_)
        {
            RCLCPP_WARN(this->get_logger(), "Navigation already active");
            return;
        }

        current_waypoint_index_ = 0;
        retry_count_ = 0;
        navigation_active_ = true;
        
        publish_status("Starting waypoint navigation");
        navigate_to_current_waypoint();
    }

    void stop_waypoint_navigation()
    {
        navigation_active_ = false;
        cancel_current_goal();
        publish_status("Waypoint navigation stopped");
    }

    void add_waypoint(const geometry_msgs::msg::PoseStamped& waypoint)
    {
        waypoints_.push_back(waypoint);
        publish_waypoints_visualization();
        RCLCPP_INFO(this->get_logger(), "Added waypoint at (%.2f, %.2f)", 
                   waypoint.pose.position.x, waypoint.pose.position.y);
    }

    void clear_waypoints()
    {
        waypoints_.clear();
        publish_waypoints_visualization();
        RCLCPP_INFO(this->get_logger(), "Cleared all waypoints");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (command == "start")
        {
            start_waypoint_navigation();
        }
        else if (command == "stop")
        {
            stop_waypoint_navigation();
        }
        else if (command == "pause")
        {
            pause_navigation();
        }
        else if (command == "resume")
        {
            resume_navigation();
        }
        else if (command == "next")
        {
            skip_to_next_waypoint();
        }
        else if (command == "reset")
        {
            reset_navigation();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }

    void load_waypoints_from_file(const std::string& filename)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(filename);
            
            if (config["waypoints"])
            {
                for (const auto& wp : config["waypoints"])
                {
                    geometry_msgs::msg::PoseStamped waypoint;
                    waypoint.header.frame_id = "map";
                    waypoint.header.stamp = this->get_clock()->now();
                    
                    waypoint.pose.position.x = wp["x"].as<double>();
                    waypoint.pose.position.y = wp["y"].as<double>();
                    waypoint.pose.position.z = wp["z"].as<double>(0.0);
                    
                    // Convert yaw to quaternion
                    double yaw = wp["yaw"].as<double>(0.0);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    waypoint.pose.orientation = tf2::toMsg(q);
                    
                    waypoints_.push_back(waypoint);
                }
                
                RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", 
                           waypoints_.size(), filename.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from %s: %s", 
                        filename.c_str(), e.what());
            load_default_waypoints();
        }
    }

    void load_default_waypoints()
    {
        // Default Mars exploration waypoints
        std::vector<std::tuple<double, double, double>> default_points = {
            {5.0, 0.0, 0.0},     // Forward
            {5.0, 5.0, M_PI/2},  // Turn right
            {0.0, 5.0, M_PI},    // Turn around
            {-5.0, 5.0, M_PI},   // Continue back
            {-5.0, 0.0, -M_PI/2}, // Turn left
            {-5.0, -5.0, -M_PI/2}, // Continue left
            {0.0, -5.0, 0.0},    // Head back
            {0.0, 0.0, 0.0}      // Return to origin
        };

        for (const auto& point : default_points)
        {
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header.frame_id = "map";
            waypoint.header.stamp = this->get_clock()->now();
            
            waypoint.pose.position.x = std::get<0>(point);
            waypoint.pose.position.y = std::get<1>(point);
            waypoint.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, std::get<2>(point));
            waypoint.pose.orientation = tf2::toMsg(q);
            
            waypoints_.push_back(waypoint);
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu default waypoints", waypoints_.size());
    }

    void navigate_to_current_waypoint()
    {
        if (!navigation_active_ || current_waypoint_index_ >= waypoints_.size())
        {
            if (loop_waypoints_ && !waypoints_.empty())
            {
                current_waypoint_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "Looping back to first waypoint");
            }
            else
            {
                complete_navigation();
                return;
            }
        }

        const auto& waypoint = waypoints_[current_waypoint_index_];
        
        if (!navigate_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoint;

        RCLCPP_INFO(this->get_logger(), 
            "Navigating to waypoint %zu: (%.2f, %.2f)", 
            current_waypoint_index_ + 1,
            waypoint.pose.position.x, 
            waypoint.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&WaypointNavigator::goal_response_callback, this, std::placeholders::_1);
            
        send_goal_options.feedback_callback =
            std::bind(&WaypointNavigator::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            
        send_goal_options.result_callback =
            std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1);

        current_goal_handle_ = navigate_action_client_->async_send_goal(goal_msg, send_goal_options);
        
        publish_status("Navigating to waypoint " + std::to_string(current_waypoint_index_ + 1));
    }

    void goal_response_callback(const GoalHandleNavigate::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            handle_navigation_failure();
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        if (current_waypoint_index_ < waypoints_.size())
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Distance to waypoint %zu: %.2f", 
                current_waypoint_index_ + 1, feedback->distance_remaining);
        }
    }

    void result_callback(const GoalHandleNavigate::WrappedResult& result)
    {
        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_ + 1);
                handle_navigation_success();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Navigation to waypoint %zu aborted", 
                           current_waypoint_index_ + 1);
                handle_navigation_failure();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation to waypoint %zu canceled", 
                           current_waypoint_index_ + 1);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown navigation result code");
                handle_navigation_failure();
                break;
        }
    }

    void handle_navigation_success()
    {
        retry_count_ = 0;
        current_waypoint_index_++;
        
        // Small delay before next waypoint
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                navigate_to_current_waypoint();
            });
        // Timer will be destroyed automatically after one shot
    }

    void handle_navigation_failure()
    {
        retry_count_++;
        
        if (retry_count_ < max_retries_)
        {
            RCLCPP_WARN(this->get_logger(), 
                "Retrying waypoint %zu (attempt %d/%d)", 
                current_waypoint_index_ + 1, retry_count_ + 1, max_retries_);
                
            // Retry after delay
            auto timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(retry_delay_ * 1000)),
                [this]() {
                    navigate_to_current_waypoint();
                });
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to reach waypoint %zu after %d attempts, skipping", 
                current_waypoint_index_ + 1, max_retries_);
            
            retry_count_ = 0;
            current_waypoint_index_++;
            navigate_to_current_waypoint();
        }
    }

    void complete_navigation()
    {
        navigation_active_ = false;
        publish_status("Waypoint navigation completed");
        RCLCPP_INFO(this->get_logger(), "All waypoints completed!");
    }

    void pause_navigation()
    {
        if (navigation_active_)
        {
            cancel_current_goal();
            navigation_paused_ = true;
            publish_status("Navigation paused");
            RCLCPP_INFO(this->get_logger(), "Navigation paused");
        }
    }

    void resume_navigation()
    {
        if (navigation_paused_)
        {
            navigation_paused_ = false;
            navigate_to_current_waypoint();
            publish_status("Navigation resumed");
            RCLCPP_INFO(this->get_logger(), "Navigation resumed");
        }
    }

    void skip_to_next_waypoint()
    {
        if (navigation_active_)
        {
            cancel_current_goal();
            current_waypoint_index_++;
            retry_count_ = 0;
            navigate_to_current_waypoint();
            RCLCPP_INFO(this->get_logger(), "Skipped to next waypoint");
        }
    }

    void reset_navigation()
    {
        cancel_current_goal();
        current_waypoint_index_ = 0;
        retry_count_ = 0;
        navigation_active_ = false;
        navigation_paused_ = false;
        publish_status("Navigation reset");
        RCLCPP_INFO(this->get_logger(), "Navigation reset");
    }

    void cancel_current_goal()
    {
        if (current_goal_handle_.valid())
        {
            navigate_action_client_->async_cancel_goal(current_goal_handle_.get());
        }
    }

    void publish_status(const std::string& status)
    {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        waypoint_status_publisher_->publish(status_msg);
    }

    void publish_waypoints_visualization()
    {
        geometry_msgs::msg::PoseArray waypoints_array;
        waypoints_array.header.frame_id = "map";
        waypoints_array.header.stamp = this->get_clock()->now();
        
        for (const auto& waypoint : waypoints_)
        {
            waypoints_array.poses.push_back(waypoint.pose);
        }
        
        waypoints_viz_publisher_->publish(waypoints_array);
    }

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_command_subscriber_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_viz_publisher_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_action_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_action_client_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    geometry_msgs::msg::Pose current_pose_;
    
    std::shared_future<GoalHandleNavigate::SharedPtr> current_goal_handle_;
    
    // Navigation state
    size_t current_waypoint_index_ = 0;
    int retry_count_ = 0;
    bool navigation_active_ = false;
    bool navigation_paused_ = false;
    bool has_odom_ = false;

    // Parameters
    std::string waypoints_file_;
    bool loop_waypoints_;
    double waypoint_tolerance_;
    int max_retries_;
    double retry_delay_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    
    RCLCPP_INFO(node->get_logger(), "Waypoint Navigator ready. Send 'start' command to begin navigation.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
