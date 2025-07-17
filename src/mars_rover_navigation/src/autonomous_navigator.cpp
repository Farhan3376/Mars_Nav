#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class AutonomousNavigator : public rclcpp::Node
{
public:
    AutonomousNavigator() : Node("autonomous_navigator")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&AutonomousNavigator::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Autonomous Navigator started (basic version)");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Mars Rover Navigation System is working!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousNavigator>());
    rclcpp::shutdown();
    return 0;
}
