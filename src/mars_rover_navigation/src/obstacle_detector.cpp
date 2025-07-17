#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("obstacle_detector")
    {
        // Declare parameters
        this->declare_parameter("laser_obstacle_threshold", 1.5);
        this->declare_parameter("vision_obstacle_threshold", 0.3);
        this->declare_parameter("emergency_stop_distance", 0.8);
        this->declare_parameter("warning_distance", 2.0);
        this->declare_parameter("detection_frequency", 10.0);
        
        laser_threshold_ = this->get_parameter("laser_obstacle_threshold").as_double();
        vision_threshold_ = this->get_parameter("vision_obstacle_threshold").as_double();
        emergency_distance_ = this->get_parameter("emergency_stop_distance").as_double();
        warning_distance_ = this->get_parameter("warning_distance").as_double();
        detection_frequency_ = this->get_parameter("detection_frequency").as_double();

        // Initialize subscribers
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/mars_rover/scan", 10,
            std::bind(&ObstacleDetector::laser_callback, this, std::placeholders::_1));

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/mars_rover/camera/image_raw", 10,
            std::bind(&ObstacleDetector::image_callback, this, std::placeholders::_1));

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ObstacleDetector::cmd_vel_callback, this, std::placeholders::_1));

        // Initialize publishers
        safe_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mars_rover/cmd_vel", 10);
            
        obstacle_alert_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mars_rover/obstacle_detected", 10);
            
        detection_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/mars_rover/detections", 10);

        debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/mars_rover/debug_image", 10);

        // Initialize timer for processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / detection_frequency_)),
            std::bind(&ObstacleDetector::process_detections, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Detector initialized");
        RCLCPP_INFO(this->get_logger(), "Laser threshold: %.2f m", laser_threshold_);
        RCLCPP_INFO(this->get_logger(), "Emergency stop distance: %.2f m", emergency_distance_);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_laser_scan_ = *msg;
        has_laser_data_ = true;
        
        // Process laser data for obstacles
        process_laser_obstacles();
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            latest_image_ = cv_ptr->image;
            has_image_data_ = true;
            
            // Process image for visual obstacles
            process_visual_obstacles();
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_cmd_vel_ = *msg;
        has_cmd_vel_ = true;
    }

    void process_laser_obstacles()
    {
        if (!has_laser_data_) return;

        laser_obstacles_.clear();
        min_laser_distance_ = std::numeric_limits<double>::max();
        
        // Analyze laser scan for obstacles
        for (size_t i = 0; i < latest_laser_scan_.ranges.size(); ++i)
        {
            float range = latest_laser_scan_.ranges[i];
            
            if (range >= latest_laser_scan_.range_min && 
                range <= latest_laser_scan_.range_max)
            {
                float angle = latest_laser_scan_.angle_min + i * latest_laser_scan_.angle_increment;
                
                // Focus on front-facing obstacles (±60 degrees)
                if (fabs(angle) <= M_PI/3)
                {
                    if (range < laser_threshold_)
                    {
                        ObstacleInfo obstacle;
                        obstacle.distance = range;
                        obstacle.angle = angle;
                        obstacle.x = range * cos(angle);
                        obstacle.y = range * sin(angle);
                        obstacle.type = ObstacleType::LASER;
                        
                        laser_obstacles_.push_back(obstacle);
                        
                        if (range < min_laser_distance_)
                        {
                            min_laser_distance_ = range;
                        }
                    }
                }
            }
        }
    }

    void process_visual_obstacles()
    {
        if (!has_image_data_) return;

        visual_obstacles_.clear();
        
        // Convert to HSV for better rock detection
        cv::Mat hsv_image;
        cv::cvtColor(latest_image_, hsv_image, cv::COLOR_BGR2HSV);
        
        // Define rock color ranges (brownish/reddish colors typical of Mars)
        cv::Scalar lower_rock1(0, 30, 30);    // Lower red-brown
        cv::Scalar upper_rock1(20, 255, 200);
        cv::Scalar lower_rock2(160, 30, 30);  // Upper red-brown
        cv::Scalar upper_rock2(180, 255, 200);
        
        cv::Mat mask1, mask2, rock_mask;
        cv::inRange(hsv_image, lower_rock1, upper_rock1, mask1);
        cv::inRange(hsv_image, lower_rock2, upper_rock2, mask2);
        cv::bitwise_or(mask1, mask2, rock_mask);
        
        // Morphological operations to clean up mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(rock_mask, rock_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(rock_mask, rock_mask, cv::MORPH_OPEN, kernel);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(rock_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Create debug image
        cv::Mat debug_image = latest_image_.clone();
        
        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 500) // Minimum area threshold
            {
                cv::Rect bounding_rect = cv::boundingRect(contour);
                
                // Estimate distance based on object size (simple heuristic)
                double estimated_distance = estimate_distance_from_size(area);
                
                if (estimated_distance < vision_threshold_ * 10) // Within detection range
                {
                    ObstacleInfo obstacle;
                    obstacle.distance = estimated_distance;
                    obstacle.angle = calculate_angle_from_pixel(bounding_rect.x + bounding_rect.width/2);
                    obstacle.x = estimated_distance * cos(obstacle.angle);
                    obstacle.y = estimated_distance * sin(obstacle.angle);
                    obstacle.type = ObstacleType::VISUAL;
                    obstacle.confidence = calculate_confidence(area, bounding_rect);
                    
                    visual_obstacles_.push_back(obstacle);
                    
                    // Draw on debug image
                    cv::rectangle(debug_image, bounding_rect, cv::Scalar(0, 255, 0), 2);
                    cv::putText(debug_image, 
                               "Dist: " + std::to_string(estimated_distance).substr(0, 4) + "m",
                               cv::Point(bounding_rect.x, bounding_rect.y - 10),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
            }
        }
        
        // Publish debug image
        publish_debug_image(debug_image);
        
        // Create and publish detections message
        publish_detections();
    }

    double estimate_distance_from_size(double area)
    {
        // Simple heuristic: larger objects are closer
        // This is a rough approximation and would need calibration for real use
        const double reference_area = 2000.0; // Reference area at 1 meter
        const double reference_distance = 1.0;
        
        return reference_distance * sqrt(reference_area / area);
    }

    double calculate_angle_from_pixel(int pixel_x)
    {
        // Convert pixel position to angle
        // Assuming 60-degree field of view
        const double fov = M_PI / 3; // 60 degrees
        const double image_width = latest_image_.cols;
        
        double normalized_x = (pixel_x - image_width/2) / (image_width/2);
        return normalized_x * fov / 2;
    }

    double calculate_confidence(double area, const cv::Rect& rect)
    {
        // Simple confidence calculation based on size and aspect ratio
        double aspect_ratio = static_cast<double>(rect.width) / rect.height;
        double size_confidence = std::min(1.0, area / 5000.0);
        double shape_confidence = 1.0 - fabs(aspect_ratio - 1.0); // Prefer square-ish objects
        
        return (size_confidence + shape_confidence) / 2.0;
    }

    void publish_debug_image(const cv::Mat& debug_image)
    {
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = this->get_clock()->now();
        cv_image.header.frame_id = "camera_link";
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = debug_image;
        
        debug_image_publisher_->publish(*cv_image.toImageMsg());
    }

    void publish_detections()
    {
        vision_msgs::msg::Detection2DArray detections_msg;
        detections_msg.header.stamp = this->get_clock()->now();
        detections_msg.header.frame_id = "camera_link";
        
        for (const auto& obstacle : visual_obstacles_)
        {
            vision_msgs::msg::Detection2D detection;
            detection.header = detections_msg.header;
            
            // Set bounding box (simplified)
            detection.bbox.center.position.x = obstacle.x;
            detection.bbox.center.position.y = obstacle.y;
            detection.bbox.size_x = 0.5; // Estimated size
            detection.bbox.size_y = 0.5;
            
            // Add detection result
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = "rock";
            hypothesis.hypothesis.score = obstacle.confidence;
            detection.results.push_back(hypothesis);
            
            detections_msg.detections.push_back(detection);
        }
        
        detection_publisher_->publish(detections_msg);
    }

    void process_detections()
    {
        bool obstacle_detected = false;
        bool emergency_stop = false;
        
        // Check laser obstacles
        if (min_laser_distance_ < emergency_distance_)
        {
            emergency_stop = true;
            obstacle_detected = true;
        }
        else if (min_laser_distance_ < warning_distance_)
        {
            obstacle_detected = true;
        }
        
        // Check visual obstacles
        for (const auto& obstacle : visual_obstacles_)
        {
            if (obstacle.distance < emergency_distance_ && obstacle.confidence > 0.5)
            {
                emergency_stop = true;
                obstacle_detected = true;
                break;
            }
            else if (obstacle.distance < warning_distance_ && obstacle.confidence > 0.3)
            {
                obstacle_detected = true;
            }
        }
        
        // Publish obstacle alert
        std_msgs::msg::Bool alert_msg;
        alert_msg.data = obstacle_detected;
        obstacle_alert_publisher_->publish(alert_msg);
        
        // Process command velocity
        if (has_cmd_vel_)
        {
            geometry_msgs::msg::Twist safe_cmd_vel = latest_cmd_vel_;
            
            if (emergency_stop)
            {
                // Complete stop
                safe_cmd_vel.linear.x = 0.0;
                safe_cmd_vel.linear.y = 0.0;
                safe_cmd_vel.angular.z = 0.0;
                
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Emergency stop! Obstacle at %.2f m", min_laser_distance_);
            }
            else if (obstacle_detected)
            {
                // Reduce speed
                safe_cmd_vel.linear.x *= 0.5;
                safe_cmd_vel.linear.y *= 0.5;
                
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Obstacle detected, reducing speed");
            }
            
            safe_cmd_vel_publisher_->publish(safe_cmd_vel);
        }
        
        // Reset minimum distance for next iteration
        min_laser_distance_ = std::numeric_limits<double>::max();
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_alert_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    
    rclcpp::TimerBase::SharedPtr processing_timer_;

    // Data storage
    sensor_msgs::msg::LaserScan latest_laser_scan_;
    cv::Mat latest_image_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    
    enum class ObstacleType { LASER, VISUAL };
    
    struct ObstacleInfo {
        double distance;
        double angle;
        double x, y;
        ObstacleType type;
        double confidence = 1.0;
    };
    
    std::vector<ObstacleInfo> laser_obstacles_;
    std::vector<ObstacleInfo> visual_obstacles_;
    
    // State flags
    bool has_laser_data_ = false;
    bool has_image_data_ = false;
    bool has_cmd_vel_ = false;
    
    // Processing variables
    double min_laser_distance_;
    
    // Parameters
    double laser_threshold_;
    double vision_threshold_;
    double emergency_distance_;
    double warning_distance_;
    double detection_frequency_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    
    RCLCPP_INFO(node->get_logger(), "Starting obstacle detection...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
