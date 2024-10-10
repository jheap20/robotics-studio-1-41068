#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>

class cylinderDetector : public rclcpp::Node
{
public:
    cylinderDetector() : Node("cylinderDetector")
    {
        // Subscribe to laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&cylinderDetector::scan_callback, this, std::placeholders::_1));

        // Publisher for cylinder position
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("cylinder_position", 10);

        // Known cylinder diameter
        cylinder_diameter_ = 0.3; // 30 cm
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<float> ranges = msg->ranges;
        float angle_increment = msg->angle_increment;
        float angle_min = msg->angle_min;

        // Find consecutive points that match the cylinder's curvature
        std::vector<size_t> cylinder_points;
        for (size_t i = 1; i < ranges.size() - 1; ++i)
        {
            if (is_part_of_cylinder(ranges[i - 1], ranges[i], ranges[i + 1], angle_increment))
            {
                cylinder_points.push_back(i);
            }
        }

        // If we found enough points, calculate the cylinder's position
        if (cylinder_points.size() >= 3)
        {
            float x_sum = 0, y_sum = 0;
            for (size_t i : cylinder_points)
            {
                float angle = angle_min + i * angle_increment;
                x_sum += ranges[i] * cos(angle);
                y_sum += ranges[i] * sin(angle);
            }

            geometry_msgs::msg::Point cylinder_position;
            cylinder_position.x = x_sum / cylinder_points.size();
            cylinder_position.y = y_sum / cylinder_points.size();
            cylinder_position.z = 0.0;

            publisher_->publish(cylinder_position);
            RCLCPP_INFO(this->get_logger(), "Cylinder detected at (%.2f, %.2f)", 
                        cylinder_position.x, cylinder_position.y);
        }
    }

    bool is_part_of_cylinder(float r1, float r2, float r3, float angle_increment)
    {
        // Calculate the expected distance between points for a cylinder
        float expected_distance = cylinder_diameter_ / 2; // Half diameter for proximity checks
        float distance1 = std::abs(r1 - r2);
        float distance2 = std::abs(r2 - r3);
        
        // Allow for some tolerance in the measurement
        float tolerance = 0.02; // 2 cm tolerance

        // Check if the distances match the expected distance for a cylinder
        return (distance1 < (expected_distance + tolerance) && distance1 > (expected_distance - tolerance) &&
                distance2 < (expected_distance + tolerance) && distance2 > (expected_distance - tolerance));
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    float cylinder_diameter_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cylinderDetector>());
    rclcpp::shutdown();
    return 0;
}