#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserProcessor : public rclcpp::Node
{
public:
    LaserProcessor() : Node("laser_scan_nth")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserProcessor::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset", 10);

    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {

      auto subset_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);

      int n = 5; //Every nth point

        // Copy all range data and set intensities
        subset_scan->ranges = scan->ranges;
        subset_scan->intensities.clear();
        subset_scan->intensities.reserve(scan->ranges.size());

        // Loog through scan ranges
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            // no remaiders (divisable by n)
            if (i % n == 0)
            {
                // Set intensity to 1.0 for every nth point
                subset_scan->intensities.push_back(1.0);
            }
            else
            {
                // Set intensity to 0.0 for other points
                subset_scan->intensities.push_back(0.0);
            }
        }



        scan_pub_->publish(*subset_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserProcessor>());
    rclcpp::shutdown();
    return 0;
}
