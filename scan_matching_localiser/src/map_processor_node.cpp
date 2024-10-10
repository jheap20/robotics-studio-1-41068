#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node"), angle_difference_(0.0), relative_orientaion_(0.0)
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MapProcessorNode::odom_callback, this, std::placeholders::_1));

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW2, cv::WINDOW_AUTOSIZE); 
        cv::namedWindow(WINDOW3, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW4, cv::WINDOW_AUTOSIZE);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            initial_pose_x_ = msg->pose.pose.position.x;
            initial_pose_y_ = msg->pose.pose.position.y;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch; // Temporary variables to hold roll and pitch
            m.getRPY(roll, pitch, initial_pose_theta_);

            current_pose_x_ = initial_pose_x_;
            current_pose_y_ = initial_pose_y_;
            current_pose_theta_ = initial_pose_theta_;

            initialized_ = true;
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //std::cout << "scanCallback" << std::endl;
        // cv::Mat tmp_col_img = m_MapColImage.clone();
        // cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        // cv::imshow(WINDOW1, tmp_col_img);
        scan_image = laserScanToImage(msg);
        cv::rotate(scan_image, scan_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        
        cv::imshow(WINDOW3, scan_image);
        cv::waitKey(1);    

        calculateYawChange();
        relative_orientaion_ = relative_orientaion_ + angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
    }
    

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat tmp_col_img = m_MapColImage.clone();
        
        cv::resize(tmp_col_img, tmp_col_img, cv::Size(500, 500));
        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW1, tmp_col_img);

         // Generate and display the edge image
        //cv::Mat edge_img;
        cv::Canny(m_MapBinImage, edge_img, 50, 100);  // Apply Canny edge detection

        cv::resize(edge_img, edge_img, cv::Size(500, 500));
        cv::rotate(edge_img, edge_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow(WINDOW2, edge_img);  // Show the edge image in a new window

        
        //extractMapSection();
        

        cv::waitKey(1);

    }

/**
    void extractMapSection()
{
    if (!initialized_ || m_MapBinImage.empty() || scan_image.empty()) {
        return;
    }

    // Get the robot's current position in the world
    double robot_x = current_pose_x_;
    double robot_y = current_pose_y_;

    // Convert world coordinates to map pixel coordinates
    int map_x = static_cast<int>((robot_x - origin_x) / map_scale_);
    int map_y = static_cast<int>((robot_y - origin_y) / map_scale_);

    // Define the size of the region to extract (in pixels)
    // We will use a larger region and resize it to match the scan image
    int region_size = 120;  // This can be adjusted if needed

    // Define the bounds of the region around the robot
    int x_start = std::max(0, map_x - region_size / 2);
    int x_end = std::min(static_cast<int>(m_MapBinImage.cols), map_x + region_size / 2);
    int y_start = std::max(0, map_y - region_size / 2);
    int y_end = std::min(static_cast<int>(m_MapBinImage.rows), map_y + region_size / 2);

    // Extract the region of interest
    extracted_region = m_MapBinImage(cv::Rect(x_start, y_start, x_end - x_start, y_end - y_start));

    cv::resize(extracted_region, extracted_region, cv::Size(500, 500));

    // Display the resized map section
    cv::rotate(extracted_region, extracted_region, cv::ROTATE_90_COUNTERCLOCKWISE);
    //cv::imshow(WINDOW4, resized_map_section);
    cv::waitKey(1);
}

  */  



    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
    int grid_data;
    unsigned int row, col, val;

    m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

    std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

    for (row = 0; row < grid->info.height; row++) {
        for (col = 0; col < grid->info.width; col++) {
            grid_data = grid->data[row * grid->info.width + col];
            if (grid_data != -1) {
                val = 255 - (255 * grid_data) / 100;
                val = (val == 0) ? 255 : 0;
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
            } else {
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
            }
        }
    }

    map_scale_ = grid->info.resolution;
    origin_x = grid->info.origin.position.x;
    origin_y = grid->info.origin.position.y;
    size_x = grid->info.width;
    size_y = grid->info.height;

    cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                0, 1, 0,
                                0, 0, 0);
    cv::erode(m_temp_img, m_MapBinImage, kernel);

    m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
    cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

    std::cout << "Occupancy grid map converted to a binary image\n";

     // Display the image to verify
     //cv::imshow("Occupancy Grid", m_MapColImage);
     //cv::waitKey(1);
}

cv::Mat laserScanToImage(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg)
    {
        int img_size = 500;
        float max_range = scan_msg->range_max;
        cv::Mat laser_image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
            float range = scan_msg->ranges[i];
            if (range > scan_msg->range_min && range < scan_msg->range_max) {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    laser_image.at<uchar>(y, x) = 255;
                }
            }
        }
        return laser_image;
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, 
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) 
    {
        if (img1.empty() || img2.empty()) {
            RCLCPP_ERROR(this->get_logger(), "One or both images are empty. Cannot draw matches.");
            return;
        }
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);
        
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);
        
        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });
        
        // Determine the number of top matches to keep (15% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.30);
        
        // Keep only the best matches (top 15%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);
        
        srcPoints.clear();
        dstPoints.clear();
        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }

        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches);
        cv::imshow(WINDOW4, img_matches);
        cv::waitKey(1);

    }

    void calculateYawChange() {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(edge_img, scan_image, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }

    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
    cv::Mat edge_img;
    cv::Mat scan_image;
    cv::Mat extracted_region;


    bool initialized_;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_theta_;
    double current_pose_x_;
    double current_pose_y_;
    double current_pose_theta_;


    const std::string WINDOW1 = "Map Image";
    const std::string WINDOW2 = "Edge Image";
    const std::string WINDOW3 = "Laser Scan";
    const std::string WINDOW4 = "Features Match";
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

 