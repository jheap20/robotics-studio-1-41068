#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

int main() {
    // Load the SLAM map (grayscale) and Gazebo world map (color)
    cv::Mat slam_map = cv::imread("/home/student/ros2_ws/src/maps/gen_map.pgm", cv::IMREAD_GRAYSCALE);
    cv::Mat gazebo_map = cv::imread("/home/student/ros2_ws/src/maps/map.pgm", cv::IMREAD_COLOR);

    // Check if the images are loaded successfully
    if (slam_map.empty()) {
        std::cerr << "Error: Could not load SLAM map." << std::endl;
        return -1;
    }
    if (gazebo_map.empty()) {
        std::cerr << "Error: Could not load Gazebo world map." << std::endl;
        return -1;
    }

    // Display the dimensions of the maps
    //std::cout << "SLAM Map Dimensions: " << slam_map.cols << "x" << slam_map.rows << " (Width x Height)" << std::endl;
    //std::cout << "Gazebo Map Dimensions: " << gazebo_map.cols << "x" << gazebo_map.rows << " (Width x Height)" << std::endl;

    // Determine the maximum dimensions of the canvas
    int max_width = std::max(slam_map.cols, gazebo_map.cols);
    int max_height = std::max(slam_map.rows, gazebo_map.rows);

    // Create a canvas of the maximum size (white background)
    cv::Mat canvas(max_height, max_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Convert the grayscale SLAM map to 3-channel
    cv::Mat slam_map_color;
    cv::cvtColor(slam_map, slam_map_color, cv::COLOR_GRAY2BGR);

    // Calculate the center positions for both maps
    int slam_center_x = canvas.cols / 2 - slam_map.cols / 2;
    int slam_center_y = canvas.rows / 2 - slam_map.rows / 2;

    int gazebo_center_x = canvas.cols / 2 - gazebo_map.cols / 2;
    int gazebo_center_y = canvas.rows / 2 - gazebo_map.rows / 2;

    // Adjust the offsets if you want to shift the maps around
    int slam_offset_x = 0; 
    int slam_offset_y = 0; 

    int gazebo_offset_x = 0; 
    int gazebo_offset_y = 0; 

    // Apply the offsets
    slam_center_x += slam_offset_x;
    slam_center_y += slam_offset_y;

    gazebo_center_x += gazebo_offset_x;
    gazebo_center_y += gazebo_offset_y;

    // Ensure the regions are within canvas bounds
    slam_center_x = std::max(0, slam_center_x);
    slam_center_y = std::max(0, slam_center_y);

    gazebo_center_x = std::max(0, gazebo_center_x);
    gazebo_center_y = std::max(0, gazebo_center_y);

    // Create ROI (Region of Interest) for each map
    cv::Rect slam_roi(slam_center_x, slam_center_y, slam_map.cols, slam_map.rows);
    cv::Rect gazebo_roi(gazebo_center_x, gazebo_center_y, gazebo_map.cols, gazebo_map.rows);

    // Copy the Gazebo map onto the canvas
    gazebo_map.copyTo(canvas(gazebo_roi));

    // Overlay the SLAM map on the canvas with transparency
    double alpha = 0.5; // transparency factor (0.0 - 1.0)
    cv::addWeighted(canvas(slam_roi), 
                    1 - alpha, 
                    slam_map_color, 
                    alpha, 
                    0.0, 
                    canvas(slam_roi));

    // Display the result
    cv::imshow("Overlaid Map", canvas);
    cv::waitKey(0); // Wait for a key press before closing the window

    // Save the overlaid map to a file
    //cv::imwrite("/home/student/ros2_ws/src/maps/overlaid_map.pgm", canvas);

    return 0;
}
