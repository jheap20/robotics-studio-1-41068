#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ImageModifier : public rclcpp::Node
{
public:
  ImageModifier() : Node("image_modifier")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ImageModifier::image_callback, this, std::placeholders::_1));

    publisher_ = image_transport::create_publisher(this, "/modified_image");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      // Get image dimensions
      int height = cv_ptr->image.rows;
      int width = cv_ptr->image.cols;
      
      // Calculate center point
      cv::Point center(width / 2, height / 2);
      
      // Draw circle at the center
      cv::circle(cv_ptr->image, center, 50, cv::Scalar(0, 255, 0), 2);
      
      // Publish modified image
      publisher_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  image_transport::Publisher publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageModifier>());
  rclcpp::shutdown();
  return 0;
}