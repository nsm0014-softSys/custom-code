#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LF2.h"
#include "LF_rebuild.h"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace cv;
using namespace std;

class ImageFilter : public rclcpp::Node
{
public:
  ImageFilter() : Node("image_sub")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/softsys/image_raw", 10, std::bind(&ImageFilter::topic_callback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/custom/found_path", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto start = std::chrono::steady_clock::now();
    cv::Mat image = convert_image_msg_to_cv(msg);
    std::vector<cv::Point> Line = BlueBoy.IdentifyLine(image);
    geometry_msgs::msg::PoseArray posearray;
    auto end = std::chrono::steady_clock::now();
    std::cout << "Find Line Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

    for (uint i = 0; i < Line.size(); i++)
    {
      geometry_msgs::msg::Pose pose;

      cv::Point linepoint = Line.at(i);

      pose.position.x = linepoint.x;
      pose.position.y = linepoint.y;

      posearray.poses.push_back(pose);
    }

    image_pub_->publish(posearray);
  }

  cv::Mat convert_image_msg_to_cv(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    return cv_ptr->image;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr image_pub_;
  FindBlueLine BlueBoy;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto imageNode = std::make_shared<ImageFilter>();
  rclcpp::spin(imageNode);
  rclcpp::shutdown();
  return 0;
}
