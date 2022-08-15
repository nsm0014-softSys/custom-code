
#include "rclcpp/rclcpp.hpp"
#include "softsys_msgs/msg/throttle.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <stdio.h>
#include <chrono>

class Throttle_ctrl : public rclcpp::Node
{
public:
  Throttle_ctrl() : Node("throttle_ctrl")
  {
    this->declare_parameter("constant_throttle_value", 0.1);
    this->declare_parameter("min_throttle", 0.325);
    this->declare_parameter("throttle_switch", 1);
    this->declare_parameter("handicap", 1.0);
    throttle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/custom/found_path", 10, std::bind(&Throttle_ctrl::topic_callback, this, std::placeholders::_1));
    throttle_pub_ = this->create_publisher<softsys_msgs::msg::Throttle>("/softsys/throttle_cmd", 10);
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {

    double chosen_x = msg->poses[0].position.x; // taking the midpoint of the x coordinates
    // double chosen_y = msg->poses[4].position.y; // taking the midpoint of the y coordinates

    double centerpoint_x = 640 / 2; // center point to calculate an angle from

    double diff_x = chosen_x - centerpoint_x;

    double throttle_cmd = diff_x / centerpoint_x;
    if (throttle_cmd < 0)
    {
      throttle_cmd = 1 - (throttle_cmd * -1);
    }
    else
    {
      throttle_cmd = 1 - throttle_cmd;
    }

    int throttle_switch = this->get_parameter("throttle_switch").as_int();
    auto throttle = softsys_msgs::msg::Throttle();
    switch (throttle_switch)
    {
    case 0:
      throttle.throttle = this->get_parameter("constant_throttle_value").as_double();
      break;
    case 1:
      // Throttle handicap
      throttle_cmd = throttle_cmd * this->get_parameter("handicap").as_double();

      double throttle_floor = this->get_parameter("min_throttle").as_double();
      if (throttle_cmd < throttle_floor)
      {
        throttle_cmd = throttle_floor;
      }
      throttle.throttle = throttle_cmd;
      break;
    }

    std::cout << "Normalized Throttle:" << throttle_cmd << std::endl;

    throttle_pub_->publish(throttle);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr throttle_sub_;
  rclcpp::Publisher<softsys_msgs::msg::Throttle>::SharedPtr throttle_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto throttle_node = std::make_shared<Throttle_ctrl>();
  rclcpp::spin(throttle_node);
  rclcpp::shutdown();
  return 0;
}
