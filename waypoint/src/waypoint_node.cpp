#include <chrono>
#include <string>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_imu_fusion.hpp"
#include "softsys_msgs/msg/steer.hpp"

using namespace std;

class WaypointFollower : public rclcpp::Node
{
public:
  WaypointFollower() : Node("waypoint")
  {
    this->declare_parameter("look_ahead", 3);
    
    way_sub_ = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePosition>("/hedgehog_pos", 10, std::bind(&WaypointFollower::topic_callback, this, std::placeholders::_1));
    // yaw_sub_ = this->create_subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>("/hedgehog_pos", 10, std::bind(&WaypointFollower::yaw_callback, this, std::placeholders::_1));
    way_pub_ = this->create_publisher<softsys_msgs::msg::Steer>("/softsys/steer_cmd", 10);
    std::ifstream waypoints;
    waypoints.open("/home/jetson/softsys2022/softsys_2022_ws/src/custom-code/waypoint/include/waypoint/waypointsFile.txt");
    if (waypoints.is_open()) {
      int i = 0;
      while (!waypoints.eof()) {
        waypoints >> waypoint_array[i][0] >> waypoint_array[i][1];
        i = i + 1;
      }
    }
    waypoints.close();
  }

private:
  struct Point {
    double x, y;
  };

  void topic_callback(const marvelmind_ros2_msgs::msg::HedgePosition msg)
  {
    Point current;
    current.x = msg.x_m;
    current.y = msg.y_m;

    std::cout << "Current: " << current.x << ", " << current.y << std::endl;

    int waypoint_index = find_closest_index(current.x, current.y);
    int look_ahead = this->get_parameter("look_ahead").as_int();

    if (waypoint_index + look_ahead > waypoint_size - 1) {
      waypoint_index = waypoint_index + look_ahead - waypoint_size;
    } else {
      waypoint_index = waypoint_index + look_ahead;
    }

    std::cout << "Looking At: " << waypoint_index << std::endl;
    std::cout << "Driving Towards: " << waypoint_array[waypoint_index][0] << ", " << waypoint_array[waypoint_index][1] << std::endl;

    Point diff;
    diff.x = current.x - waypoint_array[waypoint_index][0];
    diff.y = current.y - waypoint_array[waypoint_index][1];

    // Finds the heading angle needed to get to waypoint
    double desired_steer = atan(diff.x/diff.y);
    std::cout << "Desired Steer: " << desired_steer * 180 / 3.14159 << std::endl;

    // Finds the current heading
    double steer_current = heading_update(current, store1/*, store2, store3*/);

    if ((abs((steer_current - desired_steer) *180/3.14159) < 20) || (initial_heading == true)) {
      heading_store = steer_current;
      initial_heading = false;
    } else {
      steer_current = heading_store;
    }
    std::cout << "Current Steer: " << steer_current * 180 / 3.14159 << std::endl;

    // Heading error
    double heading_error = desired_steer - steer_current;

    double steer_cmd = -heading_error / (0.5235);

    std::cout << "Commanded Steer Angle: " << steer_cmd << "\n" << std::endl;

    auto way_steer = softsys_msgs::msg::Steer();
    way_steer.steer_angle = steer_cmd;
   // if (get_dist(store2, store3) > 0.25) {
      // store3.x = store2.x;
      // store3.y = store2.y;
    //}
    //if (get_dist(store1, store2) > 0.25) {
      // store2.x = store1.x;
      // store2.y = store1.y;
    //}
    if (get_dist(current, store1) > 0.25) {
      store1.x = current.x;
      store1.y = current.y;
    }
    way_pub_->publish(way_steer);
  }

  // void yaw_callback(const marvelmind_ros2_msgs::msg::HedgeImuFusion msg) 
  // {
  //   double yaw_in = msg.yaw;
  // }

  int find_closest_index(double x_m, double y_m)
  {
    double min_dist = 1000000;
    int index = -1;
    for(int i = 0;i<waypoint_size;i++)
    {
      double x_distance = abs(x_m - waypoint_array[i][0]);
      double y_distance = abs(y_m - waypoint_array[i][1]);
      double distance = sqrt(pow(x_distance,2) + pow(y_distance,2));
      if (distance < min_dist) {
        min_dist = distance;
        index = i;
      }
    }
    return index;
  }

  double get_dist(Point one, Point two) {
    double x_distance = abs(one.x - two.x);
    double y_distance = abs(one.x - two.x);
    double distance = sqrt(pow(x_distance,2) + pow(y_distance,2));
    return distance;
  }
  
  double heading_update(Point current, Point store1/*, Point store2, Point store3*/) {
    // double str3tostr2 = atan((store3.x-store2.x)/(store3.y-store2.y));
    // double str3tostr1 = atan((store3.x-store1.x)/(store3.y-store1.y));
    // double str3tocurr = atan((store3.x-current.x)/(store3.y-current.y));
    // double str2tostr1 = atan((store2.x-store1.x)/(store2.y-store1.y));
    // double str2tocurr = atan((store2.x-current.x)/(store2.y-current.y));
    double str1tocurr = atan((store1.x-current.x)/(store1.y-current.y));
    // double average = (str3tostr2+str3tostr1+str3tocurr+str2tostr1+str2tocurr+str1tocurr)/6;
    return str1tocurr;
  }

  rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr way_sub_;
  // rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>::SharedPtr yaw_sub_;
  rclcpp::Publisher<softsys_msgs::msg::Steer>::SharedPtr way_pub_;
  Point store1;
  Point store2;
  Point store3;
  bool initial_heading = true;
  double current_yaw;
  double heading_store;
  double waypoint_array[53][2];
  double waypoint_size = 53;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto wayNode = std::make_shared<WaypointFollower>();
  rclcpp::spin(wayNode);
  rclcpp::shutdown();
  return 0;
}
