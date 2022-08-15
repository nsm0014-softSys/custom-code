
#include "rclcpp/rclcpp.hpp"
#include "softsys_msgs/msg/steer.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position.hpp"
#include <stdio.h>
#include <chrono>

using namespace std;

class pid_ctrl : public rclcpp::Node
{
public:
    pid_ctrl() : Node("pid_ctrl")
    {
        this->declare_parameter("look_ahead_index", 0);
        this->declare_parameter("derivative", 0.02);
        this->declare_parameter("integral", 0.0);
        this->declare_parameter("proportional", 0.925);

        pid_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/custom/found_path", 10, std::bind(&pid_ctrl::topic_callback, this, std::placeholders::_1));
        pid_pub_ = this->create_publisher<softsys_msgs::msg::Steer>("/softsys/steer_cmd", 10);
        marvel_sub_ = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePosition>("/hedgehog_pos", 10, std::bind(&pid_ctrl::marvel_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // int index_array = std::ceil(msg->poses.size()); // getting size of 1 frame points array
        // index_array = std::ceil(index_array/2);
        int index_array = this->get_parameter("look_ahead_index").as_int();
        double chosen_x = msg->poses[index_array].position.x; // taking the midpoint of the x coordinates
        // double chosen_y = msg->poses[index_array].position.y; // taking the midpoint of the y coordinates
        double centerpoint_x = 640 / 2; // center point to calculate an error from
        double diff_x = chosen_x - centerpoint_x;
        double steer_cmd = -diff_x / centerpoint_x;

        // double further_x = msg->poses[index_array + 1].position.x;
        // double even_further_x = msg->poses[index_array + 2].position.x;
        // double further_cmd = -(further_x - centerpoint_x)/centerpoint_x;
        // double even_further_cmd = -(even_further_x - centerpoint_x)/centerpoint_x;
        // double further_diff = further_x - centerpoint_x;
        // double further_cmd = -further_diff / centerpoint_x;

        std::cout << "Normalized Steer Angle:" << steer_cmd << std::endl;

        auto pid_steer = softsys_msgs::msg::Steer();
        bool lost_line = false;
        if ((steer_cmd > 0.97) || (steer_cmd < -0.97))
        {
            lost_line = true;
        }
        if (lost_line)
        {
            if (steer_store > 0)
            {
                steer_cmd = 1;
            }
            else if (steer_store < 0)
            {
                steer_cmd = -1;
            }
        }

        // Turning Zones
        if ((pos_y < 3.8) && pos_x < 2.0)
        {
            steer_cmd = 1;
        }
        if (abs(pos_x - 4.6) < 0.5 && abs(pos_y - 9.5) < 0.5)
        {
            steer_cmd = 1;
        }

        // Store the last commanded steer angle
        double kp = this->get_parameter("proportional").as_double();
        double ki = this->get_parameter("integral").as_double();
        double kd = this->get_parameter("derivative").as_double();

        double proportional = steer_cmd * kp;
        double derivative = ((steer_cmd - steer_store) / dT) * kd;
        double integral = steer_cmd * dT * ki;
        steer_cmd = derivative + integral + proportional;

        std::cout << "Controlled Steer Angle:" << steer_cmd << std::endl;

        pid_steer.steer_angle = steer_cmd;
        steer_store = steer_cmd;
        pid_pub_->publish(pid_steer);
    }

    void marvel_callback(const marvelmind_ros2_msgs::msg::HedgePosition msg)
    {
        pos_x = msg.x_m;
        pos_y = msg.y_m;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pid_sub_;
    rclcpp::Publisher<softsys_msgs::msg::Steer>::SharedPtr pid_pub_;
    rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr marvel_sub_;
    double steer_store;
    double pos_y;
    double pos_x;
    double dT = 0.05;
};

int main(int argc, char **argv)
{
    {
        rclcpp::init(argc, argv);
        auto pid_ctrl_node = std::make_shared<pid_ctrl>();
        rclcpp::spin(pid_ctrl_node);
        rclcpp::shutdown();
        return 0;
    }
}
