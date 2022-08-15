#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/image.hpp"
#include <stdio.h>
#include <stdarg.h>

using namespace std;
using namespace cv;
#define PI 3.1415926

int frameWidth = 640;
int frameHeight = 480;

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

class InvPerspMap : public rclcpp::Node
{
public:
    InvPerspMap() : Node("IPMap")
    {
        this->declare_parameter("state", 1);
        this->declare_parameter("Alpha", 289);
        this->declare_parameter("Beta", 90);
        this->declare_parameter("Gamma", 90);
        this->declare_parameter("F", 2000);
        this->declare_parameter("Distance", 1000);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/softsys/image_raw", 10, std::bind(&InvPerspMap::Mapping_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/custom/invPersMap", 10);
    }

private:
    void Mapping_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        int state = this->get_parameter("state").as_int();
        int alpha_ = this->get_parameter("Alpha").as_int();
        int beta_ = this->get_parameter("Beta").as_int();
        int gamma_ = this->get_parameter("Gamma").as_int();
        int f_ = this->get_parameter("F").as_int();
        int dist_ = this->get_parameter("Distance").as_int();
        std::cout << "Getting an image" << std::endl;
        cv::Mat image = convert_image_msg_to_cv(msg);

        // while (true)
        // {
            // Defining the region of interest:
            Mat dst, cdst;

            // Converting Parameters to Degrees
            double focalLength, dist, alpha, beta, gamma;


            alpha = alpha_ * PI / 180;
            beta = ((double)beta_ - 90) * PI / 180;
            gamma = ((double)gamma_ - 90) * PI / 180;
            focalLength = (double)f_;
            dist = (double)dist_;

            // Resizing
            Size image_size = image.size();
            double w = (double)image_size.width, h = (double)image_size.height;

            // Projecion matrix 2D -> 3D
            Mat A1 = (Mat_<float>(4, 3) << 1, 0, -w / 2,
                      0, 1, -h / 2,
                      0, 0, 0,
                      0, 0, 1);

            // Rotation matrices Rx, Ry, Rz

            Mat RX = (Mat_<float>(4, 4) << 1, 0, 0, 0,
                      0, cos(alpha), -sin(alpha), 0,
                      0, sin(alpha), -cos(alpha), 0,
                      0, 0, 0, 1);

            Mat RY = (Mat_<float>(4, 4) << cos(beta), 0, -sin(beta), 0,
                      0, 1, 0, 0,
                      sin(beta), 0, cos(beta), 0,
                      0, 0, 0, 1);

            Mat RZ = (Mat_<float>(4, 4) << cos(gamma), -sin(gamma), 0, 0,
                      sin(gamma), cos(gamma), 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

            // R - rotation matrix
            Mat R = RX * RY * RZ;

            // T - translation matrix
            Mat T = (Mat_<float>(4, 4) << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, dist,
                     0, 0, 0, 1);

            // K - intrinsic matrix
            Mat K = (Mat_<float>(3, 4) << focalLength, 0, w / 2, 0,
                     0, focalLength, h / 2, 0,
                     0, 0, 1, 0);

            Mat transformationMat = K * (T * (R * A1));
            Mat frame;

            warpPerspective(image, frame, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

            image_msg = std::make_shared<sensor_msgs::msg::Image>(sensor_msgs::msg::Image());

            // Convert OpenCV Mat to ROS Image
            rclcpp::Time timestamp = this->get_clock()->now();
            image_msg->header.stamp = timestamp;
            image_msg->header.frame_id = "Inverted Image";
            image_msg->height = frame.rows;
            image_msg->width = frame.cols;
            image_msg->encoding = mat_type2encoding(frame.type());
            image_msg->is_bigendian = false;
            image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            image_msg->data.assign(frame.datastart, frame.dataend);
            image_pub_->publish(*image_msg);
        // }
    }

    cv::Mat convert_image_msg_to_cv(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

        return cv_ptr->image;
    }

    std::string
    mat_type2encoding(int mat_type)
    {
        switch (mat_type)
        {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image::SharedPtr image_msg;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto InvPerspMap_node = std::make_shared<InvPerspMap>();
    rclcpp::spin(InvPerspMap_node);
    rclcpp::shutdown();
    return 0;
}
