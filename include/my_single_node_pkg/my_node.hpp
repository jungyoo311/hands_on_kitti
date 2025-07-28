#ifndef MY_NODE_HPP
#define MY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

class BagReader : public rclcpp::Node
{
    public:
        BagReader();
        ~BagReader();
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_raw_img_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_raw_img_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_raw_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_raw_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        
        void processImageLeft(const sensor_msgs::msg::Image::SharedPtr msg);
        void processImageRight(const sensor_msgs::msg::Image::SharedPtr msg);
        void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void processTf(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        void processTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        
        int left_img_cnt;
        int right_img_cnt;
        int point_cloud_cnt;
        int tf_cnt;
};
#endif