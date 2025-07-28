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
    private:
        std::string bag_path_param;
        std::string image_param;
        std::string point_cloud_param;
        std::string tf_param;

        std::unique_ptr<rosbag2_cpp::Reader> reader;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_raw_image;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud;
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_tf;

        void callback();
        void processImage(rclcpp::SerializedMessage& extracted_serialized_msg);
        void processPointCloud(rclcpp::SerializedMessage& extracted_serialized_msg);
        void processTf(rclcpp::SerializedMessage& extracted_serialized_msg);
};
#endif