#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <functional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


// #include <point_cloud_transport/point_cloud_transport.hpp> // use this to fix the pub / sub logic
using namespace std::chrono_literals;
// make all rosbag2 read in a single node.

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("point_cloud_publisher");
    //create publisher
    auto publisher = node -> create_publisher<sensor_msgs::msg::PointCloud2>("/kitti/velo/pointcloud",10);

    // rosbag2 storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "../kitti_ros2_bag/kitti_ros2_bag.db3";
    storage_options.storage_id = "sqlite3";

    // converter options
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // make reader and open rosbag2
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pointcloud;
    while(reader.has_next() && rclcpp::ok())
    {
        //get serialized data
        auto serialized_msg = reader.read_next();
        rclcpp::SerializeMessage extracted_serialized_msg(*serialized_msg -> serialized_data);

        if (serialized_msg->topic_name == "/kitti/velo/pointcloud")
        {
            // deserialize data
            serialization_pointcloud.deserialize_message(&extracted_serialized_msg, &cloud_msg);
            //publish msg
            publisher->publish(cloud_msg);
            RCLCPP_INFO("point cloud publisehd...");
            //log using RCLCPP_INFO()
            rclcpp::spin_some(node);
            rclcpp::sleep_for(100ms);
        }
    }
    
    reader.close();
    rclcpp::shutdown();
    return 0;
}