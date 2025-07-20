/*
07192025
pct_publisher by referring this page.
https://docs.ros.org/en/ros2_packages/jazzy/api/point_cloud_transport_tutorial/
I cannot install the packages so i drop this method.
*/
#include <point_cloud_transport/point_cloud_transport.hpp>

// for reading rosbag
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("point_cloud_publisher");
    point_cloud_transport::PointCloudTransport pct(node);
    point_cloud_transport::Publisher pct_pub = pct.advertise("pct/point_cloud", 100);

    const std::string bagged_cloud_topic = "/point_cloud";
    const std::string bag_file = "../kitti_ros2_bag/kitti_ros2_bag.db3";
    //tell rosbag2 how to read the bag.
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // open rosbag
    ros2bag_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2>  serialization_pointcloud_;
    while (reader.has_next() && rclcpp::ok())
    {
        //get the serialized data
        auto serialized_message = reader.read_next();
        rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message -> serialized_data); // diff between using dot and arrow here?
        if (serialized_message->topic_name == bagged_cloud_topic)
        {
            // deserialize and convert to msg
            serialization_pointcloud.deserialize_message(*extracted_serialized_msg, &cloud_msg); // why passing addr of cloud_msg?
            //publish the msg
            pct_pub.publish(cloud_msg); // why not passing addr here?
            rclcpp::spin_some(node); // start processing data from the node        
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }
    reader.close();

    node.reset(); // why reset the node here?
    rclcpp::shutdown();
    return 0;
}