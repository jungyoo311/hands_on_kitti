#include "../include/my_single_node_pkg/my_node.hpp"
/*
right image add first? synchronization first?

synchroniation first bc of easier debugging
TODO: synchornize image, pc2, tf
1. process all msgs together not separately
2. maintain original timestamps
3. publish with timing control

each step for synchronized playback:
1. loads all msgs into memory first
2. sorts by timestamp to maintain original sequence
3. publishes with proper timing to show data it was recorded
*/
BagReader::BagReader() : Node("bag_reader")
{
    //parameter: {"(str) parameter's name", "(str) default value"}
    this->declare_parameter<std::string>("bag_path", "../kitti_ros2_bag/kitti_ros2_bag.db3");
    this->declare_parameter<std::string>("raw_image", "/kitti/camera_color_left/image_raw");
    this->declare_parameter<std::string>("point_cloud", "/kitti/velo/pointcloud");
    this->declare_parameter<std::string>("tf", "/tf");

    //get param
    bag_path_param = this->get_parameter("bag_path").as_string();
    image_param = this->get_parameter("raw_image").as_string();
    point_cloud_param = this->get_parameter("point_cloud").as_string();
    tf_param = this->get_parameter("tf").as_string();
    //initailize reader before use
    reader = std::make_unique<rosbag2_cpp::Reader>();
    //initailize publishers in constructor
    publisher_left_raw_image = this->create_publisher<sensor_msgs::msg::Image>(image_param, 10);
    publisher_point_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_param, 10);
    
    broadcaster_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    //callback
    callback();
}
    
void BagReader::callback()
{
    // rosbag2 storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_param;
    storage_options.storage_id = "sqlite3";
    
    reader->open(storage_options);
    RCLCPP_INFO(this->get_logger(), "Bag reading completed");

    sensor_msgs::msg::Image left_img_msg;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    tf2_msgs::msg::TFMessage tf_msg;

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_left_image;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pointcloud;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization_tf;

    int img_cnt = 0, cloud_cnt = 0, tf_cnt = 0;

    while(reader->has_next() && rclcpp::ok())
    {
        // serialized data -> deserialized data -> publish msg/broadcast data
        auto serialized_msg = reader->read_next();
        rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg -> serialized_data);
        if(serialized_msg->topic_name == image_param){
            processImage(extracted_serialized_msg);
            img_cnt++;
        } else if(serialized_msg->topic_name == point_cloud_param){
            processPointCloud(extracted_serialized_msg);
            cloud_cnt++;
        } else if(serialized_msg->topic_name == tf_param){
            processTf(extracted_serialized_msg);
            tf_cnt++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(this->get_logger(), "Processing Done");
    RCLCPP_INFO(this->get_logger(), "[LOG] Total Images processed: %d", img_cnt);
    RCLCPP_INFO(this->get_logger(), "[LOG] Total Point Clouds processed: %d", cloud_cnt);
    RCLCPP_INFO(this->get_logger(), "[LOG] Total TF processed: %d", tf_cnt);
    reader->close();
    rclcpp::shutdown();
}
void BagReader::processImage(rclcpp::SerializedMessage& extracted_serialized_msg)
{
    sensor_msgs::msg::Image left_img_msg;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_left_image;
    RCLCPP_INFO(this->get_logger(), "Processing Images...");
    
    //deserialized data
    serialization_left_image.deserialize_message(&extracted_serialized_msg, &left_img_msg);
    //publish msg
    publisher_left_raw_image->publish(left_img_msg);
}
void BagReader::processPointCloud(rclcpp::SerializedMessage& extracted_serialized_msg)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pointcloud;
    RCLCPP_INFO(this->get_logger(), "Processing Point Clouds...");

    //deserialized data
    serialization_pointcloud.deserialize_message(&extracted_serialized_msg, &cloud_msg);
    //publish msg
    publisher_point_cloud->publish(cloud_msg);
}
void BagReader::processTf(rclcpp::SerializedMessage& extracted_serialized_msg)
{
    // tf2 contains multiple coordinate frame relationships
    // broadcast individual transforms separetely.
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization_tf;
    RCLCPP_INFO(this->get_logger(), "Processing TF records from rosbag2...");

    serialization_tf.deserialize_message(&extracted_serialized_msg, &tf_msg);
    //broadcast msg, separately
    // it's array, i need to iterate
    for(const auto& t : tf_msg.transforms)
    {
        broadcaster_tf->sendTransform(t);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<BagReader>();
        rclcpp::spin(node);
    } catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        return 1;
    }
    return 0;
}