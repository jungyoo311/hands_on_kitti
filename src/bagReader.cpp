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
#include <chrono>
#include <functional>
#include <string>
#include <thread>
using namespace std::chrono_literals;
/*
Lidar have the correct tf so that i can see the difference 
between car frame and lidar frame perspective.
*/
class BagReader : public rclcpp::Node
{
    public:
        BagReader() : Node("bag_reader")
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
    private:
        // Open bag -> get metadata -> Process bag in sequence -> Close Bag
        std::string bag_path_param;
        std::string image_param;
        std::string point_cloud_param;
        std::string tf_param;

        // converter options
        // rosbag2_cpp::ConverterOptions converter_options;
        // converter_options.input_serialization_format = "cdr";
        // converter_options.output_serialization_format = "cdr";
        
        std::unique_ptr<rosbag2_cpp::Reader> reader;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_raw_image;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud;
        
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_tf;
        
        void callback()
        {
            // rosbag2 storage options
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_path_param;
            storage_options.storage_id = "sqlite3";
            
            // reader->open(storage_options);
            // RCLCPP_INFO(this->get_logger(), "Bag is opened: %s", bag_path_param.c_str());
            // //if topic_name == "image_param": processImage()
            // // processPointCloud()
            // processImage();
            // reader->close();

            // reader->open(storage_options);
            // processPointCloud();
            // reader->close();
            // RCLCPP_INFO(this->get_logger(), "Bag Reading finished");
            
            reader->open(storage_options);
            processTf();
            reader->close();

            RCLCPP_INFO(this->get_logger(), "Bag reading completed");
            rclcpp::shutdown();
        }
        void processImage()
        {
            sensor_msgs::msg::Image left_img_msg;
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization_left_image;
            RCLCPP_INFO(this->get_logger(), "Processing Images...");
            int image_cnt = 0;

            while(reader->has_next() && rclcpp::ok())
            {
                auto serialized_msg = reader->read_next();
                if (serialized_msg->topic_name == image_param)
                {
                    //serialized data
                    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg -> serialized_data);
                    //deserialized data
                    serialization_left_image.deserialize_message(&extracted_serialized_msg, &left_img_msg);
                    //publish msg
                    publisher_left_raw_image->publish(left_img_msg);
                    image_cnt++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
            RCLCPP_INFO(this->get_logger(), "Total left_raw_image published: %d", image_cnt);
        }
        void processPointCloud()
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pointcloud;
            RCLCPP_INFO(this->get_logger(), "Processing Point Clouds...");
            int cloud_cnt = 0;

            while(reader->has_next() && rclcpp::ok())
            {
                //get serialized data
                auto serialized_msg = reader->read_next();
                if (serialized_msg -> topic_name == point_cloud_param)
                {
                    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg -> serialized_data);
                    //deserialized data
                    serialization_pointcloud.deserialize_message(&extracted_serialized_msg, &cloud_msg);
                    //publish msg
                    publisher_point_cloud->publish(cloud_msg);
                    cloud_cnt++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
            RCLCPP_INFO(this->get_logger(), "Total point cloud published: %d", cloud_cnt);
        }
        void processTf()
        {
            // tf2 contains multiple coordinate frame relationships
            // broadcast individual transforms separetely.
            tf2_msgs::msg::TFMessage tf_msg;
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization_tf;
            RCLCPP_INFO(this->get_logger(), "Processing TF records from rosbag2...");
            int tf_cnt = 0;

            while(reader->has_next() && rclcpp::ok())
            {
                auto serialized_msg = reader->read_next();
                if(serialized_msg->topic_name == tf_param)
                {
                    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg -> serialized_data);
                    serialization_tf.deserialize_message(&extracted_serialized_msg, &tf_msg);
                    //broadcast msg, separately
                    // it's array, i need to iterate
                    for(const auto& t : tf_msg.transforms)
                    {
                        broadcaster_tf->sendTransform(t);

                    }
                    tf_cnt++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
            RCLCPP_INFO(this->get_logger(), "Total tf transformation processed: %d", tf_cnt);
        }   
};

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
    // rclcpp::shutdown();
    return 0;
}