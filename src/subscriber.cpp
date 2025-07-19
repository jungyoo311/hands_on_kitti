#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" // Don't forget to add 
using namespace std::chrono_literals;
/*
current logic needs to be modified:
Bag File -> Node (reads + publishes) -> Topics -> RVIZ2 (subscribes)
*/
class KittiSubscriber : public rclcpp::Node
{
    public:
        KittiSubscriber(const std::string & bag_filename) : Node("kitti_subscriber")
        {
            publisher_left_raw_image_ = this->create_publisher<sensor_msgs::msg::Image>("/kitti/camera_color_left/image_raw", 10);
            publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/kitti/velo/pointcloud", 10);
            timer_ = this->create_wall_timer(
                100ms, std::bind(&KittiSubscriber::timer_callback, this)
            );
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_filename;
            storage_options.storage_id = "sqlite3";

            reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options); 
            reader_ -> open(storage_options);

            RCLCPP_INFO(this->get_logger(), "Opened bag file and staring image show: %s", bag_filename.c_str());
        }
    private:
        void timer_callback()
        {
            //stopping condition: reached the end of bag file.
            if(!reader_ -> has_next())
            {
                rclcpp::shutdown();
                return;
            }
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_ -> read_next();
            
            if (msg->topic_name == "/kitti/camera_color_left/image_raw")
            {
                rclcpp::SerializedMessage serialized_msg(*msg -> serialized_data);
                sensor_msgs::msg::Image::SharedPtr ros_msg_cam1 = std::make_shared<sensor_msgs::msg::Image>();
                serialization_cam1_.deserialize_message(&serialized_msg, ros_msg_cam1.get());
                publisher_left_raw_image_->publish(*ros_msg_cam1);
            }
            // ToDo: Visualize Lidar point cloud on RVIZ
            if (msg->topic_name == "/kitti/velo/pointcloud")
            {
                rclcpp::SerializedMessage serialized_msg(*msg ->serialized_data);
                sensor_msgs::msg::PointCloud2::SharedPtr ros_msg_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
                serialization_pointcloud_.deserialize_message(&serialized_msg, ros_msg_pointcloud.get());
                publisher_point_cloud_->publish(*ros_msg_pointcloud);
            }
        }
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_raw_image_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization_cam1_;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2>  serialization_pointcloud_;
        std::unique_ptr<rosbag2_cpp::Reader> reader_; // why we choose unique ptr here?:
        // ans: use unique_ptr or shared_ptr to avoid forgetting to delete objects created using new
        
};

int main(int argc, char ** argv)
{
    // should add input argc, argv checker
    // should add error handling
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KittiSubscriber>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}