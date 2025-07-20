#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using namespace std::chrono_literals;
class PointCloudSubscriber : public rclcpp::Node
{
    public:
        PointCloudSubscriber() : Node("point_cloud_subscriber")
        {   
            subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/kitti/velo/pointcloud", 10, std::bind(&PointCloudSubscriber::timer_callback, this, std::placeholders::_1)
            );
            
            // rosbag2_storage::StorageOptions storage_options;
            // storage_options.uri = "../kitti_ros2_bag/kitti_ros2_bag.db3";
            // storage_options.storage_id = "cdr";
        }
    private:
    //implement timer_callback here
        void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            size_t num_pts = msg->width * msg->height;
            RCLCPP_INFO(this->get_logger(), "Message received and number of points is: %zu", num_pts);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}