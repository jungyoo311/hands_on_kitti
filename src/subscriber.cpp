#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/pointcloud2.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("point_cloud_subscriber");
    rclcpp::spin();
    rclcpp::shutdown()
    return 0;
}