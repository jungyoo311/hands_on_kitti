/*
07192025
pct_subscriber by referring this page.
https://docs.ros.org/en/ros2_packages/jazzy/api/point_cloud_transport_tutorial/
I cannot install the packages so i drop this method.
*/

#include "point_cloud_transport/point_cloud_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("point_cloud_subscriber");

    point_cloud_transport::PointCloudTransport pct(node);
    point_cloud_transport::Subscriber pct_sub = pct.subscribe(
        "pct/point_cloud", 100,
        [node](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
        {
            RCLCPP_INFO_STREAM(
                node->get_logger(),
                "Message received and number of points is: " << msg->width * msg->height
            );
        }, {});
    RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for point cloud msgs...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   