// Create a node to subscribe to the topics in the bagfile
/*
1. what's inside of bagfile? what topics i should subsribe?
2. should i also create publiser? or subscriber only?
*/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class KittiPublisher : rclcpp::Node
{
    public:
        KittiPublisher() : Node('kitti_publisher')
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiPublisher>()); // start processing data from the node
    rclcpp::shutdown();
    return 0;
}