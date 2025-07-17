#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KittiSubscriber : public rclcpp::Node
{
    public:
        KittiSubscriber() : Node("kitti_subscriber")
        {
            
        }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main()
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiSubscriber>());
    rclcpp::shutdown();
    return 0;
}