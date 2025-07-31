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
    //topics
    this->declare_parameter<std::string>("input_left_raw_img", "/kitti/camera_color_left/image_raw");
    this->declare_parameter<std::string>("output_left_raw_img", "/rviz/camera_color_left/image_raw");
    this->declare_parameter<std::string>("input_right_raw_img", "/kitti/camera_color_right/image_raw");
    this->declare_parameter<std::string>("output_right_raw_img", "/rviz/camera_color_right/image_raw");
    this->declare_parameter<std::string>("input_point_cloud", "/kitti/velo/pointcloud");
    this->declare_parameter<std::string>("output_point_cloud", "/rviz/velo/pointcloud");
    this->declare_parameter<std::string>("tf", "/tf");
    this->declare_parameter<std::string>("tf_static", "/tf_static");

    //get params
    std::string input_left_raw_img = this->get_parameter("input_left_raw_img").as_string();
    std::string output_left_raw_img = this->get_parameter("output_left_raw_img").as_string();
    std::string input_right_raw_img = this->get_parameter("input_right_raw_img").as_string();
    std::string output_right_raw_img = this->get_parameter("output_right_raw_img").as_string();
    std::string input_point_cloud = this->get_parameter("input_point_cloud").as_string();
    std::string output_point_cloud = this->get_parameter("output_point_cloud").as_string();
    std::string input_tf = this->get_parameter("tf").as_string();
    std::string input_tf_static = this->get_parameter("tf_static").as_string();

    //subscribe to rosbag2
    left_raw_img_sub = this->create_subscription<sensor_msgs::msg::Image>(input_left_raw_img, 10,
    std::bind(&BagReader::processImageLeft, this, std::placeholders::_1));
    right_raw_img_sub = this->create_subscription<sensor_msgs::msg::Image>(input_right_raw_img, 10,
    std::bind(&BagReader::processImageRight, this, std::placeholders::_1));
    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_point_cloud, 10,
    std::bind(&BagReader::processPointCloud, this, std::placeholders::_1));
    tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(input_tf, 10,
    std::bind(&BagReader::processTf, this, std::placeholders::_1));
    tf_static_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(input_tf_static, 10,
    std::bind(&BagReader::processTf, this, std::placeholders::_1));

    //publisher to rviz2
    left_raw_img_pub = this->create_publisher<sensor_msgs::msg::Image>(output_left_raw_img, 10);
    right_raw_img_pub = this->create_publisher<sensor_msgs::msg::Image>(output_right_raw_img, 10);
    point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_point_cloud, 10);
    
    //tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    img_w = 640;
    img_h = 640;
    thres_high = 255;
    thres_low = 85;
}

void BagReader::processImageLeft(const sensor_msgs::msg::Image::SharedPtr msg)
{   
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(this->get_logger(), "Processing LEFT Images...");
    left_img_cnt++;
    //TODO
    /*implement canny edge detection of image_left_raw*/
    /*I want to compare with the lidar point cloud detection result for sanity check*/

    // ros img msg to cv matrix format
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    original = cv_ptr->image;
    cv::resize(original, resized_img, cv::Size(img_w, img_h));
    cv::cvtColor(resized_img, rgb_img, cv::COLOR_BGR2RGB);
    
    cv::cvtColor(resized_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_img, canny_img, thres_low, thres_high);
    cv::cvtColor(canny_img, canny_3ch, cv::COLOR_GRAY2BGR);
    cv::cvtColor(gray_img, gray_3ch, cv::COLOR_GRAY2BGR);
    cv::Mat top_row, bot_row, display;
    cv::hconcat(rgb_img, resized_img, top_row);
    cv::hconcat(gray_3ch, canny_3ch, bot_row);
    cv::vconcat(top_row, bot_row, display);

    cv::imshow("2x2 visualization", display);
    cv::waitKey(1);
    //publish msg
    left_raw_img_pub->publish(*msg);
}
void BagReader::processImageRight(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(this->get_logger(), "Processing RIGHT Images...");
    right_img_cnt++;
    //publish
    right_raw_img_pub->publish(*msg);
}
void BagReader::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(this->get_logger(), "Processing Point Clouds...");
    point_cloud_cnt++;
    //fix frame
    // auto fix_msg = *msg;
    // fix_msg.header.frame_id = "base_link";
    
    //publish msg
    point_cloud_pub->publish(*msg);
}
void BagReader::processTf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(this->get_logger(), "Processing TF records from rosbag2...");
    tf_cnt++;
    //broadcast msg, separately
    // it's array, i need to iterate
    for(const auto& t : msg->transforms)
    {
        tf_broadcaster->sendTransform(t);
    }
}
void BagReader::processTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for(const auto& t : msg->transforms)
    {
        tf_broadcaster->sendTransform(t);
    }
}

BagReader::~BagReader(){
    cv::destroyAllWindows();
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
    rclcpp::shutdown();
    return 0;
}