/**
 * @file depthMap.cpp
 * @author Jung
 * @brief This file generate rgb depth map from kitti ros bag
 * @date 08-04-2025
 * @copyright Jung
 */

#include "../include/my_single_node_pkg/depth_map_node.hpp"

DepthMapNode::DepthMapNode() : Node("depth_map"),
    fx_(0.0),
    fy_(0.0),
    cx_(0.0),
    cy_(0.0),
    baseline(0.0),
    calibration_loaded(false),
    window_size(11),
    min_disp(0),
    num_disp(112)
{
    this->declare_parameter<std::string>("input_left_gray_img", "/kitti/camera_gray_left/image_raw");
    this->declare_parameter<std::string>("input_right_gray_img", "/kitti/camera_gray_right/image_raw");
    this->declare_parameter<std::string>("input_camera_info", "/kitti/camera_gray_right/camera_info");
    std::string input_left_gray_img = this->get_parameter("input_left_gray_img").as_string();
    std::string input_right_gray_img = this->get_parameter("input_right_gray_img").as_string();
    std::string input_camera_info = this->get_parameter("input_camera_info").as_string();

    depth_map_pub = this->create_publisher<sensor_msgs::msg::Image>("depth_map", 10); // depth map pub
    // rgb_depth_map_pub = this->create_publisher<sensor_msgs::msg::Image>("rgb_depth_map, 10");

    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(input_camera_info, 10,
    std::bind(&DepthMapNode::processCameraInfo, this, std::placeholders::_1));

    cv::Mat disparityMap, depthMap;
    
    try{
        rclcpp::QoS qos = rclcpp::QoS(10);
        left_gray_sub.subscribe(this, input_left_gray_img, qos.get_rmw_qos_profile());
        right_gray_sub.subscribe(this, input_right_gray_img, qos.get_rmw_qos_profile());
        uint32_t q_size = 10;
        //initialize synchronizer - ApproximateTime allows minimal(few mil secs) mismatches.
        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(q_size), left_gray_sub, right_gray_sub
        );
        sync->setAgePenalty(0.50); // synchronizer's tolerance for timestamp difference between two imgs.
        sync->registerCallback(std::bind(&DepthMapNode::processSync, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "msg filter synchronizer initialized successfully");
    } catch(const std::exception& e){
        RCLCPP_INFO(this->get_logger(), "Failed to initialize QOS : %s", e.what());
        throw;
    }

    try{
        stereoSGBM = cv::StereoSGBM::create(
        min_disp, // minDisparity
        num_disp, // numDisparities
        window_size, // blockSize
        8*3*window_size*window_size, // P1 - high for smoothness
        32*3*window_size*window_size, // P2 
        1, // disp12MaxDiff
        63, // preFilterCap
        40, //uniquenessRatio
        150, //speckleWindowSize
        32, // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY // _3WAY 3-way optimized ver. faster than MODE_SGBM; slowest
        );
    } catch(const std::exception& e ){
        RCLCPP_INFO(this->get_logger(), "Failed to initialize SGBM : %s", e.what());
        throw;
    }
    RCLCPP_INFO(this->get_logger(), "END OF CONSTRUCTOR");
}

void DepthMapNode::processCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{   
    /*
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]

    inside of P matrix.
    Tx = -fx' * B, 
    B(baseline): - Tx / fx'
            
        [fx'  0  cx' Tx]
    P = [ 0  fy' cy' Ty]
        [ 0   0   1   0]
    */
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    double fx_prime = msg->p[0];
    double Tx = msg->p[3];

    if(Tx != 0.0){
        baseline = -Tx / fx_prime;
        // this->baseline = baseline;
        RCLCPP_INFO(this->get_logger(), "RIGHT_CAMERA detected.");
    }else{
        RCLCPP_INFO(this->get_logger(), "Tx = 0 if left_camera. This is left camera info. need right camera info");
    }
    calibration_loaded = true;
    RCLCPP_INFO(this->get_logger(),"Camera INFO: fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, B: %.3f", fx_, fy_, cx_, cy_, baseline);
}
void DepthMapNode::processDepthMap(const cv::Mat& left_gray, const cv::Mat& right_gray)
{
    // python: matrix[y][x] -> c++: matrix.at<type>(y,x)

    stereoSGBM->compute(left_gray, right_gray, disparityMap); // output is 16bit signed. actual = output/16.0
    depthMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32F); // output depth map intialize in zeros
    int process_pixels = 0;

    for (int y = 0; y < disparityMap.rows; ++y){
        for(int x = 0; x < disparityMap.cols; ++x){
            // type conversion for disparity
            int16_t disparity_raw = disparityMap.at<int16_t>(y,x); // eigen? c+==
            if(disparity_raw > 0)
            {
                float disparity = static_cast<float>(disparity_raw) / 16.0f;
                float depth = (fx_* baseline) / disparity;
                if(depth > 0.1f && depth <100.0f)
                {
                    // depth[y][x] = baseline * fx_ / [y][x];
                    depthMap.at<float>(y,x) = depth;
                    process_pixels++;
                }
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "DEPTH PROCESSED");
    RCLCPP_INFO(this->get_logger(), "VALID PIXELS: %d", process_pixels);
    cv::Mat depth_8u, rgb_depth_map;
    cv::normalize(depthMap, depth_8u, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(depth_8u, rgb_depth_map, cv::COLORMAP_JET);
    
    //publishing to ROS image msg
    cv_bridge::CvImage depth_bridge;
    depth_bridge.header.stamp = this->now();
    depth_bridge.header.frame_id = "gray_leftright";
    // depth_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_bridge.encoding = sensor_msgs::image_encodings::BGR8;
    // depth_bridge.image = depthMap;
    depth_bridge.image = rgb_depth_map;

    depth_map_pub->publish(*depth_bridge.toImageMsg());
}
void DepthMapNode::processSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg, const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
{   
    try{
        // use messagefilter to retreive synced images.
        cv_bridge::CvImagePtr left_cv_ptr = cv_bridge::toCvCopy(left_msg);
        cv_bridge::CvImagePtr right_cv_ptr = cv_bridge::toCvCopy(right_msg);
        cv::Mat left_gray = left_cv_ptr->image;
        cv::Mat right_gray = right_cv_ptr->image;
        processDepthMap(left_gray, right_gray);
    } catch(const std::exception& e){
        RCLCPP_INFO(this->get_logger(), "syncing error %s", e.what());
    }
}

DepthMapNode::~DepthMapNode(){}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DepthMapNode>();
        rclcpp::spin(node);
    } catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}