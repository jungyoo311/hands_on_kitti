# launch a single launch file and play bagfile
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # this->declare_parameter<std::string>("bag_path", "../kitti_ros2_bag/kitti_ros2_bag.db3");
        # this->declare_parameter<std::string>("raw_image", "/kitti/camera/color_left/image_raw");
        # this->declare_parameter<std::string>("point_cloud", "/kitti/velo/pointcloud");
        # this->declare_parameter<std::string>("tf", "/tf");
    ld = LaunchDescription()
    bag_reader_node = Node(
        package="hands_on_kitti",
        executable="reader",
        name="bag_reader_node",
        parameters=[{
            "bag_path": "../kitti_ros2_bag/kitti_ros2_bag.db3",
            "raw_image": "/kitti/camera_color_left/image_raw",
            "point_cloud":"/kitti/velo/pointcloud",
            "tf": "/tf"
        }],
        output='screen'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen'
    )
    ld.add_action(bag_reader_node)
    ld.add_action(rviz_node)
    return ld