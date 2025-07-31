# launch a single launch file and play bagfile
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    bag_reader_node = Node(
        package="hands_on_kitti",
        executable="reader",
        name="bag_reader_node",
        output='screen'
    )
    # change_perspective = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="vehicle_frame",
    #     arguments=["0", "0", "-1.73", "0", "0", "0", "world", "base_link"],
    #     output='screen'
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen'
    )
    
    ld.add_action(bag_reader_node)
    # ld.add_action(change_perspective)
    ld.add_action(rviz_node)
    return ld