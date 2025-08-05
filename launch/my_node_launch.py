# launch a single launch file and play bagfile
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # bag_reader_node = Node(
    #     package="hands_on_kitti",
    #     executable="reader",
    #     name="bag_reader_node",
    #     output='screen'
    # )
    depth_map_node = Node(
        package="hands_on_kitti",
        executable="depth_map", # should match cmakelist txt file
        name="depth_map_node",
        output='screen',
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen'
    )
    
    ld.add_action(depth_map_node)
    ld.add_action(rviz_node)
    return ld