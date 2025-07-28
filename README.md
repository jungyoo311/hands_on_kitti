# hands_on_kitti
The goal of this task is to generate a ROS2 KITTI bag file and visualize it on Rviz. 

## Description
The goal of this task is to generate a ROS2 KITTI bag file and visualize its contents (specifically raw RGB images and LiDAR point clouds) on RViz. This repository provides the necessary steps and a framework to achieve this.

## TODOs
- [x] Generate ROS2 version KITTI dataset
  - Link: docker_4_ros1.txt
- [x] Create a node to subscribe to the topics in the bagfile
- [x] Visualize raw RGB images on RViz
- [ ] Visualize the LiDAR point cloud on RViz
- [ ] Look up what is inside the `tf`/`tf_static` topic and assign the correct frame for each component
  - **Hint:** If each component is visualized in the correct frame, you can see the relative position of the components.

