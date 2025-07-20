## Build and Run

```bash
cd ~/kitti_ws
colcon build --packages-select hands_on_kitti
source install/setup.bash
ros2 run hands_on_kitti listener ../kitti_ros2_bag/kitti_ros2_bag.db3
```

## Visualize Images

**RQT Image View:**
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/kitti/camera_color_left/image_raw` from dropdown.

**RViz2:**
```bash
ros2 run rviz2 rviz2
```
Add Image display → Set topic to `/kitti/camera_color_left/image_raw`.