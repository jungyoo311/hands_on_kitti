## Build and Run

```bash
cd ~/kitti_ws
colcon build --packages-select hands_on_kitti
source install/setup.bash
ros2 launch hands_on_kitti my_node_launch.py
```
## JJ Feedback
**Launch File will run the node and RVIZ2. LIDAR have the correct tf2 so that I can see the difference between car frame and lidar frame perspective.**


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