## Visualize Images and Point Clouds

**Build and Source:**
```bash
colcon build --packages-select hands_on_kitti
source install/setup.bash
```
Select `/kitti/camera_color_left/image_raw` from dropdown.

**Run:**
```bash
ros2 launch hands_on_kitti my_node_launch.py
```

**Data Structure**
```bash
# Show the structure of any ros2 msg types
ros2 interface show tf2_msgs/msg/TFMessage
```
## JJ Feedback
**Launch File will run the node and RVIZ2. LIDAR have the correct tf2 so that I can see the difference between car frame and lidar frame perspective.**


