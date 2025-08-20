## Demo
![RGB Depth Map](assets/color_depth_map.png)
![Grayscale Depth Map](assets/grayscale_depth_map.png)

## Visualize Images and Point Clouds
bag playback + processing node + visualization
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
