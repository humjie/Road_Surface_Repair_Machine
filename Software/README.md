
## Installation (Under Construction)
1) install microros
```bash
https://github.com/micro-ROS/micro_ros_setup
```

2) install foxglove
```bash
cd ~/ros2_jazzy/src
git clone https://github.com/facontidavide/rosx_introspection.git
git clone https://github.com/foxglove/foxglove-sdk.git
cd ~/ros2_jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to foxglove_bridge --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_jazzy/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ros2 run foxglove_bridge foxglove_bridge 
```

3) setup for camera
```bash
cd ~/ros2_jazzy/src
git clone -b rolling https://github.com/ros-perception/vision_opencv.git
cd ~/ros2_jazzy
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cv_bridge

ros2 run custom_camera custom_camera
```

4) install micro_ros
```bash
mkdir -p /microros_ws/src
cd microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```



add the fixed sequence move function into stepper code. update respective scan algo in tof costmap node. follow the below flow:

receive start command from a topic "main_cmd"? or other appropiate name - move stepper in fixed sequence while tof costmap node subscribe stepper coordinate + timestamp + tof data - fuse data - update costmap - publish hole centroid & volume - publish "main_state"

integrate stepper state with main_state, combine them if necessary
main_state topic will have multiple state such as "scanning, homing, available, nohome, filling, wheelmoving, etc". change the state name appropiately