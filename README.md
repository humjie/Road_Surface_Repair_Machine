# Main Repo of Road Surface Repair Machine for Engineering Innovative and Design (EID) Project

## Team Members
- Software & Electronics Team: Ming Jie, Jin Hern, Tze Nin, Zi Jun
- Mechanical Team: Sean, Pei Wen, Yan Sheng, Jack, Joel


## Installation (Under Construction)
1) install microros
'''bash
https://github.com/micro-ROS/micro_ros_setup
'''

2) install foxglove
'''bash
cd ~/ros2_jazzy/src
git clone https://github.com/facontidavide/rosx_introspection.git
git clone https://github.com/foxglove/foxglove-sdk.git
cd ~/ros2_jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to foxglove_bridge --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_jazzy/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ros2 run foxglove_bridge foxglove_bridge 
'''

3) setup for camera
'''bash
cd ~/ros2_jazzy/src
git clone -b rolling https://github.com/ros-perception/vision_opencv.git
cd ~/ros2_jazzy
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cv_bridge
'''

## More Coming Soon
