
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



ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_stepper









help me check and modify the code if necessary


1) tof_costmap_node.py should always listen to /main_state, when /main_state become "scanning", it will start the operations. its main tasks is use the certain algo to control the stepper to scan through the range between x and y min max. the stepper will be control by /target_xy published by tof_costmap_node. while doing this, tof_costmap_node will subscribe to /current_xy_pos published by stepper and /tof_data published by tof_publisher. it will combine both message using the time stamp, and publish /tof_costmap

2) stepper should always listen to /main_state, when /main_state = "homing", go home, then publish /change_main_state = "free".

3) remove all the homing for ?. homing operation should included in other node but not specified by main_state. main_state_publisher should publish /main_state = "filling" if the /change_main_state = "filling" when /main_state = "free"

4) stepper should always listen to /target_xy and control stepper to move. it also always publish stepper msg /current_pos

5) remove all the /stepper_state and /main_cmd as they are redundent. the state control will all made by /main_state and /change_main_state

6) tof_visualiser will always listen to /tof_costmap, and do the same things as the previous code specified. also, it need to publish the result /tof_result that contain volume and coordinates of holes.

6) filling_control.py should always listen to /main_state. if /main_state = "filling", it will start its operation. it should listen to the /tof_result published by tof_visualiser, then compute a algorithm to fill the hole in the optimal path planning sequence. (move the algo from tof_visualiser to here). when start, it will do homing first for xyz. next, it will start the filling control. next, it will control the stepper through /target_xy and /target_z and pump through /pump_cmd. for each hole, move to the centroid above, lower down z axis, then pump. then after done, zaxis go up, move to next. while doing this, it will listen to /current_xy_pos and /current_z_pos for the control. 

7) pump and cam control are both in pump_cam

8) modify A-axisMotor code so that it will do homing like the stepper code. also subcribe to /target_z and publish /current_z_pos

9) modify the launch file "/home/rsrmstayhard/Road_Surface_Repair_Machine/Software/ros2_jazzy/src/rsrm_bringup/launch/robot_start.launch.py" to start up everythings.

10) wheel will listen to "/cmd_vel"

11) custom_camera always publish and foxglove listen to display



colcon build --packages-select custom_camera filling_control main_state_repeater rsrm_bringup tof_costmap tof_publisher tof_visualiser


zaxis homing?


cam cannot connect microrosagent
stepper cant run
