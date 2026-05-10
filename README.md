# Main Repo of Road Surface Repair Machine for Engineering Innovative and Design (EID) Project

## Team Members
- Software & Electronics Team: Ming Jie, Jin Hern, Tze Nin, Zi Jun
- Mechanical Team: Sean, Pei Wen, Yan Sheng, Jack, Joel

## TO-DO
Software
1) complete and integrate all node below
2) stepper microros node: publish coordinate + time stamp 
3) tof scan node: receive start command from foxglove - move stepper in fixed sequence - subscribe stepper coordinate + timestamp + tof data - fuse data - update costmap - publish hole centroid & volume - tell foxglove finish
4) Material filling node: receive start command from foxglove - find best sequence - request move to stepper action node - request fill to pump action node - update costmap - loop until fill all - tell foxglove finish
5) Stepper action node: receive request of coordinate - move to goal (including z axis!) - send back result say reached
6) Pump action node: receive request of volume - pump material - send back result say filled
8) tof node: subscribe to tof serial and publish
9) camera node: subscribe to camera serial and publish
10) foxglove show: tof costmap, camera view, wheel control button, tof scan start button, material filling start button, emergency stop button
11) launch file

Electrical
1) Print upper board and attach all stuff

Mechanical
1) attach wheel
2) test cam
3) load simulation?
4) z axis and pump attach
6) simulation? (Load test, etc)
7) pump calculation? See put pump where is better

### To Run the walking motor
#### Start the micro-ROS Agent (Terminal 1)
```bash
source /opt/ros/jazzy/setup.bash
source ~/Road_Surface_Repair_Machine/Software/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

#### Start the Foxglove Bridge (Terminal 2)
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
## More Coming Soon
