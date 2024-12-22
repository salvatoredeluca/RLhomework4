
# FOLLOWING THE WAYPOINTS
Spawn the robot in Gazebo
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

```

Load the NAV2 stack and the SLAM node

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py

```
Visualize the robot on RVIZ
```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```

Make the robot follow the desired waypoints
```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py rviz_file_name:=goals.rviz
```

Make the roboto explore the map

```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py rviz_file_name:=explore.rviz

```

# VISION-BASED TASK
Spawn the robot on Gazebo
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

```
Launch the NAV2 stack,the SLAM node and the single node of the aruco_ros package in order to detect the Aruco marker
```bash
ros2 launch rl_fra2mo_description fr2mo_aruco.launch.py

```
To visualize the correct publication of the topic run on another terminal
```bash
ros2 topic echo /tf_static

```
To visualize the result of the detection run

```bash
rqt

```
Make the Robot move sufficiently close to the Aruco to execute the detection

```bash
ros2 run rl_fra2mo_description aruco_goal.py

```








