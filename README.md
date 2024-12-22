
# FOLLOWING THE WAYPOINTS
Spawn the robot in Gazebo
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

```

Load the NAV2 stack and the SLAM node

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py

```
Visualize the robot on RVIZ with the proper configuration for the waypoints task

```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py rviz_file_name:=goals.rviz
```
To execute the corrects waypoints go to the config folder of the rl_fra2mo_description package and change the strategy in "path" then run the command

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```
# EXPLORING THE MAP

Spawn the robot in Gazebo
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

```

Load the NAV2 stack and the SLAM node

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py

```


Visualize the robot on RVIZ with the proper configuration for the explore task
```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py rviz_file_name:=explore.rviz

```

To execute the corrects waypoints go to the config folder of the rl_fra2mo_description package and change the strategy in "explore" then run the command
```bash
ros2 run rl_fra2mo_description follow_waypoints.py
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
Make the Robot move sufficiently close to the Aruco to execute the detection running the command

```bash
ros2 run rl_fra2mo_description aruco_goal.py

```








