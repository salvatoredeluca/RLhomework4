#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Transform, TransformStamped
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf_transformations

import os

import yaml

waypoints_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "waypoints.yaml")
with open(waypoints_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)

# waypoints = yaml.safe_load('''
# waypoints:
#   - position:
#       x: 2.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.0055409271259092485
#       w: 0.9999846489454652
#   - position:
#       x: -1.8789787292480469
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.010695864295550759
#       w: 0.9999427976074288
#   - position:
#       x: 0.0
#       y: 0.6118782758712769
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.01899610435153287
#       w: 0.9998195577300264
# ''')

def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        #creo il messaggio
        name= transform["name"]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        #mi prendo i valori da yaml e trasformo in quaternioni
        x_yaml = transform["position"]["x"]
        y_yaml = transform["position"]["y"]
        z_yaml = transform["position"]["z"]
        roll = transform["orientation"]["roll"]
        pitch = transform["orientation"]["pitch"]
        yaw = transform["orientation"]["yaw"]
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        rotx_yaml = quaternion[0]
        roty_yaml = quaternion[1]
        rotz_yaml = quaternion[2]
        rotw_yaml = quaternion[3]

        #creo la prima trasformazione 
        transform1 = Transform()
        transform1.translation.x = x_yaml
        transform1.translation.y = y_yaml
        transform1.translation.z = z_yaml
        transform1.rotation.x = rotx_yaml
        transform1.rotation.y = roty_yaml
        transform1.rotation.z = rotz_yaml
        transform1.rotation.w = rotw_yaml

        t1_matrix = tf_transformations.translation_matrix([transform1.translation.x,
                                                   transform1.translation.y,
                                                   transform1.translation.z])
        t1_matrix = tf_transformations.concatenate_matrices(
            t1_matrix,
            tf_transformations.quaternion_matrix([
            transform1.rotation.x,
            transform1.rotation.y,
            transform1.rotation.z,
            transform1.rotation.w,
        ])
        )

        #creo la trasformazione 2
        roll_2 = 0.0
        pitch_2 = 0.0
        yaw_2 = -1.57
        quaternion_2 = quaternion_from_euler(roll_2, pitch_2, yaw_2)
        transform2 = Transform()
        transform2.translation.x = -3.0
        transform2.translation.y = 3.5
        transform2.translation.z = 0.0
        transform2.rotation.x = quaternion_2[0]
        transform2.rotation.y = quaternion_2[1]
        transform2.rotation.z = quaternion_2[2]
        transform2.rotation.w = quaternion_2[3]

        t2_matrix = tf_transformations.translation_matrix([transform2.translation.x,
                                                   transform2.translation.y,
                                                   transform2.translation.z])
        t2_matrix = tf_transformations.concatenate_matrices(
            t2_matrix,
            tf_transformations.quaternion_matrix([
            transform2.rotation.x,
            transform2.rotation.y,
            transform2.rotation.z,
            transform2.rotation.w,
        ])
        )

        T_inv = tf_transformations.inverse_matrix(t2_matrix)

        composed_matrix = tf_transformations.concatenate_matrices(T_inv,t1_matrix)

        translation_way = composed_matrix[:3, 3]
        quaternion_way = quaternion_from_matrix(composed_matrix)

        #composizione

        pose.pose.orientation.x = quaternion_way[0]
        pose.pose.orientation.y = quaternion_way[1]
        pose.pose.orientation.z = quaternion_way[2]
        pose.pose.orientation.w = quaternion_way[3]

        pose.pose.position.x = translation_way[0]
        pose.pose.position.y = translation_way[1]
        pose.pose.position.z = translation_way[2]
        return pose, name

    goals = list(map(create_pose, yaml_content["waypoints"]))
    goal_poses = []

    strategy=yaml_content["mode"][0]["strategy"]
    print(f"La strategia è: {strategy}")
    
    if strategy == "path":
        target_name1= "Goal_3"
        for pose, name in goals:
            if name == target_name1:
                goal_poses.append(pose)
                print(f"pose: {pose}")
    
        target_name2= "Goal_4"
        for pose, name in goals:
            if name == target_name2:
                goal_poses.append(pose)
                print(f"pose: {pose}")

        target_name3= "Goal_2"
        for pose, name in goals:
            if name == target_name3:
                goal_poses.append(pose)
                print(f"pose: {pose}")

        target_name4= "Goal_1"
        for pose, name in goals:
            if name == target_name4:
                goal_poses.append(pose)
                print(f"pose: {pose}")

    else:
        target_nameex= "explore"
        for pose, name  in goals:
            if name == target_nameex:
                goal_poses.append(pose)             


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=6000):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()