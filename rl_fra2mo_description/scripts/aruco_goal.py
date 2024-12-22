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

from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, quaternion_from_matrix, translation_from_matrix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
import tf2_geometry_msgs
import tf_transformations

import os

import yaml

waypoints_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "waypoints.yaml")
with open(waypoints_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)




class det_Aruco(Node):
    def __init__(self):
        super().__init__('det_aruco')

        # Publisher
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        #self.publisher_ = self.create_publisher(Pose, 'aruco_map', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.listener_callback,
            10
        )
        self.subscription  # prevenire l'errore di variabile non utilizzata

    def listener_callback(self, msg):
        self.get_logger().info(f'Ricevuto messaggio:')

################################################### trasformazione relativa alla posa finale rispetto coordinate di gazebo
        transform_a = Transform()
        transform_a.translation.x = -4.0
        transform_a.translation.y = 0.0
        transform_a.translation.z = 0.0
        quaternion_a = quaternion_from_euler(0.0, 0.0, -1.57)
        rotx_a = quaternion_a[0]
        roty_a = quaternion_a[1]
        rotz_a = quaternion_a[2]
        rotw_a = quaternion_a[3]

        #creo la prima trasformazione 
        
        transform_a.rotation.x = rotx_a
        transform_a.rotation.y = roty_a
        transform_a.rotation.z = rotz_a
        transform_a.rotation.w = rotw_a


        t1a_matrix = tf_transformations.translation_matrix([transform_a.translation.x,
                                                   transform_a.translation.y,
                                                   transform_a.translation.z])
        t1a_matrix = tf_transformations.concatenate_matrices(
            t1a_matrix,
            tf_transformations.quaternion_matrix([
            transform_a.rotation.x,
            transform_a.rotation.y,
            transform_a.rotation.z,
            transform_a.rotation.w,
        ])
        )
#####################################################################################################
#########################################################trasformazione base link-camera optical

        transform_b = Transform()
        transform_b.translation.x = 0.108
        transform_b.translation.y = 0.0
        transform_b.translation.z = 0.159
     
    
     
        transform_b.rotation.x = -0.5
        transform_b.rotation.y = 0.5
        transform_b.rotation.z = -0.5
        transform_b.rotation.w = 0.5


        t1b_matrix = tf_transformations.translation_matrix([transform_b.translation.x,
                                                   transform_b.translation.y,
                                                   transform_b.translation.z])
        t1b_matrix = tf_transformations.concatenate_matrices(
            t1b_matrix,
            tf_transformations.quaternion_matrix([
            transform_b.rotation.x,
            transform_b.rotation.y,
            transform_b.rotation.z,
            transform_b.rotation.w,
        ])
        )
    ###########################################################################################
    # #################################################à trasformazione tra aruco e optical

        transform_c = Transform()
        transform_c.translation.x = msg.pose.position.x
        transform_c.translation.y = msg.pose.position.y
        transform_c.translation.z = msg.pose.position.z
     
    
     
        transform_c.rotation.x = msg.pose.orientation.x
        transform_c.rotation.y = msg.pose.orientation.y
        transform_c.rotation.z = msg.pose.orientation.z
        transform_c.rotation.w = msg.pose.orientation.w


        t1c_matrix = tf_transformations.translation_matrix([transform_c.translation.x,
                                                   transform_c.translation.y,
                                                   transform_c.translation.z])
        t1c_matrix = tf_transformations.concatenate_matrices(
            t1c_matrix,
            tf_transformations.quaternion_matrix([
            transform_c.rotation.x,
            transform_c.rotation.y,
            transform_c.rotation.z,
            transform_c.rotation.w,
        ])
        )


##########################################################################################calcolo frame rispetto gazebo
        aruco_matrix=Transform()
        aruco_matrix = tf_transformations.concatenate_matrices(t1b_matrix, t1c_matrix)
        aruco_matrix = tf_transformations.concatenate_matrices(t1a_matrix,aruco_matrix)

        print(f"aruco_matrix: {aruco_matrix}")

##############################################################################################trasformazione gazebo-rviz

        #creo la trasformazione 2
        roll_g = 0.0
        pitch_g = 0.0
        yaw_g = -1.57
        quaternion_g = quaternion_from_euler(roll_g, pitch_g, yaw_g)
        transformg = Transform()
        transformg.translation.x = -3.0
        transformg.translation.y = 3.5
        transformg.translation.z = 0.0
        transformg.rotation.x = quaternion_g[0]
        transformg.rotation.y = quaternion_g[1]
        transformg.rotation.z = quaternion_g[2]
        transformg.rotation.w = quaternion_g[3]

        tg_matrix = tf_transformations.translation_matrix([transformg.translation.x,
                                                   transformg.translation.y,
                                                   transformg.translation.z])
        tg_matrix = tf_transformations.concatenate_matrices(
            tg_matrix,
            tf_transformations.quaternion_matrix([
            transformg.rotation.x,
            transformg.rotation.y,
            transformg.rotation.z,
            transformg.rotation.w,
        ])
        )

        Tg = tf_transformations.inverse_matrix(tg_matrix)
        aruco_map= tf_transformations.concatenate_matrices(Tg,aruco_matrix)
        print(f"aruco_map: {aruco_map}")
    

        ###############################################################################Broadcaster

        aruco_translation = translation_from_matrix(aruco_map)  # Restituisce [x, y, z]

# Estrarre rotazione come quaternione
        aruco_quaternion = quaternion_from_matrix(aruco_map)
   
        
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'aruco_marker_frame'

        t.transform.translation.x = aruco_translation[0]
        t.transform.translation.y = aruco_translation[1]
        t.transform.translation.z = aruco_translation[2]

        t.transform.rotation.x = aruco_quaternion[0]
        t.transform.rotation.y = aruco_quaternion[1]
        t.transform.rotation.z = aruco_quaternion[2]
        t.transform.rotation.w = aruco_quaternion[3]

        self.tf_static_broadcaster.sendTransform(t)
    


def main():
    rclpy.init()
    node = det_Aruco()
    
    navigator = BasicNavigator()

###################################################################prendo dati dallo yaml e lo converto in riferimento map
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
####################################################################################################################################

    goals = list(map(create_pose, yaml_content["waypoints"]))
    goal_poses = []

    strategy=yaml_content["mode"][0]["strategy"]
    print(f"La strategia è: {strategy}")
    

    target_nameex= "aruco"
    for pose, name  in goals:
        if name == target_nameex:
            goal_poses.append(pose)             


    #Wait for navigation to fully activate, since autostarting nav2
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
            if now - nav_start > Duration(seconds=600):
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
    executor= SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin_once(timeout_sec =2)


    navigator_finale = BasicNavigator()
    
    pose_finale = PoseStamped()
    pose_finale.header.frame_id = 'map'
    pose_finale.header.stamp = navigator.get_clock().now().to_msg()

    pose_finale.pose.orientation.x = 0.0
    pose_finale.pose.orientation.y = 0.0
    pose_finale.pose.orientation.z = 0.0
    pose_finale.pose.orientation.w = 1.0

    pose_finale.pose.position.x = 0.0
    pose_finale.pose.position.y = 0.0
    pose_finale.pose.position.z = 0.0

    goal_poses_finale = []
    goal_poses_finale.append(pose_finale)


    navigator_finale.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator_finale.get_clock().now()
    navigator_finale.followWaypoints(goal_poses_finale)

    i = 0
    while not navigator_finale.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator_finale.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses_finale)))
            now = navigator_finale.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator_finale.cancelTask()

    # Do something depending on the return code
    result = navigator_finale.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
        

    node.destroy_node()
    rclpy.shutdown()


    exit(0)


if __name__ == '__main__':
    main()