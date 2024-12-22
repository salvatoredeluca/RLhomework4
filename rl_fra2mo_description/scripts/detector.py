#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Transform, TransformStamped
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf_transformations

import os

class det_Aruco(Node):
    def __init__(self):
        super().__init__('det_aruco')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'topic_out', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.listener_callback,
            10
        )
        self.subscription  # prevenire l'errore di variabile non utilizzata

    def listener_callback(self, msg):
        self.get_logger().info(f'Ricevuto messaggio: {msg.data}')

        
        
        # Pubblica un messaggio su 'topic_out'
        self.publisher_.publish(String(data=f'Risposta a: {msg.data}'))
        self.get_logger().info(f'Pubblicato messaggio: Risposta a: {msg.data}')

def main():
    rclpy.init()
    
    node = det_Aruco()
    
    # Esegui il nodo finché non viene interrotto
    rclpy.spin(node)
    
    # Distruggi il nodo quando finito
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()