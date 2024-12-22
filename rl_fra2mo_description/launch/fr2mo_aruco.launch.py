from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    path1 = get_package_share_directory('rl_fra2mo_description')
    
    path2 = get_package_share_directory('aruco_ros')

    path1_c=os.path.join(path1, "launch","fra2mo_explore.launch.py")
    #path2_c=os.path.join(path2, "aruco_ros", "aruco_ros", "launch", "single.launch.py")
    path2_c=os.path.join(path2, "launch", "single.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path1_c),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path2_c),
        ),
    ])