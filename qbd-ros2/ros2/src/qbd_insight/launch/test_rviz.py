import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory 
from launch.actions import ExecuteProcess

# https://github.com/ros2/examples/tree/rolling/launch_testing/launch_testing_examples/launch_testing_examples

def generate_launch_description():
    depth_camera_relay_node = Node(
        package='topic_tools',        # 节点所属的包  Le paquet auquel appartient le nœud
        executable='relay',           # 可执行文件名称（对应 ros2 run 的 relay） Nom du fichier exécutable (correspondant au relais de ros2 run)
        name='depth_camera_relay',    # 节点名称 Nom du nœud
        arguments=[
            '/airsim_node/PX4/CameraDepth/DepthPerspective/camera_info', # 源话题  Sujet original
            '/airsim_node/PX4/CameraDepth/camera_info'                   # 目标话题 Sujet cible
        ],
        output='screen'               # 日志输出到终端 Journalisation des sorties vers le terminal
    )

    image_camera_relay_node = Node(
        package='topic_tools',        # 节点所属的包  Le paquet auquel appartient le nœud
        executable='relay',           # 可执行文件名称（对应 ros2 run 的 relay） Nom du fichier exécutable (correspondant au relais de ros2 run)
        name='image_camera_relay',    # 节点名称 Nom du nœud
        arguments=[
            '/airsim_node/PX4/CameraImage/Scene/camera_info',  # 源话题  Sujet original
            '/airsim_node/PX4/CameraImage/camera_info'         # 目标话题 Sujet cible
        ],
        output='screen'               # 日志输出到终端 Journalisation des sorties vers le terminal
    )

    qbd_move_velocity_node = Node(
            package='qbd_insight',
            executable='move_velocity',
            name='move_velocity',
            output='screen')

    pkg_share = get_package_share_directory('qbd_insight')
    depth_rviz_path = os.path.join(pkg_share, 'rviz/depth_cloud.rviz')
    image_lidar_rviz_path = os.path.join(pkg_share, 'rviz/image_lidar.rviz')

    qbd_rviz_depth_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', depth_rviz_path]
    )

    qbd_rviz_image_lidar_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='image_lidar_rviz2',
            arguments=['-d', image_lidar_rviz_path]
    )

    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(qbd_move_velocity_node)
    ld.add_action(depth_camera_relay_node)
    ld.add_action(image_camera_relay_node)
    ld.add_action(qbd_rviz_depth_node)
    ld.add_action(qbd_rviz_image_lidar_node)
    return ld
