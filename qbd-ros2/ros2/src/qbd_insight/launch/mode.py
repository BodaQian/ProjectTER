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

# https://github.com/ros-perception/image_pipeline/blob/rolling/image_proc/launch/image_proc.launch.py

def generate_launch_description():
    pkg_share = get_package_share_directory('qbd_insight')

	# RVIZ2显示配置
    # Configuration d'affichage RVIZ2
    mode_rviz_path = os.path.join(pkg_share, 'rviz/mode.rviz')
    mode_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='depth_rviz2',
            arguments=['-d', mode_rviz_path]
    )

	# 将深度摄像头的信息转换为点云信息，再将点云信息发送给octomap_server
    # Convertir les informations de la caméra de profondeur en informations de nuage de points, puis envoyer ces informations de nuage de points au serveur octomap
    mode_depth_image_proc_node = launch_ros.actions.Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_to_cloud',
            remappings=[
                ('image_rect', '/airsim_node/PX4/CameraDepth/DepthPlanar'),
                ('camera_info', '/airsim_node/PX4/CameraDepth/camera_info'),
                ('points', '/depth_camera/pointers')
            ]
    )

    # 用上面depth_image_proc转换出的点云信息生成为地图
    # Générer une carte à partir des informations du nuage de points converties par depth_image_proc ci-dessus.
    mode_octomap_server_node = launch_ros.actions.Node(
            package='octomap_server',          # 包名         Nom du paquet
            executable='octomap_server_node',  # 可执行文件名   Nom du fichier exécutable
            name='octomap_server',             # 节点名称      Nom du nœud
            parameters=[
                {'frame_id': 'world_ned'},         # 地图坐标系，以世界地图为坐标  Système de coordonnées cartographiques
                {'base_frame_id': 'PX4/CameraDepth_optical'},  #无人机参考坐标 coordonnées de référence du drone # PX4/CameraDepth_optical, PX4/CameraDepth_body, PX4/CameraDepth_body/static
                {'resolution': 0.25},              # 地图分辨率，单位：米，当前0.25米的分辨率 Résolution de la carte, unité : mètres, résolution actuelle : 0,25 mètre.
                {'compress_map': False},           # 是否压缩地图 Compresser la carte ?
                {'ground_filter': False},          # 过滤地面相关的配置 Configurations liées au filtrage
                {'ground_filter.distance': 0.05},
                {'ground_filter.plane_distance': 0.06},
                {'sensor_model.max_range': 25.0},  # 传感器最大有效距离 Distance maximale effective du capteur
                {'publish_free_space': False},     # 是否发布空的地点值(无阻挡的空间)，避障逻辑会使用 La logique d'évitement d'obstacles utilisera la publication ou non d'une valeur de localisation vide (espace dégagé).
                {'use_height_map': True},          # 地图是否用不同的颜色标记高度 La carte utilise-t-elle des couleurs différentes pour indiquer l'altitude ?
                {'dynamic': False},                # 不启用动态边界扩展 Désactiver l'extension dynamique des limites
                {'point_cloud_max_x': 200.0},
                {'point_cloud_min_x': -200.0},
                {'point_cloud_max_y': 200.0},
                {'point_cloud_min_y': -200.0},
                {'point_cloud_max_z': -0.17},
                {'point_cloud_min_z': -50.0},
                {'occupancy_max_z': -0.17},
                {'occupancy_min_z': -50.0},
                {'min_x_size': 0.0},
                {'min_y_size': 0.0},
                {'latch': False}                  # True for a static map, false if no initial map is given。设置为false能够加快处理速度
            ],
            remappings=[
                ('cloud_in', '/depth_camera/pointers')  # 输入点云话题重映射 Réorganisation des sujets du nuage de points d'entrée
            ]
    )

	# 启动AirSim节点
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(airsim_node_launch)
    ld.add_action(mode_depth_image_proc_node)
    ld.add_action(mode_octomap_server_node)
    ld.add_action(mode_rviz_node)
    return ld
