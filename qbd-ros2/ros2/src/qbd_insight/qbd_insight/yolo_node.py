#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from qbd_interface.msg import QBDSimpleKeyboardInfo
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory 
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
import message_filters
import time

# https://github.com/mgonzs13/yolo_ros/blob/main/yolo_ros/yolo_ros/yolo_node.py

# 需要让RGB和DEPTH图像的时间戳完全对应才行
# Les horodatages des images RGB et DEPTH doivent être parfaitement cohérents.

class YOLODetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        # 读取相机名字参数
        # Lire les paramètres du nom de la caméra
        self.declare_parameter('camera_name', "CameraDepth1")
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # 订阅相机的RGB图片和深度图片。RGB图片用来给YOLO识别物体，深度图片用来根据YOLO识别的结果计算坐标值
        # Abonnez-vous aux images RGB et de profondeur de la caméra. Les images RGB sont utilisées par YOLO pour identifier les objets, et les images de profondeur servent à calculer les coordonnées à partir des résultats de la reconnaissance YOLO.
        self.rgb_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/Scene')
        self.depth_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/DepthPlanar')
        # 创建一个近似时间同步器，确保RGB图片和DEPTH图片的同步
        # Créer un synchroniseur temporel approximatif pour assurer la synchronisation entre les images RGB et DEPTH.
        # 参数：订阅者列表，队列大小，时间戳容差（秒）
        # Paramètres : liste des abonnés, taille de la file d’attente, tolérance d’horodatage (secondes)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 2, 0.01) 
        # 注册同步后的回调函数
        # Fonction de rappel après l'enregistrement et la synchronisation
        self.time_synchronizer.registerCallback(self.sync_callback)

        # 读取YOLO模型参数，选择合适的模型
        # Lire les paramètres du modèle YOLO et sélectionner un modèle approprié
        self.declare_parameter('yolo_model', "yolov8n.pt")
        self.yolo_model = self.get_parameter('yolo_model').get_parameter_value().string_value

        # 读取YOLO模型，CvBridge用来在RGB和ROS的image格式间进行转换
        # Lire le modèle YOLO; CvBridge est utilisé pour convertir entre les formats d’image RGB et ROS.
        pkg_share = get_package_share_directory('qbd_insight')
        yolo_model_path = os.path.join(pkg_share, 'yolo/'+self.yolo_model) # 可换成其他模型
        self.model = YOLO(yolo_model_path)
        self.bridge = CvBridge()

        # 用来发布YOLO识别后，带识别方框的RGB图片
        # Utilisé pour publier des images RGB avec des boîtes englobantes de reconnaissance YOLO après la reconnaissance.
        self.yolo_pub = self.create_publisher(Image, '/yolo/output', 1)
        self.point_pub = self.create_publisher(PointStamped, '/yolo/person_position', 1)

        # TF2 buffer + listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Camera={self.camera_name}, YOLO={self.yolo_model}, Started.")


    # 订阅深度图信息的回调函数
    # Fonction de rappel pour l'abonnement aux informations de la carte de profondeur
    def depth_callback(self, msg):
        try:
            # 缓存最近的深度图信息
            # Mettre en cache les informations de carte de profondeur les plus récentes
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")


    # 同步回调函数，它会同时接收到 rgb_msg 和 depth_msg
    # Fonction de rappel synchrone, qui recevra simultanément rgb_msg et depth_msg.
    def sync_callback(self, rgb_msg, depth_msg):
        #这个回调函数只在接收到时间戳相近的RGB和深度图时才被触发。
        # Cette fonction de rappel n'est déclenchée que lorsque des cartes RGB et de profondeur avec des horodatages similaires sont reçues.
        try:
            # 将ROS图像消息转换为OpenCV图像
            # Convertir les messages d'images ROS en images OpenCV
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # 使用YOLO模型进行目标检测, classes=[0] 表示只检测 "person" 类别, verbose=False 减少控制台输出
        # Utiliser le modèle YOLO pour la détection d'objets. classes=[0] signifie détecter uniquement la catégorie «personne». verbose=False réduit les informations affichées dans la console.
        # conf置信度0.6以上
        # niveau de confiance supérieur à 0,6
        results = self.model(img, classes=[0], conf=0.6, verbose=False) 

        # 可视化检测结果
        # Résultats de détection visualisés
        annotated_frame = results[0].plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        out_msg.header = rgb_msg.header
        self.yolo_pub.publish(out_msg)

        # 如果检测到目标
        # Si la cible est détectée
        if len(results[0].boxes) > 0:
            # 只处理第一个检测到的目标
            # Ne traiter que la première cible détectée
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # 计算目标中心点像素坐标
            # Calculer les coordonnées en pixels du point central cible
            u = (x1 + x2) // 2
            v = y2

            # 检查像素坐标是否在图像范围内
            # Vérifier si les coordonnées des pixels se trouvent dans la plage de l'image
            if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
                self.get_logger().warn("Pixel coordinate out of depth image bounds.")
                return

            # 从同步好的深度图中获取深度值
            # Obtenir les valeurs de profondeur à partir d'une carte de profondeur synchronisée
            depth = depth_image[v, u]
            
            # 检查深度值是否有效
            # Vérifier si la valeur de profondeur est valide
            if depth == 0 or np.isinf(depth) or np.isnan(depth):
                self.get_logger().warn(f"Invalid depth value at ({u},{v}): {depth}")
                return
            
            #self.get_logger().info(f"Detected person at pixel ({u}, {v}) with depth: {depth:.2f} meters")

            # 相机内参, 分辨率 800x600，水平视场角 FOV = 120°
            # Paramètres internes de la caméra : Résolution 800x600, Champ de vision horizontal (FOV) = 120°
            HFOV_deg = 120.0
            HFOV_rad = math.radians(HFOV_deg)
            W = 800.0
            H = 600.0
            # 根据公式 fx = W / (2 * tan(FOV/2)) 计算焦距（像素）
            # Calculez la distance focale (en pixels) en utilisant la formule fx = W / (2 * tan(FOV/2)).
            fx = W / (2 * math.tan(HFOV_rad / 2))
            fy = fx  # AirSim像素是方形的 Les pixels de l'AirSim sont carrés
            cx = W / 2
            cy = H / 2

            # 像素坐标到相机坐标系转换
            # Conversion des coordonnées de pixels en coordonnées de la caméra
            x_cam = (u - cx) * depth / fx
            y_cam = (v - cy) * depth / fy
            z_cam = float(depth)

            # 创建 PointStamped 消息
            # Créer un message PointStamped
            point_camera = PointStamped()
            point_camera.header = rgb_msg.header
            point_camera.point.x = x_cam
            point_camera.point.y = y_cam
            point_camera.point.z = z_cam

            try:
                target_frame = 'PX4_odom'
                point_odom = self.tf_buffer.transform(
                    point_camera,
                    target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.01) # 设置一个合理的超时
                )

                #self.get_logger().info(f"Person position in {target_frame}: x={point_odom.point.x:.2f}, y={point_odom.point.y:.2f}, z={point_odom.point.z:.2f}")
                self.point_pub.publish(point_odom)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"TF transform failed: {e}")


def main(args=None) -> None:
    print('Starting YOLO test1 node...')
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
