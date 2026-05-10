# ProjectTER (Project 50) - Autonomous Drone Swarm for S&R

Ce projet porte sur la simulation d'un système de drone autonome dédié à la recherche et au sauvetage (R&S), intégrant ROS2, PX4, et des algorithmes de vision par ordinateur.

## 📺 Démonstrations (Vidéos)
*   **Détection humaine (YOLO) :** `Video/RecherchePersonne.mp4`
*   **Reconstruction 3D (OctoMap) :** `Video/3D.mp4`

## 🏗 Architecture du Système
L'architecture de ce projet est basée sur les standards officiels de **ROS2**. Le système est divisé en plusieurs packages pour assurer la modularité entre la perception, le contrôle et la communication.

### 🚀 Points Clés : Déploiement et Launch
Les scripts de lancement essentiels, qui orchestrent la simulation et les nœuds de calcul, se trouvent dans :
`qbd-ros2/ros2/src/qbd_insight/launch`

*   **Logic de Mission :** Ces fichiers `.py` coordonnent l'intégration entre l'interface PX4, les capteurs (Lidar/Caméra) et les algorithmes de traitement.

## 📚 Références et Sources
La conception de ce projet s'appuie sur des standards industriels et des recherches académiques. 
> **Note :** Pour chaque module spécifique, les liens vers les dépôts GitHub de référence et la documentation officielle utilisés sont directement annotés dans les **commentaires du code source**.

## 🛠 Technologies Utilisées
*   **Middleware :** ROS2 (Humble/Foxy)
*   **Simulation :** Webots, FlightGear, PX4
*   **Vision :** YOLOv8, OpenCV
*   **Cartographie :** OctoMap
*   **Communication :** Modélisation 5G
