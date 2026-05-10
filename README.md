# ProjectTER (Project 50) - Autonomous Drone Swarm for S&R

Ce projet porte sur la simulation d'un système de drone autonome dédié à la recherche et au sauvetage (R&S), intégrant ROS2, PX4, et des algorithmes de vision par ordinateur.

## 📺 Démonstrations (Vidéos)
* **Détection humaine (YOLO) :** `Video/RecherchePersonne.mp4`
* **Reconstruction 3D (OctoMap) :** `Video/3D.mp4`

## 🏗 Architecture du Système
L'architecture de ce projet est basée sur les standards officiels de **ROS2**. Le système est divisé en plusieurs packages pour assurer la modularité entre la perception, le contrôle et la communication.

### 📂 Structure des Modules (Aperçu du Code)
Le dossier `qbd_insight/qbd_insight/` regroupe les scripts Python principaux pilotant le drone :

* **Contrôle & Mouvement** : 
    * `offboard.py` & `px4_test.py` : Gestion du mode Offboard et communication avec le pilotage automatique PX4.
    * `move_position.py` / `move_velocity.py` : Algorithmes de contrôle de vol basés sur la position et la vitesse.
    * `keyboard_position.py` / `keyboard_velocity.py` : Interface de téléopération par clavier.
* **Perception & Mapping** :
    * `yolo_node.py` & `track.py` : Détection et suivi d'objets en temps réel via YOLO.
    * `octomap_dynamic.py` : Génération de cartes 3D dynamiques pour l'évitement d'obstacles.
* **Télémétrie** :
    * `msg_px4_fmu_out_vehicle_status.py` : Analyse de l'état du système de vol.

### 🚀 Points Clés : Déploiement et Launch
Les scripts de lancement essentiels, qui orchestrent la simulation et les nœuds de calcul, se trouvent dans :
`qbd-ros2/ros2/src/qbd_insight/launch`

* **Logic de Mission :** Ces fichiers `.py` coordonnent l'intégration entre l'interface PX4, les capteurs (Lidar/Caméra) et les algorithmes de traitement.

## 📚 Références et Sources
La conception de ce projet s'appuie sur des standards industriels et des recherches académiques. 

> **Note sur l'intégrité académique :** Pour chaque module spécifique, les liens vers les dépôts GitHub de référence et la documentation officielle utilisés (code source adapté ou inspiré) sont systématiquement indiqués directement dans les **commentaires du code source**.

## 🛠 Technologies Utilisées
* **Middleware :** ROS2 (Humble/Foxy)
* **Simulation :** Webots, FlightGear, PX4, MATLAB
* **Vision :** YOLOv8, OpenCV
* **Cartographie :** OctoMap
* **Communication :** Modélisation 5G
