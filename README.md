# Projet TurtleBot3 - ROS2 Navigation Challenge

## Contexte du projet

Ce projet a été réalisé dans le cadre du cours **ROS et robotique expérimentale**. Il consiste à contrôler un robot **TurtleBot3** pour le faire naviguer de manière autonome dans un environnement simulé et réel. Le robot doit accomplir plusieurs défis, incluant le suivi de lignes, l'évitement d'obstacles, la navigation en corridor et le marquage de buts, en utilisant les capteurs LDS et caméra.

## Objectifs

* Détecter et suivre des lignes à l'aide de la caméra embarquée.
* Éviter les obstacles détectés par le LIDAR.
* Naviguer dans un corridor étroit sans collision.
* Pousser une balle vers un but pour marquer un point.


## Lancer les défis

1. **Build du package**

```bash
cd ~/ros2_ws
colcon build --packages-select projet
source install/setup.bash
```

2. **Lancer les défis**

* **Défi 1 - Suivi de ligne**

```bash
ros2 run robot_perception follow
```

* **Défi 2 - Évitement d'obstacles**

```bash
ros2 launch robot_navigation lane_avoid.launch.py
```

* **Défi 3 - Navigation en corridor**

```bash
ros2 run robot_perception corridor
```



