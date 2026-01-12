Cloner le projet et lancer la visualisation dans RViz
=====================================================

Objectif
--------

Cette section explique comment :

- récupérer le projet depuis mon GitHub;
- reconstruire le workspace ROS2;
- lancer la visualisation du robot dans RViz à partir du fichier URDF.

Lien vers la `page github du projet <https://github.com/matthieuuhry/UHRY_MUGNIER_Info_Indus_projet>`_.

Cloner le dépôt GitHub
----------------------

1. Récupérer l’URL du dépôt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sur la page GitHub du projet :

- cliquer sur le bouton **Code** ;
- copier l’URL HTTPS ou SSH du dépôt

2. Se placer dans le workspace ROS2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dans un terminal :

.. code-block:: bash

   cd ~/mon_projet/

Ici, ``~/ros2_ws`` sera le workspace ROS2 utilisé pour le projet.

3. Cloner le dépôt
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone https://github.com/matthieuuhry/UHRY_MUGNIER_Info_Indus_projet

Puis entrer dans le dossier du dépôt :

.. code-block:: bash

   cd mon_projet
   git status

À ce stade, tous les fichiers du projet sont disponibles localement
(description URDF, meshes ``.dae``, fichiers de lancement, documentation, etc.).

Compilation du workspace
------------------------

Le projet fait partie d’un workspace ROS2, donc :

1. Revenir à la racine du workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/mon_projet/ros2_ws

2. Construire le workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2_jazzy
   ros2_build

Cette commande compile l’ensemble des packages présents dans ``src`` et
met à jour le dossier ``install`` du workspace.

3. Sourcer l’environnement
^^^^^^^^^^^^^^^^^^^^^^^^^^

Après la compilation (et à chaque nouveau terminal) :

.. code-block:: bash

   source install/setup.bash

Cela ajoute les packages du workspace (dont la description du robot)
au chemin de recherche ROS2.

Lancer la visualisation du robot dans RViz
------------------------------------------

Le projet fournit un fichier URDF (par exemple dans
``scara_description/urdf/``) et un ou plusieurs fichiers de lancement
permettant d’ouvrir RViz directement avec le robot.

1. Lancer le node robot_state_publisher + RViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Lancer la commande :

.. code-block:: bash

   ros2 launch scara_bringup scara.launch.py

Ce fichier de lancement se charge de :

- charger le URDF décrit dans le précédent chapitre ;
- démarrer ``robot_state_publisher`` avec la description du robot ;
- lancer RViz avec une configuration préenregistrée (positions, topics,
  affichage des liens, etc.).

2. Visualiser le robot
^^^^^^^^^^^^^^^^^^^^^^

Une fois RViz ouvert :

- la base du robot doit apparaître fixée au repère ``world`` ;
- les différents maillons (``link1``, ``link2``, etc.) s’affichent à partir
  des meshes ``.dae`` générés précédemment ;
- les articulations peuvent être animées en modifiant les valeurs de joint.

Résumé des commandes à exécuter
-------------------------------

Pour un nouvel utilisateur qui part de zéro (workspace déjà créé) :

.. code-block:: bash

   # Aller dans le dossier des sources du workspace ROS2
   cd ~/mon_projet/

   # Cloner le dépôt de l’étudiant
   git clone https://github.com/mon-user/mon-projet-ros2.git

   # Revenir à la racine du workspace
   cd ~/ros2_ws

   # Construire le workspace
   ros2_jazzy
   ros2_build

   # Sourcer l’environnement
   source install/setup.bash

   # Lancer la visualisation du robot dans RViz
   ros2 launch scara_bringup scara.launch.py

   


