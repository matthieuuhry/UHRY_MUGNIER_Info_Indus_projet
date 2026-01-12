Cloner le dépôt GitHub du projet
================================

Pourquoi cloner le dépôt du professeur ?
----------------------------------------

Pour commencer le projet, il fallait récupérer :
- les fichiers de description du robot (dont le fichier ``.urdf``) ;
- les packages ROS2 fournis (lancement, configuration, etc.) ;
- la structure déjà prête pour visualiser le robot dans RViz.

Cloner le dépôt GitHub du professeur permet de créer **une copie locale complète**
du projet sur la machine, avec tout l’historique Git.  
À partir de cette copie, on peut :
- modifier le ``.urdf`` et les autres fichiers ;
- lancer les nœuds ROS2 et visualiser le robot dans RViz ;
- versionner ensuite nos propres modifications.

Qu’est‑ce que « cloner » un dépôt ?
-----------------------------------

Cloner un dépôt signifie :
- télécharger tous les fichiers présents sur GitHub ;
- récupérer également l’historique Git (commits, branches, etc.) ;
- créer un dossier local lié au dépôt distant, ce qui permet ensuite de faire des ``git pull`` (récupérer les mises à jour du prof) et des ``git push`` (envoyer nos modifications sur notre propre dépôt).

Après le clonage, on travaille toujours **dans le dossier cloné**, et non
directement sur GitHub.

Étapes pour cloner le projet du professeur
------------------------------------------

1. Récupérer l’URL du dépôt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Aller sur la page GitHub du projet du professeur.
- Cliquer sur le bouton **Code**.
- Copier l’URL du dépôt (HTTPS ou SSH, selon la configuration).

2. Se placer dans un dossier de travail
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dans un terminal :

.. code-block:: bash

   cd ~/ros2_ws/    # exemple de dossier de travail

3. Lancer la commande de clonage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone https://github.com/prof/nom-du-projet.git

ou, avec SSH :

.. code-block:: bash

   git clone git@github.com:prof/nom-du-projet.git

Git crée alors un dossier contenant tous les fichiers du projet :

.. code-block:: bash

   ls
   nom-du-projet/

4. Entrer dans le projet cloné
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd nom-du-projet
   git status

À partir de là, on peut modifier les fichiers, par exemple le ``.urdf``,
ou les fichiers de lancement ROS2.

Lien avec RViz et ROS2
----------------------

Une fois le dépôt cloné, il devient possible de :
- construire le workspace ROS2 si nécessaire (par exemple avec ``ros2_build``) ;
- sourcer l’environnement (``source install/setup.bash`` ou équivalent) ;
- utiliser les fichiers de lancement fournis pour démarrer le robot dans RViz.

Sans clonage initial, ces fichiers (``.urdf``, launch files, meshes, etc.)
ne seraient pas disponibles localement et il serait impossible de tester ou
modifier le robot.

Bonnes pratiques après le clonage
---------------------------------

- Ne pas modifier directement les fichiers sur GitHub : toujours passer par
  la copie locale clonée.
- Faire régulièrement :

  .. code-block:: bash

     git pull

  pour récupérer les éventuelles mises à jour du professeur.
- Utiliser des branches pour les grosses modifications (par exemple une
  refonte du ``.urdf`` ou de la structure du package).

