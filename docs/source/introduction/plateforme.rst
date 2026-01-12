Présentation de la plateforme mécanique
========================================

La plateforme pantographe illustrée ci-dessous est conçue pour réaliser des trajectoires précises sur une surface plane, combinant simplicité mécanique et contrôle électronique.

.. figure:: img/real_system_photo.png
   :align: center
   :width: 60%

Description du matériel de la plateforme pantographe
*****************************************************

* Composants principaux :

1. **Bras robotiques imprimés en 3D :**

   - Les bras articulés sont fabriqués à l’aide de l’impression 3D.
   - Ils sont connectés via des liaisons de révolution.

2. **Servomoteurs Dynamixel AX-12A :**

   - Deux servomoteurs **Dynamixel AX-12A** assurent le mouvement des bras robotiques.
   - Ils permettent un contrôle précis des positions pour suivre les trajectoires programmées.

3. **Châssis en aluminium :**

   - La structure repose sur deux plaques d’aluminium.
   - La plaque supérieure supporte les bras et pivots, tandis que la plaque inférieure accueille les servomoteurs.

4. **Support central pour outil traceur :**

   - Un emplacement central permet de fixer un outil, tel qu’un stylo, pour dessiner ou suivre des trajectoires sur une feuille.

Cette plateforme est idéale pour des applications de traçage ou pour l’étude de systèmes mécaniques à deux degrés de liberté.

Description mécanique
**********************

.. figure:: img/meca.png
   :align: center
   :width: 60%


Génération de la description URDF du pantographe
*************************************************

Nous allons utiliser le modèle 3D au format step conçu et réalisé par M. Olivier PICCIN pour générer la description URDF du pantographe.

.. figure:: img/pantograph.png
   :align: center
   :width: 60%


Il s'agit d'un assemblage comportant 5 pièces principales :

  #. BATI_ASM: le bâti de la strucuture du pantographe avec les 2 moteurs pas à pas. Dans l'URDF nous allons le nommer ``base``.
  #. LINK1_ASM: le bras gauche du pantographe. Dans l'URDF nous allons le nommer ``link1``.
  #. LINK2_ASM: l'avant bras gauche du pantographe. Dans l'URDF nous allons le nommer ``link2``.
  #. LINK3_ASM: l'avant bras droit du pantographe. Dans l'URDF nous allons le nommer ``link3``.
  #. LINK4_ASM: le bras droit du pantographe. Dans l'URDF nous allons le nommer ``link4``.

Du point de vue URDF, nous aurons aussi besoin de définir l'outil de travail: le crayon et donc de le référencer par rapport à son support qui se trouve être sur l'avant bras gauche du pentographe (``link2``).

Dans un fichier URDF, les modèles 3D sont utilisés à partir de fichiers collada (extension .dae).
Il va donc falloir convertir notre modèle 3D au format step en un ensemble de 5 modèles 3D au format collada.


Extraction des modèles 3D en step
---------------------------------

Pour extraire les modèles 3D au format step, nous pouvons utiliser le logiciel freecad.


Conversion des modèles 3D en collada
------------------------------------

Pour convertir les modèles 3D en collada (dae), nous pouvons utiliser le logiciel freecad.

Création du fichier URDF
------------------------

Dans un fichier URDF les modèles 3D sont référencés par des balises ``<mesh>``.