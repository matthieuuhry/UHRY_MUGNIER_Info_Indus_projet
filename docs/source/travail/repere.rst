Génération des fichiers .dae pour RViz
======================================

Objectif
--------

Les fichiers ``.dae`` (format Collada) décrivent la géométrie 3D des pièces
sous forme de maillages.  
Ils sont référencés dans le fichier URDF pour que RViz puisse afficher le robot
de manière réaliste (au‑delà de simples primitives géométriques).

Dans ce projet, les modèles 3D d’origine ont été créés dans Creo.  
Pour les exploiter dans RViz, nous avons suivi deux grandes étapes :

1. Exporter les pièces depuis Creo au format neutre STEP (``.step`` / ``.stp``) ;
2. Convertir ces fichiers STEP en fichiers DAE (Collada) avec FreeCAD.

Étape 1 : export depuis Creo en STEP
------------------------------------

Préparer les repères
^^^^^^^^^^^^^^^^^^^^

Avant l’export, il est important de vérifier l’orientation de chaque pièce
dans Creo :

- Choisir un repère cohérent avec le monde ROS (par exemple ``Z`` vers le haut,
  ``X`` vers l’avant, ``Y`` vers la gauche) ;
- Vérifier que la pièce est correctement positionnée par rapport à ce repère
  (origine logique, axes alignés).

Une bonne définition des repères évite d’avoir des pièces tournées ou décalées
dans RViz une fois intégrées dans le URDF.

Procédure d’export STEP
^^^^^^^^^^^^^^^^^^^^^^^

Pour chaque pièce (ou sous‑ensemble) modélisée dans Creo :

1. Ouvrir la pièce ou l’assemblage.
2. Vérifier le repère actif (orientation des axes).
3. Utiliser la commande d’export :

   - Menu d’export (type « Enregistrer sous » / « Exporter ») ;
   - Choisir le format **STEP** (``.step`` ou ``.stp``).

4. Donner un nom explicite au fichier.

Les fichiers STEP ainsi obtenus sont des représentations géométriques neutres,
faciles à réimporter dans d’autres logiciels de CAO ou de conversion.

Étape 2 : conversion en DAE avec FreeCAD
----------------------------------------

Pourquoi passer par FreeCAD ?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

RViz attend un maillage (mesh) dans les formats supportés (STL, Collada DAE,
etc.).  
FreeCAD sait :

- importer des fichiers STEP ;
- convertir la géométrie solide en maillage ;
- exporter ce maillage en **Collada (.dae)**, format très utilisé pour les
  visualisations 3D.

Import des fichiers STEP
^^^^^^^^^^^^^^^^^^^^^^^^

Pour chaque pièce :

1. Lancer FreeCAD.
2. Créer un nouveau document ou utiliser un document vide.
3. Importer le fichier STEP :

   - Menu **Fichier → Importer** ;
   - Sélectionner le fichier ``.step`` / ``.stp`` exporté depuis Creo.

4. Vérifier que la pièce apparaît correctement dans la vue 3D
   (orientation, taille, cohérence générale).

Conversion en maillage et export DAE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dans FreeCAD, la géométrie STEP doit être convertie en maillage avant
l’export Collada :

1. Sélectionner l’objet importé dans l’arborescence.
2. Utiliser l’outil de conversion en maillage :

   - Menu **Fichier → Exporter** ;
   - Choisir le format **Collada (.dae)** ;
   - Nommer le fichier de manière cohérente, par exemple :

     - ``base.dae``
     - ``link1.dae``

Les fichiers ``.dae`` générés sont ensuite copiés dans un dossier dédié du
package ROS2, typiquement :

.. code-block:: text

   mon_package_description/
     meshes/
       base.dae
       bras_superieur.dae
       pince.dae

Intégration dans le URDF et RViz
--------------------------------

Dans le fichier URDF, chaque lien du robot peut référencer le fichier DAE
correspondant, par exemple :

.. code-block:: xml

   <mesh filename="package://mon_package_description/meshes/base.dae"/>

Lors du lancement du robot dans RViz (via les fichiers de lancement ROS2), RViz :

- charge le URDF ;
- retrouve les chemins des maillages ``.dae`` ;
- affiche le robot avec les formes et orientations définies dans Creo.

Points d’attention et problèmes rencontrés
------------------------------------------

- **Orientation des pièces** : si les repères ne sont pas cohérents dans Creo,
  les pièces peuvent apparaître retournées ou inclinées dans RViz.  
  D’où l’importance de choisir les bons repères avant l’export.
- **Taille / échelle** : vérifier l’unité utilisée (mm, m) dans Creo et
  FreeCAD pour éviter un robot trop grand ou trop petit dans RViz.
- **Emplacement des fichiers** : les chemins utilisés dans le URDF doivent
  correspondre exactement à l’emplacement réel des ``.dae`` dans le package.

En suivant cette chaîne Creo → STEP → FreeCAD → DAE, il devient possible
d’obtenir une représentation fidèle du robot dans RViz à partir des modèles
CAO d’origine.

