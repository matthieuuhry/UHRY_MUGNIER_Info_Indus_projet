Utiliser Git et GitHub dans le projet
=====================================

Objectifs
---------

- Versionner le code et la documentation.
- Travailler proprement sur des branches.
- Garder un historique clair des modifications.
- Corriger les erreurs (retours en arrière, suppression de fichiers suivis par erreur).

Initialisation du dépôt local
-----------------------------

Cloner le dépôt existant depuis GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone git@github.com:MonUser/MonDepot.git
   cd MonDepot

Vérifier l’état du dépôt
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git status

Cette commande permet de voir :
- la branche courante (souvent ``main``) ;
- les fichiers modifiés ;
- les fichiers non suivis (``Untracked files``).

Mettre à jour sa copie locale
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git checkout main
   git pull

Gestion des fichiers inutiles (.venv, .DS_Store, etc.)
-------------------------------------------------------

Problème rencontré
^^^^^^^^^^^^^^^^^^

Au départ, certains fichiers/dossiers n’auraient pas dû être versionnés :
- l’environnement virtuel Python ``.venv/`` ;
- des fichiers système comme ``.DS_Store`` sous macOS.

Ils apparaissaient dans ``git status`` comme modifiés ou suivis, ce qui polluait l’historique.

Ajouter des entrées dans .gitignore
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Exemple de contenu du fichier ``.gitignore`` :

.. code-block:: text

   .venv/
   __pycache__/
   .DS_Store

Cela empêche ces fichiers d’être ajoutés à l’avenir.

Retirer un dossier déjà suivi (ex. .venv)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si ``.venv`` avait déjà été ajouté au dépôt, il fallait le retirer de l’index tout en le conservant localement :

.. code-block:: bash

   git rm -r --cached .venv
   git commit -m "Retirer .venv du suivi Git"
   git push

Même principe pour d’autres fichiers indésirables (par exemple ``.DS_Store``) :

.. code-block:: bash

   git rm --cached .DS_Store
   git commit -m "Retirer .DS_Store du dépôt"
   git push

Cycle de travail quotidien
--------------------------

Modifier des fichiers
^^^^^^^^^^^^^^^^^^^^^

Après avoir travaillé (code, docs, config Sphinx), on peut vérifier les changements :

.. code-block:: bash

   git status
   git diff         # voir le détail des modifications

Préparer le commit
^^^^^^^^^^^^^^^^^^

Plusieurs possibilités :

- Ajouter tous les fichiers suivis :

  .. code-block:: bash

     git add .

- Ou ajouter explicitement certains fichiers :

  .. code-block:: bash

     git add docs/source/index.rst
     git add docs/source/venv.rst
     git add .gitignore

Créer un commit avec un message explicite :

.. code-block:: bash

   git commit -m "Documenter l'utilisation de .venv et nettoyer .gitignore"

Envoyer les changements sur GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git push

Si c’est la première fois sur une nouvelle branche :

.. code-block:: bash

   git push -u origin ma-branche

Travail avec des branches
-------------------------

Créer une branche de fonctionnalité
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git checkout main
   git pull
   git checkout -b feature/docs-venv

Travailler sur la branche
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # modifications…
   git status
   git add .
   git commit -m "Rédiger la page sur l'environnement virtuel"

Pousser la branche sur GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git push -u origin feature/docs-venv

Ensuite, la fusion vers ``main`` se fait via une *Pull Request* sur l’interface GitHub.

Corriger des erreurs de suivi
-----------------------------

Retirer un fichier ajouté par erreur
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si un fichier ne devrait pas être versionné :

.. code-block:: bash

   git rm --cached chemin/du/fichier
   echo "chemin/du/fichier" >> .gitignore
   git add .gitignore
   git commit -m "Arrêter de suivre fichier inutile"
   git push

Retirer un dossier entier (exemple : résidus d’un mauvais build)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git rm -r --cached docs/_build
   echo "docs/_build/" >> .gitignore
   git add .gitignore
   git commit -m "Ignorer les fichiers de build Sphinx"
   git push

Récupérer les dernières modifications de main
---------------------------------------------

Mettre à jour sa branche locale
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git checkout main
   git pull

Mettre à jour une branche de travail par rapport à main
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git checkout feature/docs-venv
   git merge main   # ou git rebase main, selon la politique de l'équipe

Explications des commandes Git de base
======================================

git commit
----------

Un *commit* enregistre un « instantané » de l’état des fichiers suivis à un moment donné.

- Avant le commit, on sélectionne les fichiers avec ``git add``.
- Le commit crée un point dans l’historique avec un message explicite.

Exemple :

.. code-block:: bash

   git add docs/source/venv.rst
   git commit -m "Expliquer l'utilisation de l'environnement virtuel .venv"

Ici :
- ``git add`` prépare le fichier pour le prochain commit.
- ``git commit`` enregistre ce changement dans l’historique local du dépôt.
- Le commit reste **local** tant qu’on n’a pas fait de ``git push``.

git push
--------

``git push`` envoie les commits locaux vers le dépôt distant (GitHub).

- Sans ``git push``, les autres collaborateurs ne voient pas les derniers commits.
- On pousse généralement sur la même branche (``main`` ou une branche de fonctionnalité).

Exemple :

.. code-block:: bash

   git push           # sur une branche déjà configurée
   # ou la première fois sur une nouvelle branche :
   git push -u origin feature/docs-venv

Ici :
- ``origin`` est le nom par défaut du dépôt distant.
- ``feature/docs-venv`` est le nom de la branche envoyée sur GitHub.
- L’option ``-u`` mémorise le lien entre la branche locale et la branche distante.

git pull
--------

``git pull`` récupère les derniers commits depuis GitHub et les fusionne dans la branche locale.

- À utiliser régulièrement avant de commencer à travailler.
- Permet de rester synchronisé avec les modifications des autres (ou celles faites depuis une autre machine).

Exemple :

.. code-block:: bash

   git checkout main
   git pull

Ici :
- ``git checkout main`` place la copie locale sur la branche ``main``.
- ``git pull`` récupère depuis GitHub les nouveaux commits de ``origin/main`` et les fusionne dans la ``main`` locale.
- Si des modifications locales entrent en conflit avec celles du distant, Git demande de résoudre les conflits avant de continuer.

Résumé du cycle
---------------

Pour une petite modification simple sur ``main`` :

.. code-block:: bash

   git pull                           # mettre à jour la branche
   # modifier des fichiers…
   git status                         # vérifier l'état
   git add fichier1 fichier2          # préparer les fichiers
   git commit -m "Décrire la modif"   # enregistrer localement
   git push                           # envoyer sur GitHub
