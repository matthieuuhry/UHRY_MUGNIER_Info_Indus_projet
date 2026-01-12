Utilisation de l’environnement virtuel (.venv)
==============================================

Motivations
-----------

Pour ce projet, l’utilisation d’un environnement virtuel Python ``.venv`` permet d’isoler
les dépendances (Sphinx, thèmes, extensions) du reste du système.  
Ainsi, les versions de paquets utilisées pour la documentation ne perturbent pas les
autres projets installés sur la machine, et inversement.

Principe général
----------------

Un environnement virtuel est un dossier qui contient :
- une copie locale de l’interpréteur Python ;
- un dossier de paquets indépendants du système (``site-packages``) ;
- des scripts dédiés (``pip``, ``sphinx-build``, etc.) liés à cet interpréteur.

Lorsque l’environnement est activé, les commandes ``python`` et ``pip`` pointent vers
cette installation locale plutôt que vers celle du système.

Mise en place dans ce projet
----------------------------

1. Création de l’environnement virtuel à la racine du dépôt :

   .. code-block:: bash

      python3 -m venv .venv

2. Activation de l’environnement avant d’installer les dépendances :

   .. code-block:: bash

      source .venv/bin/activate   # macOS / Linux
      # ou
      .venv\\Scripts\\activate    # Windows (PowerShell / CMD)

3. Installation des paquets nécessaires à la documentation :

   .. code-block:: bash

      pip install -r docs/requirements.txt
      # ou, à défaut de fichier requirements :
      pip install sphinx sphinx-rtd-theme

4. Désactivation de l’environnement une fois le travail terminé :

   .. code-block:: bash

      deactivate

Intégration avec Sphinx
-----------------------

Toutes les commandes Sphinx sont exécutées depuis l’environnement virtuel :

.. code-block:: bash

   source .venv/bin/activate
   sphinx-build -b html docs/source docs

Cela garantit que la version de Sphinx et du thème utilisée en local est la même que
celle déclarée dans ``requirements.txt``, ce qui facilite la reproductibilité sur
d’autres machines (par exemple pour les enseignants ou les correcteurs).

Bonnes pratiques
----------------

- Ne pas versionner le dossier ``.venv/`` dans Git (il est listé dans ``.gitignore``) ;
- Documenter les dépendances dans un fichier ``requirements.txt`` ;
- Toujours activer ``.venv`` avant de générer la documentation ou d’installer de
  nouveaux paquets ;
- Recréer l’environnement en cas de problème :

  .. code-block:: bash

     rm -rf .venv
     python3 -m venv .venv
     source .venv/bin/activate
     pip install -r docs/requirements.txt


