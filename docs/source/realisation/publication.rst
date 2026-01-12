Publier la documentation Sphinx en ligne avec GitHub Pages
==========================================================

Objectif
--------

L’objectif est de publier la documentation générée par Sphinx sur le web, en
utilisant GitHub Pages, à partir des fichiers HTML (``index.html`` et le
dossier ``_static``) placés dans le répertoire ``docs/`` du dépôt.

Organisation des dossiers
-------------------------

Dans ce projet, la documentation est organisée ainsi :

- ``docs/source/`` : fichiers source Sphinx (``.rst``) ;
- ``docs/``        : **sortie HTML** destinée à GitHub Pages, contenant
  directement ``index.html`` et le dossier ``_static/``.

La génération des pages HTML se fait donc de la manière suivante :

.. code-block:: bash

   sphinx-build -b html docs/source docs

Comprendre GitHub Pages
-----------------------

GitHub Pages est un service d’hébergement statique lié au dépôt Git.  
Il peut servir :

- soit la racine du dépôt ;
- soit un sous-dossier comme ``/docs`` sur une branche donnée (ici ``main``).

Dans ce projet, on configure GitHub Pages pour servir le contenu du dossier
``docs/`` de la branche ``main``.  
Tout ce qui se trouve dans ``docs/`` (dont ``index.html``) est alors accessible
via une URL publique du type :

.. code-block:: text

   https://<utilisateur>.github.io/<nom-du-depot>/

Étapes de publication
---------------------

1. Générer la documentation HTML dans docs/
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Depuis la racine du dépôt :

.. code-block:: bash

   sphinx-build -b html docs/source docs

Cette commande :

- lit les sources Sphinx dans ``docs/source`` ;
- produit les fichiers HTML, CSS et JS directement dans ``docs/``
  (``index.html``, ``_static/``, ``_sources/``, etc.).

2. Empêcher Jekyll d’interférer : .nojekyll
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Problème rencontré
""""""""""""""""""

Lors des premiers essais, le thème Sphinx ne s’affichait pas correctement en
ligne : la page apparaissait avec un style très basique, comme si les fichiers
CSS n’étaient pas trouvés.

En inspectant la console du navigateur, on voyait des erreurs 404 sur des
fichiers comme :

- ``_static/pygments.css`` ;
- ``_static/css/theme.css`` ;
- ``_static/js/theme.js``.

Cela venait du fait que GitHub Pages utilise par défaut Jekyll, qui peut
ignorer ou filtrer certains dossiers, notamment ceux commençant par ``_``
(comme ``_static``).

Solution : ajouter un fichier ``.nojekyll`` dans ``docs/`` pour indiquer à
GitHub de **servir les fichiers tels quels** sans passer par Jekyll.

Depuis la racine du dépôt :

.. code-block:: bash

   touch docs/.nojekyll
   git add docs/.nojekyll
   git commit -m "Ajouter .nojekyll pour GitHub Pages"

3. Configurer GitHub Pages dans l’interface GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sur GitHub, dans les paramètres du dépôt :

- aller dans **Settings → Pages** ;
- choisir **Deploy from a branch** ;
- sélectionner :

  - branche : ``main`` ;
  - dossier : ``/docs``.

Après validation, GitHub va servir directement le contenu du dossier ``docs/``
de la branche ``main``.

4. Pousser la documentation à chaque modification
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dès que la documentation est modifiée (nouveaux fichiers ``.rst``, corrections,
etc.), la procédure est la suivante :

.. code-block:: bash

   # 1. Regénérer la doc dans docs/
   sphinx-build -b html docs/source docs

   # 2. Vérifier que docs/ contient bien index.html et _static/
   ls docs

   # 3. Ajouter et committer les changements
   git add docs
   git commit -m "Mettre à jour la documentation Sphinx"

   # 4. Envoyer sur GitHub
   git push

GitHub Pages détecte alors le nouveau contenu dans ``main/docs`` et met le site
à jour après quelques dizaines de secondes.

Validation locale avant publication
-----------------------------------

Pour vérifier que la doc est correcte avant de la publier, il est possible de
la tester en local :

.. code-block:: bash

   # Se placer dans le répertoire docs
   cd docs

   # Lancer un petit serveur HTTP local
   python3 -m http.server 8000

Puis ouvrir dans le navigateur :

.. code-block:: text

   http://localhost:8000/

Si le thème s’affiche correctement en local mais pas en ligne, il faut
vérifier :

- la présence du fichier ``.nojekyll`` dans ``docs/`` ;
- la configuration GitHub Pages (branch = ``main``, dossier = ``/docs``) ;
- que les fichiers générés (``index.html``, ``_static/``) ont bien été ajoutés
  et poussés (``git add docs`` puis ``git commit`` et ``git push``).

Synthèse des commandes utilisées
--------------------------------

Depuis la racine du projet, la séquence typique est :

.. code-block:: bash

   # Générer la doc dans docs/
   sphinx-build -b html docs/source docs

   # S’assurer que .nojekyll est présent dans docs/
   touch docs/.nojekyll      # une seule fois si le fichier n'existe pas
   git add docs/.nojekyll    # une seule fois aussi

   # Suivre tous les changements de documentation
   git add docs

   # Créer un commit
   git commit -m "Mettre à jour la documentation Sphinx"

   # Publier sur GitHub
   git push

Après ces étapes, GitHub Pages reconstruit le site et la documentation est
disponible en ligne avec le thème Sphinx correctement chargé.

