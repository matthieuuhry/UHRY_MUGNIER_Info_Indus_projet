Utiliser GitHub dans le projet
==============================

Objectifs
---------

- Gérer le code source en équipe.
- Suivre l'historique et les versions.
- Publier automatiquement la documentation et/ou le site en ligne.

Workflow Git : branches et pull requests
----------------------------------------

Principe général
^^^^^^^^^^^^^^^^

Le développement se fait sur des branches dédiées, qui sont intégrées à la branche principale (`main`) via des pull requests.

Créer une branche de travail
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Récupérer la dernière version de main
   git checkout main
   git pull

   # Créer une branche
   git checkout -b feature/ma-fonctionnalite

Cycle de travail
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Travailler, puis enregistrer les modifications
   git status
   git add .
   git commit -m "Décrit clairement la modification"

   # Pousser la branche sur GitHub
   git push -u origin feature/ma-fonctionnalite

Ouvrir une pull request
^^^^^^^^^^^^^^^^^^^^^^^

- Aller sur le dépôt GitHub du projet.
- Créer une *Pull Request* de `feature/ma-fonctionnalite` vers `main`.
- Décrire la modification (objectif, impacts, tests effectués).
- Demander une revue, corriger si besoin, puis fusionner la PR.
- Supprimer la branche une fois fusionnée.

GitHub Actions : intégration et publication automatiques
--------------------------------------------------------

Principe
^^^^^^^^

GitHub Actions exécute automatiquement des jobs (tests, build, déploiement) à chaque push ou pull request sur certaines branches.

Exemple de workflow de build de la doc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fichier : ``.github/workflows/docs.yml``

.. code-block:: yaml

   name: Build docs

   on:
     push:
       branches: [ "main" ]
     pull_request:
       branches: [ "main" ]

   jobs:
     build:
       runs-on: ubuntu-latest

       steps:
         - uses: actions/checkout@v4

         - name: Setup Python
           uses: actions/setup-python@v5
           with:
             python-version: "3.11"

         - name: Install dependencies
           run: |
             pip install -r docs/requirements.txt

         - name: Build Sphinx HTML
           working-directory: docs
           run: |
             make html

Ce workflow vérifie que la doc se construit bien à chaque push / PR sur `main`.

Publication en ligne : GitHub Pages
-----------------------------------

Principe
^^^^^^^^

Le résultat HTML généré par Sphinx est publié automatiquement sur GitHub Pages, ce qui fournit une URL publique pour la documentation.

Workflow de déploiement
^^^^^^^^^^^^^^^^^^^^^^^

Fichier : ``.github/workflows/deploy-docs.yml``

.. code-block:: yaml

   name: Deploy docs to GitHub Pages

   on:
     push:
       branches: [ "main" ]

   permissions:
     contents: read
     pages: write
     id-token: write

   concurrency:
     group: "pages"
     cancel-in-progress: true

   jobs:
     build:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v4

         - name: Setup Python
           uses: actions/setup-python@v5
           with:
             python-version: "3.11"

         - name: Install dependencies
           run: |
             pip install -r docs/requirements.txt

         - name: Build Sphinx HTML
           working-directory: docs
           run: |
             make html

         - name: Upload artifact
           uses: actions/upload-pages-artifact@v3
           with:
             path: docs/_build/html

     deploy:
       needs: build
       runs-on: ubuntu-latest
       environment:
         name: github-pages
         url: ${{ steps.deployment.outputs.page_url }}
       steps:
         - name: Deploy to GitHub Pages
           id: deployment
           uses: actions/deploy-pages@v4

Configuration GitHub Pages
^^^^^^^^^^^^^^^^^^^^^^^^^^

- Dans les paramètres du dépôt, activer *Pages*.
- Choisir *Deploy from a branch* comme source.
- L’URL publique de la documentation est indiquée une fois le premier déploiement effectué.

Bonnes pratiques d’équipe
-------------------------

- Toujours partir de `main` à jour pour créer une nouvelle branche.
- Une fonctionnalité = une branche = une pull request.
- Vérifier que les workflows (tests, build docs) passent avant de fusionner.
- Mettre à jour la documentation en même temps que le code.

