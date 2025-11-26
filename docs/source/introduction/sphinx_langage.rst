Le langage Sphinx
=================

Sphinx est un générateur de documentation puissant, principalement utilisé dans l’écosystème Python et pour tout projet technique nécessitant une documentation structurée, lisible sur le web ou au format PDF.

Qu’est-ce que Sphinx ?
----------------------

Sphinx est un outil open source permettant de transformer des fichiers texte écrits en reStructuredText (ou Markdown, avec extensions) en pages HTML, PDF, LaTeX ou autres formats. Il est particulièrement populaire dans le monde du logiciel libre, pour la documentation de bibliothèques, modules ou projets complexes.

Fonctionnement
--------------

- **Entrée :** des fichiers `.rst` (reStructuredText, syntaxe textuelle proche du Markdown mais plus puissante) organisés dans un dossier, plus un fichier de configuration `conf.py`.
- **Processus :** tu organises ta documentation avec des “toctree” pour structurer la navigation, puis tu lances Sphinx pour générer automatiquement une arborescence HTML, PDF, etc.
- **Sortie :** un site web statique et lisible, que tu peux héberger, partager ou versionner.

Principales fonctionnalités
----------------------------

- Arbres de navigation automatiques (toctree)
- Possibilité d’écrire en français ou dans toute autre langue
- Index et tables des matières générés dynamiquement
- Support de la documentation API avec extraction automatique des docstrings (pour les projets Python)
- Thèmes personnalisés et extension facile avec de nombreux plugins (intégration de diagrammes, mathématiques, code exécuté, etc.)
- Export multi-format (HTML, PDF/LaTeX, EPUB…)

Pourquoi utiliser Sphinx ?
--------------------------

- **Standard de facto en Python** (utilisé pour Python, NumPy, ROS, etc.)
- Versionnage et publication aisés (intégration avec GitHub Pages ou ReadTheDocs)
- Gestion avancée des références croisées (indices, liens internes)
- Adapté aussi pour les rapports de projet, guides techniques, TP/TD et même des livres entiers


Exemple d'organisation du sommaire :
------------------------------------

.. code-block:: rst

   .. toctree::
      :maxdepth: 2
      :caption: Documentation du projet

      introduction
      realisation
      conclusion


Exemple de syntaxe reStructuredText
-----------------------------------

Un titre :

.. code-block:: rst

   Titre principal
   ===============

   Sous-titre
   ----------

Une liste :

.. code-block:: rst

   - Premier point
   - Second point

Une note :

.. note::

   Ceci est une note informative mise en avant par Sphinx.

Un lien :

`Documentation Sphinx <https://www.sphinx-doc.org/fr/master/>`_

Voir plus d'exemples :  
`Syntaxe reStructuredText — Quick Reference <https://docutils.sourceforge.io/docs/user/rst/quickref.html>`_

Extensions populaires
---------------------

- `sphinx_rtd_theme <https://sphinx-rtd-theme.readthedocs.io/en/stable/>`_ : thème ReadTheDocs
- `sphinx.ext.autodoc <https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html>`_ : extraction de docs Python
- `sphinx.ext.napoleon <https://www.sphinx-doc.org/en/master/usage/extensions/napoleon.html>`_ : docstrings Google/Numpy

Ressources complémentaires
--------------------------

- `Blog : Introduction à Sphinx sur FLOZz.fr <https://blog.flozz.fr/2020/09/07/introduction-a-sphinx-un-outil-de-documentation-puissant/>`_
- `Guide CNRS : Documenter son projet avec Sphinx <https://perso.liris.cnrs.fr/francoise.conil/documenter-son-projet-avec-sphinx/>`_
- `Bases de la syntaxe Sphinx — UGA <https://inspe-sciedu.gricad-pages.univ-grenoble-alpes.fr/reflexpro/syntaxe_sphinx.html>`_

En résumé
---------

Sphinx est une solution reconnue pour produire une documentation technique, collaborative, et maintenable, et il s’adapte à de nombreux besoins modernes[1][5][8].

