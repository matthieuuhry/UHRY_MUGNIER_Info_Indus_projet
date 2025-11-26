Installer Ubuntu Jammy Jellyfish sur Mac avec UTM
=================================================

Ce guide explique pas à pas comment installer **Ubuntu 22.04 (Jammy Jellyfish)** sur un Mac en utilisant l’outil de virtualisation UTM.

.. warning ::
    Cette machine virtuelle ne sera pas très puissante étant donné que l'on simule un processeur intel, elle permet cependant de travailler sur votre ordinateur perso depuis chez vous.

Prérequis
---------

- Un Mac sous macOS (Apple Silicon ou Intel)
- 20 Go d’espace disque disponible
- Connexion internet performante

Téléchargement des outils
-------------------------

**a. Télécharger UTM**

- Site officiel : `https://mac.getutm.app/ <https://mac.getutm.app/>`_
- Télécharge le fichier `.dmg` et installe UTM via le dossier "Applications".

**b. Télécharger Ubuntu ARM**

- Va sur : `https://releases.ubuntu.com/jammy/ <https://releases.ubuntu.com/jammy/>`_
- Télécharge l’image **ubuntu-22.04.4-desktop-amd64.iso** (pour Intel) car l'image desktop qui permet donc d'avoir un bureau classique et pas uniquement un terminal n'existe pas en ARM64 (pour MAC).

Création de la machine virtuelle
--------------------------------

#. **Lance UTM** et clique sur le bouton "+" pour créer une nouvelle VM.
#. Choisis **Emulate** pour simuler un processeur Intel et pouvoir lancer une image amd64.
#. Sélectionne **Linux**.
#. Configure les ressources :
   - **Machine** : Choisis Intel ICH9 based PC. 
   - **RAM** : 4 à 8 Go selon la mémoire disponible.
   - **CPU** : Laisse le paramètre par défaut ou adapte.
#. Ajoute ton ISO Ubuntu téléchargé dans le champ "CD/DVD".
#. Choisis au minimum 20 GB de stockage.
#. Définis un chemin pour le dossier partagé si tu le souhaites.
#. Donne un nom à ta VM (ex : `Ubuntu 22.04`).
#. Sauvegarde et termine la configuration.

Installation d'Ubuntu
---------------------

#. Démarre la VM (bouton "Play").
#. À l’écran d’accueil, sélectionne *Try or Install Ubuntu*.
#. Suis l’assistant d’installation :
   - Choisis la langue d’installation.
   - Configure ton clavier.
   - Paramètre le fuseau horaire.
   - Renseigne le nom d’utilisateur et le mot de passe.
   - Choisis "Installation normale" ou minimale.
   - (Optionnel) Installe les logiciels tiers recommandés.
#. Patiente jusqu’à la fin de l’installation (environ dix minutes).

Premiers pas et conseils
------------------------

- Éjecte le disque ISO si besoin avant de redémarrer la VM (option via UTM).
- Connecte-toi à Ubuntu.
- Mets à jour le système :
  
  .. code-block:: bash

     sudo apt update && sudo apt upgrade

- Pour partager des fichiers Mac ↔ Ubuntu, utilise la fonction "Shared Directory" de UTM (voir documentation officielle : `https://docs.getutm.app/ <https://docs.getutm.app/>`_).
- Tu peux maintenant commencer l'installation de ROS2.'

Astuces supplémentaires
-----------------------

- Pour améliorer la résolution de l'affichage, ajuste les paramètres d’affichage dans Ubuntu ou dans UTM.
- Le réseau "Bridged" permet un accès Internet fiable dans la VM, mais le mode par défaut fonctionne généralement.

Références et documentation
---------------------------

- Documentation UTM (fr/en): `https://docs.getutm.app/ <https://docs.getutm.app/>`_
- Téléchargement Ubuntu: `https://ubuntu.com/download/desktop <https://ubuntu.com/download/desktop>`_
- [Guide officiel Ubuntu](https://help.ubuntu.com/lts/installation-guide/)

---


