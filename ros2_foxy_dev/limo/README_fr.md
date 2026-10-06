# Configuration du robot LIMO : Ubuntu 20.04 et ROS 2 Foxy

Ce tutoriel installe les dépendances **directement sur le robot physique**.
Il prend en charge Ubuntu 20.04 sur ARM64 et x86_64. Docker n'est pas nécessaire.
L'installation comprend ROS 2 Foxy, les dépendances du châssis et des capteurs,
la localisation, la navigation, le SLAM et les outils RViz facultatifs. **Gazebo est exclu.**

Exécutez vous-même les commandes ci-dessous sur le robot, dans un terminal local
ou par SSH. L'installation et le fonctionnement du matériel n'ont pas été testés
sur le robot lors de la préparation de ces fichiers ; aucune installation ni
compilation n'a été effectuée à cette occasion.

## 0. Informations sur le robot

`NAME`:     jetson
`PASSWORD`: jetson

## 1. Vérifier le robot et copier le projet

Vérifiez le système d'exploitation et l'architecture :

```bash
cat /etc/os-release
dpkg --print-architecture
```

Le système doit être **Ubuntu 20.04** et l'architecture doit être `arm64` ou
`amd64`. L'installateur s'arrête sur les autres systèmes. Foxy est conservé pour
correspondre à ce workspace ; il s'agit d'une ancienne distribution ROS qui
n'est plus prise en charge.

Copiez le projet complet sur le robot, par exemple dans :

```text
~/Limo/
  workspace/
    src/
  ros2_foxy_dev/
    limo/
```

Utilisez le compte Linux habituel qui exécutera les programmes du robot. Ne lancez
pas les scripts de configuration en tant que root ; ils demandent `sudo` lorsque
des modifications du système sont nécessaires.

```bash
cd ~/Limo/ros2_foxy_dev/limo
```

## 2. Installer les dépendances système

```bash
bash install-deps.sh
```

Ce script installe les paquets de `apt-packages.txt`, configure le dépôt apt
de ROS 2 et installe, avec rosdep, les dépendances supplémentaires déclarées
dans les manifestes des paquets du robot physique. Un accès à Internet et les
droits sudo sont nécessaires. Il ne met pas à niveau le système d'exploitation
et n'installe pas Docker.

Les composants inclus sont :

- La base de ROS 2 Foxy, la prise en charge des fichiers launch/XML, la génération
  des messages et les outils de compilation.
- Les dépendances du pilote du châssis LIMO, TF et la localisation du robot/EKF.
- Les dépendances du pilote YDLidar et les outils pour compiler le SDK C++ local.
- Les dépendances de la caméra Orbbec/Astra/Dabai : OpenCV, Eigen, USB, libuvc,
  glog/gflags, image transport, camera info manager, cv_bridge et les outils TF.
- Nav2, SLAM Toolbox, Cartographer, teleop, twist mux et les outils RViz facultatifs.
- `rqt_image_view` pour afficher les topics d'images de la caméra.
- NumPy, OpenCV, SciPy et YAML pour le code Python disponible.

Le manifeste original de `custom_start` déclare des dépendances pour le simulateur
et pour le robot. L'utilitaire crée des copies temporaires des manifestes pour
le robot uniquement, en supprimant les dépendances à `limo_car` et à Gazebo avant
d'exécuter rosdep. Les fichiers originaux restent inchangés. Une préférence apt
temporaire bloque l'installation de nouveaux paquets Gazebo, y compris les
dépendances transitives. Les installations existantes de Gazebo ne sont pas
supprimées. Si une future dépendance exige Gazebo, l'installation échoue au lieu
de l'ajouter.

Le [manifeste actuel de Nav2 bringup pour Foxy](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_bringup/bringup/package.xml)
ne déclare pas Gazebo comme dépendance.

## 3. Configurer les permissions du matériel

```bash
bash setup-devices.sh
```

Ce script ajoute votre utilisateur aux groupes `dialout` et `video`, installe
les règles udev fournies et les recharge. **Déconnectez-vous puis reconnectez-vous**
pour appliquer l'appartenance aux groupes. Si vous utilisez SSH, fermez la
connexion puis reconnectez-vous. Rebranchez les capteurs USB si nécessaire.

Vérifiez les périphériques :

```bash
id
ls -l /dev/ttyTHS1
ls -l /dev/ydlidar
lsusb
```

Le port par défaut du châssis est `/dev/ttyTHS1`. Le launch du lidar utilise
`/dev/ydlidar`, créé par les règles pour les identifiants d'adaptateur utilisés
par le pilote du workspace. Si l'adaptateur possède un autre identifiant USB,
modifiez `99-limo-hardware.rules` pour qu'il corresponde et relancez
`setup-devices.sh`.

Si plusieurs adaptateurs partagent ces identifiants, identifiez le lidar par
son numéro de série et affinez la règle pour que `/dev/ydlidar` pointe vers
le bon périphérique. L'accès USB à la caméra est accordé au groupe `video`
pour l'identifiant de fabricant Orbbec `2bc5`.

## 4. Restaurer le runtime de la caméra

Le pilote Astra fourni sous forme de code source nécessite le **runtime Orbbec
OpenNI2 du fabricant**. Les fichiers binaires requis sont absents de cette copie
du dépôt. La procédure ci-dessous restaure le runtime ARM64 depuis le
[dépôt officiel de la caméra ROS 2 d'Orbbec](https://github.com/orbbec/ros2_astra_camera/tree/master/astra_camera/openni2_redist/arm64).
Vous pouvez également obtenir un runtime compatible depuis le SDK du fabricant
ou une installation du robot dont le bon fonctionnement est connu.

### Vérifier une installation existante

Sur le robot, recherchez une bibliothèque OpenNI2 et des pilotes de caméra
déjà installés :

```bash
find /home/jetson /opt /usr/local /usr/lib \
  -name 'libOpenNI2.so*' 2>/dev/null
find /usr/lib /usr/local /opt /home/jetson \
  -path '*/OpenNI2/Drivers/*' -type f 2>/dev/null
```

Remplacez `/home/jetson` si le compte du robot possède un autre répertoire
personnel. Trouver uniquement `/usr/lib/libOpenNI2.so` ne suffit pas. Lors de
la configuration de la Jetson Nano, le répertoire des pilotes système contenait
`libPSLink.so.0`, `libOniFile.so.0`, `libDummyDevice.so.0` et `libPS1080.so.0`,
mais pas `liborbbec.so`. Cette installation ne fournissait pas le pilote Orbbec
nécessaire à cette procédure.

### Télécharger et copier le runtime ARM64

Vérifiez l'architecture du robot :

```bash
dpkg --print-architecture
```

**Continuez avec les commandes suivantes uniquement si le résultat est `arm64`.**
Téléchargez le dépôt officiel dans un répertoire séparé, sans `sudo` :

```bash
git clone --depth 1 --branch master \
  https://github.com/orbbec/ros2_astra_camera.git \
  ~/orbbec-astra-runtime
```

Si `~/orbbec-astra-runtime` contient déjà cette copie du dépôt, réutilisez-la
plutôt que de cloner à nouveau dans le même répertoire. Les commandes ci-dessous
supposent que le projet se trouve dans `~/Limo` ; adaptez ce chemin si nécessaire.

```bash
cd ~/Limo/workspace/src/ros2_astra_camera/astra_camera
```

Si la commande `cd` réussit, copiez tout le répertoire du runtime, y compris
ses fichiers de configuration :

```bash
mkdir -p openni2_redist/arm64
cp -a ~/orbbec-astra-runtime/astra_camera/openni2_redist/arm64/. \
  openni2_redist/arm64/
```

Le runtime doit être placé dans le paquet source original, dans le répertoire
correspondant au processeur du robot :

| Architecture du robot | Répertoire source requis |
| --- | --- |
| `arm64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/arm64/` |
| `amd64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/x64/` |

Le répertoire ARM64 doit contenir :

```text
openni2_redist/arm64/
  libOpenNI2.so
  OpenNI.ini
  OpenNI2/
    Drivers/
      liborbbec.so
      libOniFile.so
      orbbec.ini
      OniFile.ini
```

Pour `amd64`, obtenez le runtime `x64` correspondant au lieu de copier les
fichiers ARM64. Utilisez des binaires compatibles avec l'architecture du robot
et Ubuntu 20.04. La bibliothèque OpenNI2 générique d'Ubuntu ne remplace pas
le pilote Orbbec fourni avec le runtime attendu par ce paquet.

### Vérifier le runtime avant la compilation

Depuis le même répertoire `astra_camera`, exécutez :

```bash
file openni2_redist/arm64/libOpenNI2.so \
  openni2_redist/arm64/OpenNI2/Drivers/liborbbec.so
ldd openni2_redist/arm64/libOpenNI2.so
ldd openni2_redist/arm64/OpenNI2/Drivers/liborbbec.so
```

La commande `file` doit identifier les deux bibliothèques comme des binaires
ELF ARM aarch64. La sortie de `ldd` ne doit contenir ni `not found` ni erreur
de version. Résolvez les dépendances manquantes avant la compilation.

La vérification avec `ldd` de `liborbbec.so` a réussi sur la Jetson Nano lors
de cette configuration : toutes les dépendances affichées ont été résolues
avec les bibliothèques AArch64 du système. Cela confirme que les dépendances
de la bibliothèque du pilote sont disponibles ; la détection de la caméra
et la diffusion des images doivent encore être vérifiées après le lancement
de ROS.

Revenez au répertoire de configuration avant de suivre l'étape 5 :

```bash
cd ~/Limo/ros2_foxy_dev/limo
```

Si les fichiers de la caméra ne sont pas encore disponibles, utilisez la
compilation sans caméra à l'étape suivante. Le châssis, le lidar et la
localisation peuvent être préparés indépendamment.

## 5. Compiler le workspace du robot physique pour la première fois

Avant la compilation, supprimez une inclusion JSON inutilisée du code source
original de la caméra pour éviter une erreur de fichier d'en-tête manquant.
Exécutez cette commande depuis `ros2_foxy_dev/limo` :

```bash
sed -i '/^#include <nlohmann\/json\.hpp>$/d' \
  ../../workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_info.cpp
```

Vous pouvez répéter cette commande sans risque si l'inclusion a déjà été supprimée.

Une fois le runtime de la caméra restauré :

```bash
bash build-robot.sh
```

Ou, pour compiler le châssis, le lidar et la localisation sans la caméra :

```bash
bash build-robot.sh --without-camera
```

Le script vérifie d'abord les fichiers requis. Ensuite, il :

1. Vérifie les paquets du robot physique dans le répertoire original `workspace/src`.
2. Active ces paquets en supprimant leurs éventuels fichiers `COLCON_IGNORE`.
3. Compile le SDK C++ YDLidar dans `workspace/build/ydlidar_sdk` et l'installe
   dans `workspace/install/ydlidar_sdk`.
4. Compile les paquets ROS sélectionnés directement depuis leurs répertoires
   sources originaux avec `colcon build --symlink-install`, en utilisant ce SDK.
5. Configure l'environnement ROS et les alias du robot dans votre `~/.bashrc`.

Le SDK est compilé avant le pilote du lidar, car le manifeste original du
pilote ne déclare pas cette dépendance de compilation. Le SDK est installé
dans un répertoire propre au projet, sans remplacer une installation système
existante.

Les résultats de compilation utilisent les répertoires standard
`workspace/build`, `workspace/install` et `workspace/log`. Le script sélectionne
uniquement les paquets du robot physique ; les paquets du simulateur sont
exclus de la découverte. Les fichiers sources et les manifestes sont utilisés
directement, sans créer de copies des paquets. Modifiez les fichiers dans
`workspace/src`, puis relancez la compilation. Par défaut, elle utilise deux
processus de travail pour limiter l'utilisation de la RAM ; pour un robot
disposant de moins de mémoire, utilisez :

```bash
BUILD_JOBS=1 bash build-robot.sh
```

### Environnement Bash et alias

Après une compilation réussie, `build-robot.sh` exécute `setup-shell.sh`.
Celui-ci ajoute un bloc géré dans votre `~/.bashrc`, qui charge
`/opt/ros/foxy/setup.bash`, puis `workspace/install/setup.bash` si ce
fichier existe, et enfin `aliases/limo.bash`. Lorsqu'il est relancé, le script
remplace son propre bloc sans le dupliquer ni modifier les autres paramètres
du shell. Le fichier `.bashrc` original est sauvegardé une seule fois dans
`~/.bashrc.limo-backup`, avant la première modification.

Pour configurer le shell avant la première compilation, ou actualiser les
chemins après avoir déplacé le projet, exécutez depuis `ros2_foxy_dev/limo` :

```bash
bash setup-shell.sh
```

Ouvrez un nouveau terminal ou chargez la configuration dans le terminal actuel :

```bash
source ~/.bashrc
```

| Alias | Action |
| --- | --- |
| `cb_limo` | Exécuter le script de compilation du robot avec `colcon build --symlink-install` |
| `wsp` | Accéder au répertoire `workspace` du projet |
| `start` | Lancer le robot réel avec `launch-robot.sh` |

L'alias de compilation compile le SDK YDLidar avant colcon, puis les paquets
du robot physique directement dans `workspace`. Le répertoire sélectionné
par `wsp` contient `src`, `build`, `install` et `log`. Vous pouvez ajouter des
options de compilation et de lancement après l'alias, par exemple :

```bash
BUILD_JOBS=1 cb_limo --without-camera
start use_camera:=false use_lidar:=false
```

Après une nouvelle compilation, rechargez `~/.bashrc` ou ouvrez un nouveau
terminal pour charger l'environnement du workspace mis à jour. `start`
charge également cet environnement lui-même.

## 6. Lancer le LIMO pour la première fois

Depuis `ros2_foxy_dev/limo`, exécutez :

```bash
bash launch-robot.sh
```

Le script charge Foxy et le workspace du robot, puis lance :

```text
custom_start/limo_real.launch.py
```

Cela démarre le châssis, la transformation de l'IMU, le lidar, la caméra et la
localisation EKF. RViz est désactivé par défaut. Une compilation sans caméra
transmet automatiquement `use_camera:=false`.

Le launch utilise la **configuration Ackermann** actuelle du workspace et
son étalonnage de la direction. Vérifiez que ces paramètres correspondent
au châssis physique. Il ne lance pas automatiquement la navigation ni le SLAM.

Si le châssis utilise un adaptateur série USB, indiquez le **nom du périphérique
sans `/dev/`**, par exemple :

```bash
bash launch-robot.sh port_name:=ttyUSB0
```

Le pilote actuel du châssis ajoute lui-même `/dev/` lorsque le nom contient
`tty`. Le port du lidar est configuré séparément dans
`workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml`.

Pour lancer uniquement le châssis et l'EKF pendant le dépannage des capteurs :

```bash
bash launch-robot.sh use_camera:=false use_lidar:=false
```

Pour lancer RViz dans une session graphique locale :

```bash
bash launch-robot.sh open_rviz:=true
```

Utilisez **Ctrl+C** dans le terminal du launch pour arrêter les nœuds.

## 7. Vérifier les topics ROS

Ouvrez un deuxième terminal sur le robot et chargez le même environnement :

```bash
source /opt/ros/foxy/setup.bash
source ~/Limo/workspace/install/setup.bash
ros2 topic list
```

Vérifiez les topics des composants que vous avez activés :

```bash
ros2 topic hz /odom
ros2 topic hz /limo/imu
ros2 topic hz /odometry/filtered
ros2 topic hz /scan
ros2 topic hz /rgb/image_raw
ros2 topic hz /depth_camera/depth/image_raw
```

Arrêtez chaque vérification de topic avec Ctrl+C avant de passer à la suivante.

Pour afficher un topic d'images, exécutez cette commande dans un terminal
d'une session graphique locale, après avoir chargé l'environnement ROS :

```bash
ros2 run rqt_image_view rqt_image_view
```

Sélectionnez `/rgb/image_raw` ou `/depth_camera/depth/image_raw` dans la liste
des topics. Consultez la [documentation du paquet rqt_image_view](https://index.ros.org/p/rqt_image_view/).

Le profil actuel de la caméra désactive la publication du nuage de points ;
`/depth/points` n'est donc pas attendu par défaut. Nav2 et le SLAM nécessitent
leurs propres commandes launch ainsi qu'une carte et des paramètres, en plus
de ce lancement du matériel.

Pour utiliser un autre ordinateur ROS sur le même réseau, définissez le même
`ROS_DOMAIN_ID` sur les deux systèmes. Ce script utilise le domaine `0` par
défaut ; pour le modifier :

```bash
ROS_DOMAIN_ID=10 bash launch-robot.sh
```

Utilisez la même valeur dans le deuxième terminal avant d'inspecter les topics.

## Fichiers et limites des sources

| Fichier | Rôle |
| --- | --- |
| `install-deps.sh` | Installer les dépendances système Ubuntu/ROS et exécuter rosdep |
| `apt-packages.txt` | Liste explicite des paquets sans Gazebo |
| `no-gazebo.pref` | Bloquer les nouveaux paquets Gazebo pendant l'installation des dépendances |
| `setup-devices.sh` | Installer les règles du matériel et configurer les groupes d'utilisateurs |
| `99-limo-hardware.rules` | Permissions du châssis, du lidar et de la caméra |
| `prepare-workspace.py` | Créer des manifestes temporaires pour l'installation des dépendances du robot |
| `build-robot.sh` | Compiler le SDK et les paquets du robot physique |
| `setup-shell.sh` | Configurer Foxy, l'installation du robot et les alias dans `.bashrc` |
| `aliases/limo.bash` | Définir `cb_limo`, `wsp` et `start` |
| `launch-robot.sh` | Charger l'environnement et démarrer le launch du robot réel |

Les cinq répertoires de `workspace/src/ros2_ws` contiennent actuellement des
caches Python, mais aucun module source ni manifeste de paquet. Ils ne sont
pas inclus dans cette compilation. Restaurez leurs sources originales avant
d'ajouter des paquets de commande, de trajectoire ou de perception. Ce profil
n'installe pas ONNX Runtime ni PyTurboJPEG : le launch du robot physique et
les pilotes disponibles ne les importent pas, et les sources de perception
manquantes doivent être vérifiées avant de choisir des versions compatibles
avec le robot.
