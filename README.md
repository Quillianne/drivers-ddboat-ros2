
# Installation sur matériel existant


Sur les DDBoat actuels, il peut être utile de lancer :
```bash
sudo raspi-config
```
et d'utiliser l'option "Expand Filesystem" pour que tout l'espace de la carte SD soit disponible. Redémarrez ensuite le Raspberry Pi.

2. **Installer les drivers et Docker**
  - Placez-vous dans votre dossier personnel puis clonez le dépôt et lancez l'installation :
    ```bash
    cd $HOME
    git clone https://github.com/Quillianne/drivers-ddboat-ros2
    cd drivers-ddboat-ros2/install_ddboat
    ./install.bash
    ```
   - Ce script installe Docker, Compose, git, wget et configure le système pour DDBoat.
   - Il lance automatiquement `docker compose` en mode reboot : les services démarrent à chaque redémarrage du Raspberry Pi.
   - Si vous souhaitez que les services ne démarrent pas automatiquement, il faut stopper le stack avec :
     ```bash
     docker compose --profile hw down
     ```

3. **Configurer le projet**
  - Adaptez le fichier `.env` selon votre matériel (ports, noms d’images) mais il devrait être fonctionnel sur les ddboat


Cette procédure est idéale pour installer rapidement sur un DDBoat tel qu'ils sont actuellement (septembre 2025)

# DDBoat ROS 2 Drivers — Guide d'installation complet

Ce dépôt regroupe tous les drivers bas niveau pour piloter un DDBoat via **ROS 2 Humble** sur Raspberry Pi (32 ou 64 bits). Tout est prévu pour tourner dans un conteneur Docker, avec orchestration via `docker-compose`.

---

## Installation rapide sur Raspberry Pi (debian bullseye ou bookworm)

### 1. Préparation et installation de Docker

Ouvrez un terminal et lancez les commandes suivantes pour installer Docker et ses dépendances :

```bash
# Supprimer les anciens paquets Docker
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
  sudo apt-get remove -y $pkg
done

# Ajouter la clé GPG officielle de Docker
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Ajouter le dépôt Docker
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
  https://download.docker.com/linux/raspbian \
  $(. /etc/os-release && echo \"$VERSION_CODENAME\") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Installer Docker et les outils associés
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# Ajouter votre utilisateur au groupe docker
sudo usermod -aG docker $USER
newgrp docker  # (optionnel, pour appliquer le changement sans déconnexion)
```

### 2. Récupération du projet et installation

Placez-vous dans votre dossier personnel, clonez le dépôt et lancez l'installation :

```bash
cd $HOME
git clone https://github.com/Quillianne/drivers-ddboat-ros2
cd drivers-ddboat-ros2/install_ddboat
./install.bash
```

Ce script installe les dépendances supplémentaires et lance automatiquement `docker compose` en mode reboot (démarrage automatique à chaque redémarrage).
Si vous souhaitez désactiver ce démarrage automatique, utilisez :
```bash
docker compose --profile hw down
```

### 3. Activation des interfaces matérielles

Pour que les capteurs IMU et température fonctionnent, activez l'I2C :

```bash
sudo raspi-config
# Interface Options > Serial > desactiver terminal série
# Interface Options > SPI > Enable
# Interface Options > I2C > Enable puis redémarrez

```

---

## Récupération et configuration du projet

Clonez le dépôt :

```bash
git clone https://github.com/Quillianne/drivers-ddboat-ros2
cd drivers-ddboat-ros2
```

### Configuration des ports et images

Adaptez le fichier `.env` à votre matériel :

```bash
# Exemple de .env
IMAGE=quillianne/ddboat
ROS2_IMAGE=quillianne/ros2
GPS_DEV=/dev/ttyGPS0
ARDUINO_DEV=/dev/ttyV0
ENC_DEV=/dev/ttyENC0
LORA_DEV=/dev/ttyLORA1
```

---

## Lancement des drivers avec Docker Compose

Trois profils sont disponibles :

- `hw` : tous les drivers dans un seul conteneur (pour le vrai hardware)
- `sim` : émulation logicielle pour développement
- `hw_extra` : drivers séparés pour debug

Lancez tous les drivers :

```bash
docker compose --profile hw up -d
```

Pour voir les logs :

```bash
docker compose --profile hw logs -f
```

Pour arrêter ou redémarrer :

```bash
docker compose --profile hw stop
docker compose --profile hw start
docker compose --profile hw down
```

---

## Mise à jour ou recompilation des images Docker

Pour mettre à jour les images :

```bash
docker compose --profile hw pull
```

Pour recompiler localement (après modification du code) :

```bash
sh build_and_push.sh
```

---

## Utilisation des drivers depuis Python

Une fois le service rosbridge lancé, vous pouvez interagir avec le bateau en Python via `roslibpy` :

```python
import roslibpy
client = roslibpy.Ros(host='localhost', port=9090)
client.run()
twist_pub = roslibpy.Topic(client, '/motors_cmd', 'geometry_msgs/Twist')
twist_pub.publish(roslibpy.Message({'linear': {'x': 50.0, 'y': 50.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}))
twist_pub.unadvertise()
client.terminate()
```

Des exemples complets sont disponibles dans le dossier `tests/`.

---

## Architecture des nodes

| Node (exécutable) | Rôle | Topic(s) |
|-------------------|------|----------|
| `gps_node`        | GNSS position | `sensor_msgs/NavSatFix` |
| `arduino_node`    | Commande moteurs Arduino | `geometry_msgs/Twist` (sub) |
| `encoders_node`   | Comptage propulseurs | `std_msgs/Int32MultiArray` |
| `imu_node`        | IMU 9 axes + heading | `sensor_msgs/Imu`, `sensor_msgs/MagneticField`, `std_msgs/Float64` |
| `temperature_node`| Températures moteurs | `sensor_msgs/Temperature` |
| `radio_node`      | Communication LoRa | `std_msgs/String` |

---

## Conseils pratiques

- Après installation, vérifiez que tout l'espace de la carte SD est utilisé (`raspi-config expand filesystem`).
- Le script `install.bash` automatise toute la configuration sur les DDBoat actuels.
- Pour tout problème matériel, vérifiez les ports dans `.env`.
- Pour le debug, utilisez les profils `sim` ou `hw_extra`.

---

## Communication avec le bateau en Python (`roslibpy`)

La librairie Python `roslibpy` permet de communiquer avec le bateau via le protocole rosbridge. Une fois le service `rosbridge` lancé (via `docker compose`), vous pouvez publier et recevoir des messages sur le réseau :




```python
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

twist_pub = roslibpy.Topic(client, '/motors_cmd', 'geometry_msgs/Twist')
twist_pub.publish(roslibpy.Message({'linear': {'x': 50.0, 'y': 50.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}))
twist_pub.unadvertise()

client.terminate()
```

Des exemples complets sont disponibles dans le dossier `tests/`.
Pour les exécuter :

```bash
pip install -r requirements.txt
python3 tests/test_xxxx_xxxx.py
```

---

## Construction des images Docker pour DDBoat

### Construction sur un PC

Il est recommandé de construire les images Docker sur un PC pour réduire le temps de compilation. Voir [`ros2_image_builder/README.md`](ros2_image_builder/README.md) pour plus de détails.

Pour construire et pousser les images :

```bash
# Pour l'image ros2
cd ros2_image_builder && sh build_and_push.sh

# Pour l'image ddboat (depuis la racine du projet)
sh build_and_push.sh
```

Une fois les images poussées, le déploiement sur Raspberry Pi se fait simplement avec `docker compose`.

---

### Construction sur le Raspberry Pi

Vous pouvez aussi construire les images directement sur le Raspberry Pi. Pensez à adapter le fichier `.env` avec le nom d'image local.

#### Recompiler l'image ddboat après modification des drivers

```bash
# Depuis la racine du dépôt
docker build -t ddboat_ros2 .
```

Pour les cartes ARM 64 bits, vous pouvez utiliser le Dockerfile alternatif pour une image plus légère :

```bash
docker build -t ddboat_ros2 -f old_Dockerfile .
```

La librairie `wjwwood/serial` est incluse dans l'image, aucune installation supplémentaire n'est nécessaire sur le système hôte.

---

#### Recompiler l'image ros2 après ajout de dépendances

L'image de base ROS 2 inclut uniquement les dépendances minimales et les paquets essentiels.
Si vous ajoutez des dépendances dans le code, il faut recompiler l'image ros2 :

```bash
# Dans le dossier ros2_image_builder
./build_ros2.sh
```

Testez l'image et poussez-la si vous souhaitez l'utiliser sur plusieurs appareils.
