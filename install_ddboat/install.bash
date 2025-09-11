#!/bin/bash
set -euo pipefail

echo "=== DDBoat • Installation Docker + Compose (armhf) & lancement profile=hw ==="

# 0) Pré-requis minimes
if ! command -v wget >/dev/null 2>&1; then
  echo "[prep] Installation de wget..."
  sudo apt-get update -y
  sudo apt-get install -y wget
fi
if ! command -v git >/dev/null 2>&1; then
  echo "[prep] Installation de git..."
  sudo apt-get update -y
  sudo apt-get install -y git
fi

# 1) Téléchargements
WORKDIR="$HOME/docker_install"
mkdir -p "$WORKDIR"
cd "$WORKDIR"
echo "[1/6] Téléchargement des paquets .deb Docker (Stretch/Buster armhf)..."
wget -O containerd.io_1.4.3-1_armhf.deb \
  https://download.docker.com/linux/raspbian/dists/stretch/pool/stable/armhf/containerd.io_1.4.3-1_armhf.deb
wget -O docker-ce_19.03.9_armhf.deb \
  https://download.docker.com/linux/raspbian/dists/stretch/pool/stable/armhf/docker-ce_19.03.9~3-0~raspbian-stretch_armhf.deb
wget -O docker-ce-cli_19.03.9_armhf.deb \
  https://download.docker.com/linux/raspbian/dists/stretch/pool/stable/armhf/docker-ce-cli_19.03.9~3-0~raspbian-stretch_armhf.deb
wget -O docker-compose-plugin_2.6.0_armhf.deb \
  https://download.docker.com/linux/raspbian/dists/buster/pool/stable/armhf/docker-compose-plugin_2.6.0~raspbian-buster_armhf.deb

# 2) Installation
echo "[2/6] Installation de Docker CE, CLI, containerd et plugin compose..."
sudo dpkg -i containerd.io_1.4.3-1_armhf.deb \
            docker-ce_19.03.9_armhf.deb \
            docker-ce-cli_19.03.9_armhf.deb \
            docker-compose-plugin_2.6.0_armhf.deb || true

echo "[3/6] Résolution des dépendances (si besoin)..."
sudo apt-get install -f -y

# 3) Activer et démarrer Docker (et containerd)
echo "[4/6] Activation des services au démarrage..."
sudo systemctl enable --now containerd || true
sudo systemctl enable --now docker || true

# 4) Groupe docker
echo "[5/6] Ajout de l’utilisateur '$USER' au groupe docker (pour éviter sudo)..."
if ! id -nG "$USER" | grep -qw docker; then
  sudo usermod -aG docker "$USER"
  ADDED_TO_GROUP=1
else
  ADDED_TO_GROUP=0
fi

# 5) Repo drivers-ddboat-ros2
echo "[6/6] Récupération du dépôt drivers-ddboat-ros2..."
cd "$HOME"
if [ ! -d "$HOME/drivers-ddboat-ros2" ]; then
  git clone https://github.com/Quillianne/drivers-ddboat-ros2
else
  (cd drivers-ddboat-ros2 && git pull --ff-only) || true
fi

# 6) Lancement automatique du profile=hw avec droits docker
echo ""
echo "=== Lancement de 'docker compose --profile hw up -d' ==="
cd "$HOME/drivers-ddboat-ros2"

# Si l’utilisateur vient d’être ajouté au groupe docker, sa session ne le voit pas encore.
# On exécute donc la commande via le groupe docker immédiatement (sans attendre une reconnexion).
if command -v sg >/dev/null 2>&1; then
  # Tente d’utiliser le groupe docker; sinon, fallback en sudo
  if [ "$ADDED_TO_GROUP" -eq 1 ]; then
    echo "(session pas encore rafraîchie) utilisation de 'sg docker -c ...'"
    sg docker -c "docker compose --profile hw pull && docker compose --profile hw up -d" \
      || sudo docker compose --profile hw up -d
  else
    docker compose --profile hw pull || true
    docker compose --profile hw up -d
  fi
else
  echo "('sg' indisponible) utilisation de sudo pour lancer docker compose."
  sudo docker compose --profile hw pull || true
  sudo docker compose --profile hw up -d
fi

echo ""
echo "Installation & lancement terminés."
echo "ℹLe container ddboat tourne et redémarrera automatiquement au boot."
echo "    (Il tournera encore après redémarrage **sauf si tu l’arrêtes manuellement**.)"
echo "    - Voir les conteneurs en cours :  docker ps"
echo "    - Voir tous les conteneurs :     docker ps -a"
echo "    - Arrêter le stack DDBoat :      docker compose --profile hw down   (depuis drivers-ddboat-ros2)"
echo ""
if [ "$ADDED_TO_GROUP" -eq 1 ]; then
  echo "Tu as été ajouté au groupe 'docker'. Pour que ta session en profite sans sudo :"
  echo "   • soit tu te déconnectes/reconnectes,"
  echo "   • soit tu lances:  newgrp docker"
fi