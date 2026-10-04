#!/usr/bin/env bash
set -euo pipefail

if ! grep -qi microsoft /proc/sys/kernel/osrelease; then
  echo "Eseguire questo script dalla distro Ubuntu in WSL2." >&2
  exit 1
fi
if [[ "$(id -u)" == 0 ]]; then
  echo "Usare l'utente normale della distro WSL, per mantenere corretti UID/GID." >&2
  exit 1
fi
if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
  echo "Abilitare Ubuntu in Docker Desktop > Settings > Resources > WSL Integration." >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo "Avviare Docker Desktop e verificare WSL Integration." >&2
  exit 1
fi
if [[ "$(docker info --format '{{.OSType}}')" != linux ]]; then
  echo "Selezionare Linux containers in Docker Desktop." >&2
  exit 1
fi
if [[ ! -d /mnt/wslg/.X11-unix ]]; then
  echo "WSLg non disponibile: eseguire wsl --update su Windows, poi riavviare WSL." >&2
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "$SCRIPT_DIR/../../workspace/src" ]]; then
  echo "Manca workspace/src: copiare l'intero progetto in WSL." >&2
  exit 1
fi
docker version
docker compose version
echo "Host pronto. Il motore Docker e gestito da Docker Desktop; proseguire con bash dev.sh build."
