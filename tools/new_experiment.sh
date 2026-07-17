#!/usr/bin/env bash
# Crea la carpeta de un experimento nuevo dentro de experiments/ y apunta
# el enlace experiments/current a ella. Los comandos del bench test escriben
# siempre a experiments/current, así que cada corrida queda en su carpeta.
#
# Uso:
#   ./tools/new_experiment.sh            → experiments/2026-07-17_123456/
#   ./tools/new_experiment.sh vuelo1     → experiments/2026-07-17_123456_vuelo1/
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${1:-}"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
DIR="$REPO/experiments/${STAMP}${NAME:+_$NAME}"

mkdir -p "$DIR"
ln -sfn "$DIR" "$REPO/experiments/current"

echo "Experimento nuevo: $DIR"
echo "experiments/current ya apunta aquí — corre las terminales del bench test."
