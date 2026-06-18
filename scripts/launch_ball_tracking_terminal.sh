#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/launch_ball_tracking.sh"
status=$?

echo
echo "Programme termine (code ${status}). Appuie sur Entree pour fermer."
read -r
