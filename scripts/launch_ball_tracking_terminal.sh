#!/usr/bin/env bash

/home/rigon/Dv-Rosws/Dv-Rosws/scripts/launch_ball_tracking.sh
status=$?

echo
echo "Programme termine (code ${status}). Appuie sur Entree pour fermer."
read -r
