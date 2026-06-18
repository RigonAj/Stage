#!/usr/bin/env bash
set -euo pipefail

APP_NAME="UR3e Control"
DESKTOP_ID="ur3e-control.desktop"
URL="${UR3E_WEB_APP_URL:-http://127.0.0.1:8080}"
INSTALL_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
DESKTOP_FILE="$INSTALL_DIR/$DESKTOP_ID"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--url URL]

Installs a Linux desktop launcher that opens the UR3e web UI as an app window.
Default URL: $URL

Examples:
  $(basename "$0")
  $(basename "$0") --url http://192.168.0.3:8080
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --url)
      URL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

find_browser() {
  for candidate in google-chrome chromium-browser chromium brave-browser microsoft-edge firefox; do
    if command -v "$candidate" >/dev/null 2>&1; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

BROWSER="$(find_browser || true)"
if [[ -z "$BROWSER" ]]; then
  echo "ERROR: no supported browser found (Chrome, Chromium, Brave, Edge or Firefox)." >&2
  exit 1
fi

mkdir -p "$INSTALL_DIR"

case "$BROWSER" in
  firefox)
    EXEC_LINE="$BROWSER --new-window $URL"
    ;;
  *)
    EXEC_LINE="$BROWSER --app=$URL --class=UR3eControl"
    ;;
esac

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=$APP_NAME
Comment=Open the UR3e Control web UI
Exec=$EXEC_LINE
Terminal=false
Categories=Development;Robotics;
StartupWMClass=UR3eControl
EOF

chmod +x "$DESKTOP_FILE"

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$INSTALL_DIR" >/dev/null 2>&1 || true
fi

echo "Installed: $DESKTOP_FILE"
echo "URL: $URL"
echo "Start the backend first, then open '$APP_NAME' from the application menu."
