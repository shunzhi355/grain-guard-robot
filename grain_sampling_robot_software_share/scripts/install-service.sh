#!/bin/bash
set -e

SERVICE_NAME="grain-sampling"
SERVICE_FILE="$(dirname "$0")/${SERVICE_NAME}.service"
TARGET="/etc/systemd/system/${SERVICE_NAME}.service"

if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: Service file not found: $SERVICE_FILE"
    exit 1
fi

echo "=== Installing ${SERVICE_NAME} service ==="
echo "[1/4] Copying service file..."
sudo cp "$SERVICE_FILE" "$TARGET"
echo "[2/4] Reloading systemd..."
sudo systemctl daemon-reload
echo "[3/4] Enabling on boot..."
sudo systemctl enable "${SERVICE_NAME}"
echo "[4/4] Verifying syntax..."
sudo systemd-analyze verify "${SERVICE_NAME}.service" 2>&1 || true
echo ""
echo "=== Done ==="
echo "Start: sudo systemctl start ${SERVICE_NAME}"
echo "Status: sudo systemctl status ${SERVICE_NAME}"
echo "Logs: sudo journalctl -u ${SERVICE_NAME} -f"
