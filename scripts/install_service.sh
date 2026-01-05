#!/bin/bash
#
# Install the suspension DAQ as a systemd service
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SERVICE_FILE="$PROJECT_DIR/scripts/systemd/suspension_daq.service"

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (sudo)"
   exit 1
fi

# Update paths in service file
ACTUAL_PROJECT_DIR=$(realpath "$PROJECT_DIR")
sed -i "s|/home/pi/celica_suspension_analysis|$ACTUAL_PROJECT_DIR|g" "$SERVICE_FILE"

# Copy service file
cp "$SERVICE_FILE" /etc/systemd/system/

# Reload systemd
systemctl daemon-reload

# Enable service
systemctl enable suspension_daq.service

echo ""
echo "Service installed successfully!"
echo ""
echo "Commands:"
echo "  Start:   sudo systemctl start suspension_daq"
echo "  Stop:    sudo systemctl stop suspension_daq"
echo "  Status:  sudo systemctl status suspension_daq"
echo "  Logs:    journalctl -u suspension_daq -f"
echo ""
