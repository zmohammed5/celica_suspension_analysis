#!/bin/bash
#
# Celica Suspension DAQ - Installation Script
#
# This script sets up the Raspberry Pi for the suspension
# data acquisition system, including I2C, 1-Wire, and
# serial port configuration.
#

set -e

echo "======================================"
echo "Celica Suspension DAQ - Setup Script"
echo "======================================"

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (sudo)"
   exit 1
fi

# Update system
echo ""
echo "Updating system packages..."
apt-get update
apt-get upgrade -y

# Install system dependencies
echo ""
echo "Installing system dependencies..."
apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-smbus \
    i2c-tools \
    git \
    bluetooth \
    bluez \
    libbluetooth-dev

# Enable I2C
echo ""
echo "Enabling I2C..."
if ! grep -q "^dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" >> /boot/config.txt
fi

# Enable 1-Wire for temperature sensors
echo ""
echo "Enabling 1-Wire..."
if ! grep -q "^dtoverlay=w1-gpio" /boot/config.txt; then
    echo "dtoverlay=w1-gpio" >> /boot/config.txt
fi

# Enable serial port (for GPS)
echo ""
echo "Enabling serial port..."
if ! grep -q "^enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" >> /boot/config.txt
fi

# Disable serial console (conflicts with GPS)
sed -i 's/console=serial0,115200 //g' /boot/cmdline.txt 2>/dev/null || true

# Load I2C module
echo ""
echo "Loading I2C modules..."
modprobe i2c-dev
if ! grep -q "^i2c-dev" /etc/modules; then
    echo "i2c-dev" >> /etc/modules
fi

# Set I2C speed for faster sensor reading
if ! grep -q "^dtparam=i2c_arm_baudrate" /boot/config.txt; then
    echo "dtparam=i2c_arm_baudrate=400000" >> /boot/config.txt
fi

# Create project directory
PROJECT_DIR="/home/pi/celica_suspension_analysis"
echo ""
echo "Setting up project directory..."
mkdir -p "$PROJECT_DIR"
chown pi:pi "$PROJECT_DIR"

# Create virtual environment
echo ""
echo "Creating Python virtual environment..."
cd "$PROJECT_DIR"
sudo -u pi python3 -m venv venv

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
sudo -u pi "$PROJECT_DIR/venv/bin/pip" install --upgrade pip
sudo -u pi "$PROJECT_DIR/venv/bin/pip" install -r requirements.txt

# Create data directories
echo ""
echo "Creating data directories..."
sudo -u pi mkdir -p "$PROJECT_DIR/data/sessions"
sudo -u pi mkdir -p "$PROJECT_DIR/logs"

# Setup Bluetooth for ELM327
echo ""
echo "Setting up Bluetooth..."
systemctl enable bluetooth
systemctl start bluetooth

# Add user to necessary groups
echo ""
echo "Adding user to required groups..."
usermod -a -G i2c,dialout,bluetooth pi

# Install systemd service
echo ""
echo "Installing systemd service..."
cp "$PROJECT_DIR/scripts/systemd/suspension_daq.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable suspension_daq.service

echo ""
echo "======================================"
echo "Setup complete!"
echo "======================================"
echo ""
echo "Please reboot the Raspberry Pi for changes to take effect:"
echo "  sudo reboot"
echo ""
echo "After reboot, start the DAQ system with:"
echo "  sudo systemctl start suspension_daq"
echo ""
echo "Or run manually with:"
echo "  cd $PROJECT_DIR"
echo "  source venv/bin/activate"
echo "  python src/main.py"
echo ""
echo "Access the dashboard at: http://<raspberry_pi_ip>:5000"
echo ""
