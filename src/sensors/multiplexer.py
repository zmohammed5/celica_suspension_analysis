"""
TCA9548A I2C Multiplexer Driver.

This module provides an interface to the TCA9548A 8-channel I2C multiplexer,
which allows multiple I2C devices with the same address to be used on a single bus.
Used for reading multiple MPU6050 accelerometers at the four corners of the vehicle.
"""

import logging
from typing import Optional

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

logger = logging.getLogger(__name__)


class TCA9548A:
    """
    TCA9548A I2C Multiplexer Driver.

    The TCA9548A has 8 bidirectional translating switches that can be controlled
    through the I2C bus. Each channel can be individually selected.

    Attributes:
        address: I2C address of the multiplexer (default 0x70)
        bus_num: I2C bus number (default 1 for Raspberry Pi)
        current_channel: Currently selected channel (None if all disabled)
    """

    def __init__(
        self,
        address: int = 0x70,
        bus_num: int = 1,
        simulate: bool = False
    ):
        """
        Initialize the TCA9548A multiplexer.

        Args:
            address: I2C address (0x70-0x77 based on A0-A2 pins)
            bus_num: I2C bus number
            simulate: If True, run in simulation mode without hardware
        """
        self.address = address
        self.bus_num = bus_num
        self.simulate = simulate or not HAS_SMBUS
        self.current_channel: Optional[int] = None
        self._bus: Optional['smbus2.SMBus'] = None

        if self.simulate:
            logger.warning("TCA9548A running in simulation mode")
        else:
            self._init_bus()

    def _init_bus(self) -> None:
        """Initialize the I2C bus connection."""
        try:
            self._bus = smbus2.SMBus(self.bus_num)
            logger.info(f"TCA9548A initialized at address 0x{self.address:02X}")
        except Exception as e:
            logger.error(f"Failed to initialize I2C bus: {e}")
            self.simulate = True

    def select_channel(self, channel: int) -> bool:
        """
        Select a specific multiplexer channel.

        Args:
            channel: Channel number (0-7)

        Returns:
            True if channel was selected successfully

        Raises:
            ValueError: If channel number is out of range
        """
        if not 0 <= channel <= 7:
            raise ValueError(f"Channel must be 0-7, got {channel}")

        if self.simulate:
            self.current_channel = channel
            return True

        try:
            # Write the channel bit to the control register
            self._bus.write_byte(self.address, 1 << channel)
            self.current_channel = channel
            logger.debug(f"Selected multiplexer channel {channel}")
            return True
        except Exception as e:
            logger.error(f"Failed to select channel {channel}: {e}")
            return False

    def disable_all(self) -> bool:
        """
        Disable all multiplexer channels.

        Returns:
            True if all channels were disabled successfully
        """
        if self.simulate:
            self.current_channel = None
            return True

        try:
            self._bus.write_byte(self.address, 0x00)
            self.current_channel = None
            logger.debug("All multiplexer channels disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to disable all channels: {e}")
            return False

    def scan_channels(self) -> dict[int, list[int]]:
        """
        Scan all channels for connected I2C devices.

        Returns:
            Dictionary mapping channel numbers to lists of detected device addresses
        """
        results: dict[int, list[int]] = {}

        for channel in range(8):
            self.select_channel(channel)
            devices = self._scan_bus()
            if devices:
                results[channel] = devices
                logger.info(f"Channel {channel}: devices at {[hex(d) for d in devices]}")

        self.disable_all()
        return results

    def _scan_bus(self) -> list[int]:
        """
        Scan the I2C bus for connected devices.

        Returns:
            List of detected device addresses
        """
        if self.simulate:
            return [0x68]  # Simulated MPU6050

        devices = []
        for addr in range(0x08, 0x78):
            if addr == self.address:
                continue
            try:
                self._bus.read_byte(addr)
                devices.append(addr)
            except Exception:
                pass
        return devices

    def get_channel(self) -> Optional[int]:
        """
        Get the currently selected channel.

        Returns:
            Current channel number or None if all disabled
        """
        return self.current_channel

    def __enter__(self) -> 'TCA9548A':
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - disable all channels and close bus."""
        self.disable_all()
        self.close()

    def close(self) -> None:
        """Close the I2C bus connection."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None


class MultiplexedDevice:
    """
    Base class for devices connected through a TCA9548A multiplexer.

    Provides automatic channel selection before device access.
    """

    def __init__(
        self,
        multiplexer: TCA9548A,
        channel: int,
        device_address: int
    ):
        """
        Initialize a multiplexed device.

        Args:
            multiplexer: TCA9548A multiplexer instance
            channel: Multiplexer channel this device is connected to
            device_address: I2C address of the device
        """
        self.multiplexer = multiplexer
        self.channel = channel
        self.device_address = device_address

    def _select_channel(self) -> bool:
        """
        Select the multiplexer channel for this device.

        Returns:
            True if channel was selected successfully
        """
        return self.multiplexer.select_channel(self.channel)
