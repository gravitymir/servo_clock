# ESP32-C3 Project with Dual PCA9685 Servo Control

This project implements an ESP32-C3 development board with:
- Serial communication via USB Type-C
- WiFi Access Point (AP) mode with web interface
- I2C connection to DS3231 RTC and AT24C32 EEPROM module
- **Two PCA9685 PWM servo driver boards with GPIO power control**
- **14 servos per board (28 servos total)**

## Hardware Requirements

- ESP32-C3 development board (with USB Type-C)
- DIYUSER DS3231 AT24C32 IIC Module
- **Two PCA9685 PWM Servo Driver boards** (with different I2C addresses)
- **Power control modules** (for turning PCA9685 boards on/off)
- Jumper wires for I2C connection and GPIO control

## Wiring Diagram

### I2C Devices (Shared Bus)
Connect all I2C devices to the same bus:

```
DS3231 Module    ESP32-C3
-----------      --------
VCC          ->  3.3V
GND          ->  GND
SDA          ->  GPIO5
SCL          ->  GPIO6

PCA9685 Board 1  ESP32-C3
--------------   --------
VCC          ->  5V (or 3.3V if supported)
GND          ->  GND
SDA          ->  GPIO5 (shared I2C bus)
SCL          ->  GPIO6 (shared I2C bus)
Address      ->  Set to 0x40 (default) or configure with jumpers

PCA9685 Board 2  ESP32-C3
--------------   --------
VCC          ->  5V (or 3.3V if supported)
GND          ->  GND
SDA          ->  GPIO5 (shared I2C bus)
SCL          ->  GPIO6 (shared I2C bus)
Address      ->  Set to 0x41 (or different from Board 1)
```

### Power Control GPIO
One high-side (P-channel) switch on **GPIO 7** feeds **both** PCA9685 boards (same rail).

```
Power control   ESP32-C3
-------------   --------
Control pin  ->  GPIO7
(see your FET module for VCC/GND and load wiring)
```

**Note:** Firmware uses **active-LOW** on GPIO7 (**LOW** = boards powered, **HIGH** = off). Invert `setPca9685RailsPower()` if your driver is the opposite.

## Software Setup

### Prerequisites

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or standalone)
2. Connect ESP32-C3 via USB Type-C cable

### Build and Upload

1. Open this project in PlatformIO
2. Build the project:
   ```bash
   pio run
   ```
3. Upload to ESP32-C3:
   ```bash
   pio run -t upload
   ```
4. Open Serial Monitor:
   ```bash
   pio device monitor
   ```
   Or use the Serial Monitor button in PlatformIO

## Features

### Serial Communication
- Baud rate: 115200
- View system status, RTC time, and debug information
- Connect via USB Type-C cable

### WiFi Access Point
- SSID: `ESP32-C3-AP`
- Password: `12345678`
- IP Address: `192.168.4.1`
- Open browser and navigate to: `http://192.168.4.1`

### Web Interface
The web interface provides:
- System status display
- Real-time RTC clock display
- Time synchronization with browser
- EEPROM (AT24C32) test functionality
- **PCA9685 servo control with power management**
- **Power status display and manual control**
- Auto-refresh every 5 seconds

### I2C Devices
- **DS3231 RTC**: Real-time clock module
  - I2C Address: 0x68 (default)
  - Provides accurate date and time
- **AT24C32 EEPROM**: 4KB non-volatile memory
  - I2C Address: 0x57
  - Can store data persistently
- **PCA9685 Board 1**: PWM servo driver
  - I2C Address: 0x40 (default, configurable)
  - Controls 14 servos (channels 0-13)
- **PCA9685 Board 2**: PWM servo driver
  - I2C Address: 0x41 (or different from Board 1)
  - Controls 14 servos (channels 0-13)
  - Both boards share **GPIO7** high-side power enable

### Power Management
- **Default at boot**: GPIO7 **HIGH** = rails **off**. Rails go **LOW** only for scan/servo moves.
- **Servos / time display**: Pin **LOW** while powered; after the hold timer, **HIGH** (off) again.
- **Web UI**: Manual GPIO7 toggle still available; idle default after boot is **off**.

### GPIO Control
- **GPIO7**: **LOW** = both boards powered, **HIGH** = off

## API Endpoints

- `GET /` - Main HTML page
- `GET /api/time` - Get current RTC time (JSON)
- `GET /api/sync?timestamp=<unix_timestamp>` - Sync RTC with timestamp
- `GET /api/eeprom-test` - Test AT24C32 EEPROM read/write
- `GET /api/pca9685-status` - Get PCA9685 modules status and power state (JSON)
- `GET /api/servo?module=<n>&channel=<ch>&value=<pwm>` - Control servo (auto-powers on board)
- `GET /api/power?gpio=7&state=<on|off>` - Enable/disable power to both boards (GPIO7)
- `GET /api/power?status` - Get power status of all boards

## Troubleshooting

### RTC Not Found
- Check I2C connections (SDA to GPIO5, SCL to GPIO6)
- Verify power connections (3.3V and GND)
- Check Serial Monitor for I2C scan results

### WiFi AP Not Visible
- ESP32-C3 may need a moment to initialize
- Check Serial Monitor for AP status
- Try resetting the board

### Serial Monitor Not Working
- Ensure correct USB Type-C cable (data-capable)
- Check COM port in Device Manager (Windows)
- Verify baud rate is set to 115200

### PCA9685 Not Found
- Check I2C connections (SDA to GPIO5, SCL to GPIO6)
- Verify both boards have different I2C addresses (0x40 and 0x41)
- Ensure GPIO7 is wired to your high-side power switch and both boards are on that rail
- Check Serial Monitor for I2C scan results
- Make sure boards are powered on (check power status in web interface)

### Servos Not Moving
- Verify the board is powered ON (check power status in web interface)
- Boards are automatically powered on when servos are moved
- Check servo connections to PCA9685 channels (0-13 for each board)
- Verify servo power supply is adequate
- Check Serial Monitor for servo control messages

## Project Structure

```
esp32_c3/
├── platformio.ini    # PlatformIO configuration
├── src/
│   └── main.cpp      # Main application code
└── README.md         # This file
```

## Libraries Used

- RTClib (Adafruit) - For DS3231 RTC communication
- Adafruit PWM Servo Driver Library - For PCA9685 control
- ESP32 WiFi and WebServer libraries (built-in)
- Wire library (built-in) - For I2C communication

## GPIO Pin Usage

- **GPIO5**: I2C SDA (shared by all I2C devices)
- **GPIO6**: I2C SCL (shared by all I2C devices)
- **GPIO7**: PCA9685 power enable for **both** boards (firmware: LOW = on, HIGH = off)

## License

This project is provided as-is for educational and development purposes.

