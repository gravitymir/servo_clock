#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <RTClib.h>

#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// WiFi AP Configuration
const char* ssid = "ESP32-C3-AP";
const char* password = "12345678";  // Minimum 8 characters

// Web Server
WebServer server(80);

// DS3231 RTC
RTC_DS3231 rtc;
bool rtcAvailable = false;  // Track if RTC is connected

// I2C Configuration — use GPIO 5 and 6 only!
// Do NOT use GPIO 0 or 1 for I2C: GPIO0 is strapping pin (boot mode), can cause empty Serial / boot issues.
#define I2C_SDA 5  // SDA = GPIO5 (safe)
#define I2C_SCL 6  // SCL = GPIO6 (safe)
#if (I2C_SDA == 0 || I2C_SCL == 0 || I2C_SDA == 1 || I2C_SCL == 1)
#error "I2C must not use GPIO 0 or 1 - use GPIO 5 and 6 for SDA/SCL"
#endif

// AT24C32 EEPROM Configuration
#define EEPROM_SIZE 4096  // AT24C32 has 4KB (4096 bytes)
#define EEPROM_I2C_ADDRESS 0x57  // AT24C32 default I2C address
#define EEPROM_ADDR_WORK_START 0x0030  // start hour 0–23 (clock active from this hour)
#define EEPROM_ADDR_WORK_STOP 0x0031   // stop hour 0–23 (active through end of this hour)

uint8_t workHoursStartHour = 9;   // default 09:00–22:59 if EEPROM invalid
uint8_t workHoursStopHour = 22;

// PCA9685 PWM Servo Driver Configuration
#define MAX_PCA9685_MODULES 10  // Support up to 10 PCA9685 modules
#define NUM_PCA9685_BOARDS 2     // Two PCA9685 boards
#define SERVOS_PER_BOARD 14      // 14 servos per board
Adafruit_PWMServoDriver* pca9685[MAX_PCA9685_MODULES] = {nullptr};
byte pca9685Addresses[MAX_PCA9685_MODULES] = {0};
int pca9685Count = 0;
bool pca9685Available = false;

// PCA9685 power: one GPIO enables both boards (same rail). **Active-LOW** on GPIO7:
// GPIO LOW = rails ON (powered), GPIO HIGH = rails OFF.
#define PCA9685_POWER_GPIO 7
bool pca9685RailsPowered = false;
static unsigned long pca9685RailsAutoOffAt = 0;  // millis deadline; 0 = none
static const uint32_t PCA9685_RAILS_PRE_MS = 300;   // after rail ON, before I2C / PWM
static const uint32_t PCA9685_RAILS_HOLD_MS = 1000; // keep rails ON (GPIO LOW) after last command, then HIGH

// Auto time display update
bool autoUpdateEnabled = true;  // 7-segment time: auto-update every minute (default ON; toggle via web)
int lastDisplayedMinute = -1;  // Track last displayed minute to update only when changed
// Time display animation delays (ms) - configurable via web
int delayBetweenDigitsMs = 0;   // Delay between switching digits (e.g. hours tens -> hours ones)
int delayBetweenServosMs = 0;   // Delay between each servo move within one digit
int lastDisplayedHour = -1;    // Track last displayed hour to update only when changed
unsigned long lastUpdateTime = 0;  // Track last update timestamp to prevent rapid updates
int rtcInvalidCount = 0;  // Count consecutive invalid RTC reads
const int MAX_RTC_INVALID_COUNT = 5;  // Stop trying after 5 consecutive invalid reads
unsigned long lastRTCErrorTime = 0;  // Track when RTC error occurred
char rtcTimeCache[24] = "";         // Last good RTC time string (for when read fails after WiFi)

// DS3231 RTC + AT24C32 EEPROM on main I2C (GPIO 5/6) with PCA9685
#define DS3231_I2C_ADDRESS 0x68
#define DS3231_CONTROL_REG 0x0E
#define DS3231_STATUS_REG 0x0F
#define DS3231_ALARM2_MINUTES 0x0B
#define DS3231_ALARM2_HOURS 0x0C
#define DS3231_ALARM2_DAYDATE 0x0D

// HTML Page
const char htmlPage[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C3 Control Panel</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .info-box {
            background: #e8f4f8;
            padding: 15px;
            border-radius: 5px;
            margin: 20px 0;
        }
        .status {
            font-weight: bold;
            color: #2e7d32;
        }
        button {
            background-color: #4CAF50;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
            margin: 5px;
        }
        button:hover {
            background-color: #45a049;
        }
        /* Full-width ON/OFF style toggles: flush in panel (parent uses padding 0) */
        .info-box--toggle-panel {
            padding: 0;
        }
        .info-box--toggle-panel > h2 {
            margin: 0;
            padding: 15px 15px 8px 15px;
        }
        .info-box--toggle-panel > p {
            margin: 0;
            padding: 0 15px 8px 15px;
        }
        .info-box--toggle-panel > input[type="time"] {
            display: block;
            margin: 0 15px 0 15px;
            width: calc(100% - 30px);
            max-width: 100%;
            box-sizing: border-box;
        }
        button.btn-toggle-full {
            width: 100%;
            max-width: 100%;
            margin: 0;
            padding: 14px 16px;
            box-sizing: border-box;
            display: block;
            border: none;
            border-radius: 0;
            font-size: 16px;
            font-weight: bold;
            color: white;
            cursor: pointer;
        }
        button.btn-toggle-full:hover {
            filter: brightness(0.92);
        }
        /* PCA power bar: edge-to-edge inside .info-box (15px padding) */
        #pca9685-power-bar {
            margin: 0 -15px 12px -15px;
            width: calc(100% + 30px);
            max-width: calc(100% + 30px);
            box-sizing: border-box;
            padding: 0;
        }
        .time-display {
            font-size: 24px;
            color: #1976d2;
            text-align: center;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32-C3 Control Panel</h1>
        
        <div class="info-box">
            <h2>System Status</h2>
            <p><span class="status">WiFi AP:</span> <span id="ap-status">Active</span></p>
            <p><span class="status">SSID:</span> ESP32-C3-AP</p>
            <p><span class="status">RTC Status:</span> <span id="rtc-status">Checking...</span></p>
        </div>
        
        <div class="info-box">
            <h2>RTC Time (DS3231)</h2>
            <p>Address: 0x68</p>
            <div class="time-display" id="rtc-time">Loading...</div>
            <button type="button" class="btn-toggle-full" onclick="updateTime()" style="background-color: #2196F3; margin-top: 12px;">Refresh Time</button>
            <button type="button" class="btn-toggle-full" onclick="syncTime()" style="background-color: #FF9800; margin-top: 8px;">Sync with Browser</button>
        </div>
        
        <div class="info-box info-box--toggle-panel">
            <h2>EEPROM (AT24C32)</h2>
            <p>Address: 0x57</p>
            <p>Size: 4KB (4096 bytes)</p>
            <button type="button" class="btn-toggle-full" onclick="testEEPROM()" style="background-color: #4CAF50; margin-top: 12px;">Test EEPROM</button>
        </div>
        
        <div class="info-box info-box--toggle-panel">
            <h2>Works hours</h2>
            <p style="margin: 0; padding: 0 15px 8px 15px;">
                Working hours limit when the firmware <strong>auto-updates</strong> the 7-segment time display from the RTC.
                While the RTC hour is between the saved start and stop (inclusive), updates run; outside that window, auto-update does nothing (manual / API display is unchanged).
            </p>
            <p style="margin: 0; padding: 0 15px 8px 15px;">
                Values are saved on the same <strong>AT24C32</strong> EEPROM as above (I²C device <strong>0x57</strong>), as two separate bytes—no multi-byte structure:
            </p>
            <ul style="margin: 0 0 8px 0; padding: 8px 15px 8px 35px;">
                <li><strong>Start hour</strong> → EEPROM byte at <strong>0x0030</strong>. Stored value <strong>0–23</strong> = hour when the window begins (e.g. <code>9</code> → from 09:00).</li>
                <li><strong>Stop hour</strong> → absolute address <strong>0x0031</strong>. One byte, <strong>0–23</strong> = last hour included in the window (e.g. <code>22</code> → through 22:59). <strong>Start must be less than stop.</strong></li>
            </ul>
            <p style="margin: 0; padding: 0 15px 12px 15px;">
                If either byte is missing or invalid (outside 0–23, or start ≥ stop), the firmware uses defaults <strong>9</strong> and <strong>22</strong>. Current window shown below: <strong><span id="work-hours-hint-start">09:00</span></strong> – <strong><span id="work-hours-hint-end">22:59</span></strong>.
            </p>
            <p style="margin: 0; padding: 8px 15px 8px 15px; display: flex; flex-wrap: wrap; align-items: center; gap: 10px;">
                <label for="work-start">Start (0–23 h)</label>
                <input type="number" id="work-start" min="0" max="23" style="width: 4em; padding: 6px; box-sizing: border-box;">
                <label for="work-stop">Stop (0–23 h)</label>
                <input type="number" id="work-stop" min="0" max="23" style="width: 4em; padding: 6px; box-sizing: border-box;">
            </p>
            <button type="button" class="btn-toggle-full" onclick="saveWorkHours()" style="background-color: #2196F3; margin-top: 8px;">Save to EEPROM</button>
        </div>
        
        <div class="info-box info-box--toggle-panel">
            <h2>Set time</h2>
            <input type="time" id="time-input" style="padding: 8px; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;" onchange="setTimeDisplay(this.value)">
            <button type="button" class="btn-toggle-full" id="auto-update-button" onclick="toggleAutoUpdate()" style="background-color: #4CAF50; margin-top: 12px;">Auto-update: ON</button>
        </div>
        
        <div class="info-box">
            <h2>PCA9685 Servo Control</h2>
            <div id="pca9685-power-bar"></div>
            <p id="pca9685-status">Loading...</p>
            <div id="pca9685-controls"></div>
        </div>
        
        <div class="info-box">
            <h2>Serial Monitor</h2>
            <p>Connect via USB Type-C to view serial output</p>
            <p>Baud Rate: 115200</p>
        </div>
    </div>
    
    <script>
        function loadPCA9685Status() {
            fetch('/api/pca9685-status')
                .then(response => response.json())
                .then(data => {
                    const statusDiv = document.getElementById('pca9685-status');
                    const controlsDiv = document.getElementById('pca9685-controls');
                    const powerBar = document.getElementById('pca9685-power-bar');

                    // Count servo modules (exclude RTC 0x68 and EEPROM 0x57)
                    let servoCount = 0;
                    if (data.count > 0) {
                        for (let m = 0; m < data.count; m++) {
                            if (data.addresses[m] !== 0x68 && data.addresses[m] !== 0x57) {
                                servoCount++;
                            }
                        }
                    }
                    
                    // Update status message
                    if (servoCount > 0) {
                        statusDiv.innerHTML = '<span class="status">Found ' + servoCount + ' PCA9685 module(s)</span>';
                    } else {
                        statusDiv.innerHTML = '<span class="status">No PCA9685 modules found</span>';
                    }
                    controlsDiv.innerHTML = '';

                    const railsOn = data.power0 === true;
                    const isPowered = (data.power === true) || railsOn;

                    if (powerBar) {
                        powerBar.innerHTML = '<button type="button" class="btn-toggle-full" id="btn-gpio7" onclick="toggleGpioPower(7)" style="background-color: ' + (railsOn ? '#4CAF50' : '#f44336') + ';">Servo control: ' + (railsOn ? 'ON' : 'OFF') + '</button>';
                    }

                    // All Servos control (before modules)
                    if (servoCount > 0 && railsOn) {
                        const allServosDiv = document.createElement('div');
                        allServosDiv.style.marginTop = '15px';
                        allServosDiv.style.padding = '10px';
                        allServosDiv.style.backgroundColor = '#e3f2fd';
                        allServosDiv.style.borderRadius = '5px';
                        allServosDiv.style.border = '2px solid #2196F3';
                        allServosDiv.innerHTML = '<h3 style="margin-top: 0;">All Servos Control</h3>' +
                            '<p style="margin: 8px 0; font-size: 13px; color: #555;">Move ALL servos on powered boards to the same position</p>' +
                            '<div style="display: flex; align-items: center; gap: 10px; flex-wrap: wrap;">' +
                            '<button onclick="setAllServosAngle(0)" style="padding: 10px 18px; background-color: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold;">0°</button>' +
                            '<button onclick="setAllServosAngle(55)" style="padding: 10px 18px; background-color: #03A9F4; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold;">55°</button>' +
                            '<button onclick="setAllServosAngle(110)" style="padding: 10px 18px; background-color: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold;">110°</button>' +
                            '</div>' +
                            '<div style="margin-top: 12px;">' +
                            '<input id="all-servos-slider" type="range" min="0" max="110" value="0" oninput="setAllServosFromSlider(this.value)" style="width: 250px;">' +
                            '<span id="all-servos-value" style="margin-left: 10px; font-weight: bold; font-size: 16px;">0</span>° (0–110)' +
                            '</div>';
                        controlsDiv.appendChild(allServosDiv);
                    }
                    
                    // Show modules and servo controls
                    if (servoCount > 0) {
                        for (let m = 0; m < data.count; m++) {
                            // Skip RTC (0x68) and EEPROM (0x57) - they are not servo boards
                            if (data.addresses[m] === 0x68 || data.addresses[m] === 0x57) {
                                continue;
                            }
                            
                            const moduleDiv = document.createElement('div');
                            moduleDiv.style.marginTop = '15px';
                            moduleDiv.style.padding = '10px';
                            moduleDiv.style.backgroundColor = '#f0f0f0';
                            moduleDiv.style.borderRadius = '5px';
                            
                            const boardPowered = railsOn;
                            let html = '<h3>Module ' + m + ' (Address: 0x' + data.addresses[m].toString(16) + ')</h3>';
                            
                            // Only show servo controls if this board is powered on
                            if (boardPowered) {
                                html += '<p><strong>14 Servos Available (Channels 0-13)</strong></p>';
                                
                                // Show only first 14 channels (servos)
                                for (let ch = 0; ch < 14; ch++) {
                                    html += '<div style="display: inline-block; margin: 5px; padding: 10px; background: white; border-radius: 5px; min-width: 200px;">';
                                    html += '<label style="display: block; margin-bottom: 5px; font-weight: bold;">Servo ' + ch + ' (Ch' + ch + '):</label>';
                                    html += '<button onclick="setServoAngleFromButton(' + m + ', ' + ch + ', 0)" style="margin: 2px; padding: 8px 12px; background-color: #2196F3; color: white; border: none; border-radius: 3px; cursor: pointer; font-size: 14px;">0°</button>';
                                    html += '<button onclick="setServoAngleFromButton(' + m + ', ' + ch + ', 55)" style="margin: 2px; padding: 8px 12px; background-color: #03A9F4; color: white; border: none; border-radius: 3px; cursor: pointer; font-size: 14px;">55°</button>';
                                    html += '<button onclick="setServoAngleFromButton(' + m + ', ' + ch + ', 110)" style="margin: 2px; padding: 8px 12px; background-color: #4CAF50; color: white; border: none; border-radius: 3px; cursor: pointer; font-size: 14px;">110°</button>';
                                    html += '<div style="margin-top: 8px;">';
                                    html += '<input id="servo-slider-' + m + '-' + ch + '" type="range" min="0" max="110" value="0" oninput="setServoAngleFromSlider(' + m + ', ' + ch + ', this.value)" style="width: 160px;">';
                                    html += '<span id="servo-slider-value-' + m + '-' + ch + '" style="margin-left: 6px; font-weight: bold;">0</span> ° (0–110)';
                                    html += '</div>';
                                    html += '</div>';
                                }
                            } else {
                                html += '<p style="color: #666; font-style: italic;">Turn ON Board ' + m + ' above to control servos</p>';
                            }
                            
                            moduleDiv.innerHTML = html;
                            controlsDiv.appendChild(moduleDiv);
                        }
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    document.getElementById('pca9685-status').textContent = 'Error loading PCA9685 status';
                });
        }
        
        function toggleGpioPower(gpio) {
            var btn = document.getElementById('btn-gpio' + gpio);
            var isOn = btn && btn.textContent.trim().endsWith('ON');
            var newOn = !isOn;
            var state = newOn ? 'on' : 'off';
            fetch('/api/power?gpio=' + String(gpio) + '&state=' + state)
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        var b7 = document.getElementById('btn-gpio7');
                        var on = data.power0 === true;
                        if (b7) {
                            b7.textContent = on ? 'Servo control: ON' : 'Servo control: OFF';
                            b7.style.backgroundColor = on ? '#4CAF50' : '#f44336';
                        }
                        loadPCA9685Status();
                    } else {
                        alert('Failed to set power');
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Failed to set power');
                });
        }
        
        // Servo angle: OFF = 0°, ON = 110°
        const MAX_SERVO_ANGLE = 110;
        function angleToPWM(angle) {
            if (angle > MAX_SERVO_ANGLE) angle = MAX_SERVO_ANGLE;
            if (angle < 0) angle = 0;
            if (angle === 0) return 100;   // 0° = OFF
            if (angle === 110) return 375;   // 110° = ON
            return Math.round((angle / 110) * 275 + 100);  // 0–110° linear
        }
        
        function setServoAngle(module, channel, angle) {
            if (angle > MAX_SERVO_ANGLE) angle = MAX_SERVO_ANGLE;
            if (angle < 0) angle = 0;
            const pwmValue = angleToPWM(angle);
            fetch('/api/servo?module=' + module + '&channel=' + channel + '&value=' + pwmValue)
                .then(response => response.json())
                .then(data => {
                    // Success
                })
                .catch(error => {
                    console.error('Error:', error);
                });
        }

        function setServoAngleFromButton(module, channel, angle) {
            const slider = document.getElementById('servo-slider-' + module + '-' + channel);
            const label = document.getElementById('servo-slider-value-' + module + '-' + channel);
            if (slider) {
                slider.value = angle;
            }
            if (label) {
                label.textContent = angle;
            }
            setServoAngle(module, channel, angle);
        }

        function setServoAngleFromSlider(module, channel, angle) {
            const label = document.getElementById('servo-slider-value-' + module + '-' + channel);
            if (label) {
                label.textContent = angle;
            }
            setServoAngle(module, channel, angle);
        }

        function setAllServosAngle(angle) {
            if (angle > MAX_SERVO_ANGLE) angle = MAX_SERVO_ANGLE;
            if (angle < 0) angle = 0;
            const slider = document.getElementById('all-servos-slider');
            const label = document.getElementById('all-servos-value');
            if (slider) slider.value = angle;
            if (label) label.textContent = angle;
            for (let m = 0; m < 2; m++) {
                for (let ch = 0; ch < 14; ch++) {
                    setServoAngle(m, ch, angle);
                    const indSlider = document.getElementById('servo-slider-' + m + '-' + ch);
                    const indLabel = document.getElementById('servo-slider-value-' + m + '-' + ch);
                    if (indSlider) indSlider.value = angle;
                    if (indLabel) indLabel.textContent = angle;
                }
            }
        }

        function setAllServosFromSlider(angle) {
            const label = document.getElementById('all-servos-value');
            if (label) label.textContent = angle;
            for (let m = 0; m < 2; m++) {
                for (let ch = 0; ch < 14; ch++) {
                    setServoAngle(m, ch, angle);
                    const indSlider = document.getElementById('servo-slider-' + m + '-' + ch);
                    const indLabel = document.getElementById('servo-slider-value-' + m + '-' + ch);
                    if (indSlider) indSlider.value = angle;
                    if (indLabel) indLabel.textContent = angle;
                }
            }
        }

        function loadAutoUpdateStatus() {
            fetch('/api/auto-update')
                .then(response => response.json())
                .then(data => {
                    const buttonEl = document.getElementById('auto-update-button');
                    if (!buttonEl) return;
                    const isEnabled = data.enabled === true;
                    buttonEl.textContent = isEnabled ? 'Auto-update: ON' : 'Auto-update: OFF';
                    buttonEl.style.backgroundColor = isEnabled ? '#4CAF50' : '#f44336';
                })
                .catch(error => {
                    console.error('Error:', error);
                    const buttonEl = document.getElementById('auto-update-button');
                    if (buttonEl) {
                        buttonEl.textContent = 'Auto-update: ?';
                        buttonEl.style.backgroundColor = '#757575';
                    }
                });
        }

        function toggleAutoUpdate() {
            fetch('/api/auto-update')
                .then(response => response.json())
                .then(data => {
                    const newState = data.enabled === true ? 'off' : 'on';
                    return fetch('/api/auto-update?state=' + newState);
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        loadAutoUpdateStatus();
                    } else {
                        alert('Failed to toggle auto update');
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Failed to toggle auto update');
                });
        }

        function pad2(n) { return (n < 10 ? '0' : '') + n; }
        function updateWorkHoursHint(s, t) {
            var elS = document.getElementById('work-hours-hint-start');
            var elE = document.getElementById('work-hours-hint-end');
            if (elS) elS.textContent = pad2(s) + ':00';
            if (elE) elE.textContent = pad2(t) + ':59';
        }
        function loadWorkHours() {
            fetch('/api/work-hours')
                .then(function(r) { return r.json(); })
                .then(function(d) {
                    if (d.start !== undefined && d.stop !== undefined) {
                        document.getElementById('work-start').value = d.start;
                        document.getElementById('work-stop').value = d.stop;
                        updateWorkHoursHint(d.start, d.stop);
                    }
                })
                .catch(function(e) { console.error('work-hours load', e); });
        }
        function saveWorkHours() {
            var s = parseInt(document.getElementById('work-start').value, 10);
            var t = parseInt(document.getElementById('work-stop').value, 10);
            if (isNaN(s) || isNaN(t) || s < 0 || s > 23 || t < 0 || t > 23) {
                alert('Hours must be 0–23');
                return;
            }
            if (s >= t) {
                alert('Start hour must be less than stop hour.');
                return;
            }
            fetch('/api/work-hours?start=' + s + '&stop=' + t)
                .then(function(r) { return r.json().then(function(data) { return { ok: r.ok, data: data }; }); })
                .then(function(result) {
                    if (!result.ok || result.data.error) {
                        alert(result.data.error || 'Save failed');
                        return;
                    }
                    updateWorkHoursHint(s, t);
                    alert('Works hours saved');
                })
                .catch(function(e) { console.error(e); alert('Save failed'); });
        }
        
        loadPCA9685Status();
        loadAutoUpdateStatus();
        loadWorkHours();
        function updateTime() {
            fetch('/api/time')
                .then(response => response.text())
                .then(text => {
                    try {
                        var data = JSON.parse(text);
                        var timeEl = document.getElementById('rtc-time');
                        var statusEl = document.getElementById('rtc-status');
                        if (timeEl) timeEl.textContent = (data.time !== undefined) ? data.time : '--';
                        if (statusEl) statusEl.textContent = (data.status !== undefined) ? data.status : '--';
                    } catch (e) {
                        console.error('Parse error:', e);
                        var timeEl = document.getElementById('rtc-time');
                        if (timeEl) timeEl.textContent = 'Error loading time';
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    var timeEl = document.getElementById('rtc-time');
                    var statusEl = document.getElementById('rtc-status');
                    if (timeEl) timeEl.textContent = 'Error loading time';
                    if (statusEl) statusEl.textContent = 'Request failed';
                });
        }
        
        function syncTime() {
            var now = new Date();
            var syncUrl = '/api/sync?local=1&year=' + now.getFullYear() + '&month=' + (now.getMonth() + 1) + '&day=' + now.getDate()
                + '&hour=' + now.getHours() + '&minute=' + now.getMinutes() + '&second=' + now.getSeconds();
            fetch(syncUrl)
                .then(function(response) {
                    return response.text().then(function(text) {
                        try {
                            return { ok: response.ok, data: JSON.parse(text) };
                        } catch (e) {
                            return { ok: false, data: { error: text || 'Invalid response' } };
                        }
                    });
                })
                .then(function(result) {
                    if (result.ok) {
                        alert(result.data.message || 'Time synced');
                        updateTime();
                    } else {
                        alert(result.data.error || 'Sync failed');
                    }
                })
                .catch(function(error) {
                    console.error('Error:', error);
                    alert('Failed to sync time (check connection)');
                });
        }
        
        function testEEPROM() {
            fetch('/api/eeprom-test')
                .then(response => response.json())
                .then(data => {
                    alert(data.message);
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('EEPROM test failed');
                });
        }
        
        function setTimeDisplay(timeValue) {
            if (!timeValue) {
                alert('Please select a time');
                return;
            }
            const colon = timeValue.indexOf(':');
            if (colon < 0) {
                alert('Invalid time format');
                return;
            }
            const h = parseInt(timeValue.substring(0, colon), 10);
            const mi = parseInt(timeValue.substring(colon + 1), 10);
            if (isNaN(h) || isNaN(mi) || h < 0 || h > 23 || mi < 0 || mi > 59) {
                alert('Invalid time values');
                return;
            }
            const now = new Date();
            const syncUrl = '/api/sync?local=1&year=' + now.getFullYear() + '&month=' + (now.getMonth() + 1) + '&day=' + now.getDate()
                + '&hour=' + h + '&minute=' + mi + '&second=' + now.getSeconds();
            const displayUrl = '/api/set-time-display?time=' + encodeURIComponent(timeValue);
            fetch(syncUrl)
                .then(function(response) {
                    return response.json().then(function(data) {
                        return { ok: response.ok, data: data };
                    });
                })
                .then(function(result) {
                    if (!result.ok || (result.data && result.data.error)) {
                        throw new Error((result.data && result.data.error) ? result.data.error : 'RTC sync failed');
                    }
                    return fetch(displayUrl);
                })
                .then(function(response) { return response.json(); })
                .then(function(data) {
                    if (!data.success) throw new Error('Failed to set 7-segment display');
                    return fetch('/api/auto-update?state=on');
                })
                .then(function(response) { return response.json(); })
                .then(function() {
                    loadAutoUpdateStatus();
                    updateTime();
                })
                .catch(function(err) {
                    console.error('setTimeDisplay:', err);
                    alert(err.message || 'Error');
                });
        }
        
        setInterval(updateTime, 30000);  // Poll every 30s to reduce I2C/WiFi conflict
        updateTime();
    </script>
</body>
</html>
)rawliteral";

// 7-Segment Display Patterns
// Each digit (0-9) maps to 7 servo positions [segment0, segment1, segment2, segment3, segment4, segment5, segment6]
// 0 = OFF (0°), 1 = ON (110°)
const byte digitPatterns[10][7] = {
  {0,1,1,1,1,1,1},  // 0
  {0,1,0,0,0,0,1},  // 1
  {1,0,1,1,0,1,1},  // 2
  {1,1,1,0,0,1,1},  // 3
  {1,1,0,0,1,0,1},  // 4
  {1,1,1,0,1,1,0},  // 5
  {1,1,1,1,1,1,0},  // 6
  {0,1,0,0,0,1,1},  // 7
  {1,1,1,1,1,1,1},  // 8
  {1,1,1,0,1,1,1}   // 9
};

// Servo PWM values: OFF = 0°, ON = 110°
#define SERVO_POS_0 100   // 0° = OFF (segment off)
#define SERVO_POS_1 375   // 110° = ON (segment on)
#define SERVO_MAX_PWM 375 // Cap at 110° for all servo moves

// Function prototypes
void handleRoot();
void handleNotFound();
void handleAPITime();
void handleAPISync();
void handleAPIEEPROMTest();
void handleAPIWorkHours();
void handleAPIPCA9685Status();
void handleAPIServo();
void handleAPIPowerControl();
void handleAPISetTimeDisplay();
void handleAPITimeDisplayDelays();
void handleAPIAutoUpdate();
void setDigitOnBoard(int boardIndex, int startChannel, int digit);
void setTimeDisplay(int hours, int minutes);
void reinitI2C();
void setupWiFiAP();
void setupI2C();
void setupRTC();
void setupDS3231SQW();
void setupEEPROM();
void loadWorkHoursFromEeprom();
void setupPCA9685();
void setupPCA9685PowerControl();
void turnOnPCA9685();
void turnOffPCA9685();
void turnOnBoard(int boardIndex);
void turnOffBoard(int boardIndex);
bool isBoardPowered(int boardIndex);
bool isPCA9685Powered();
static void setPca9685RailsPower(bool on);
static void cancelRailsAutoOff(void);
static void armRailsAutoOff(uint32_t holdMs);
void enterDeepSleep();
void updateTimeDisplayAuto();
static bool isRTCTimeValid(const DateTime& now);
static bool readRTCWithRetry(DateTime& outNow, int maxAttempts, bool trySlowI2C);

void setup() {
  // USB Serial (CDC) - ESP32-C3 native USB: open Serial Monitor at 115200, then reset board
  Serial.begin(115200);
  // Wait for USB host to open the port (up to 5 sec), then extra delay
  for (int i = 0; i < 500; i++) {
    delay(10);
    if (Serial) break;
  }
  delay(500);
  Serial.print("\n\n\n");
  Serial.println("========================================");
  Serial.println(">>> SERIAL INITIALIZED - CHECK OK <<<");
  Serial.println("========================================");
  Serial.println("Baud: 115200");
  Serial.println("Starting system initialization...");
  Serial.flush();
  delay(100);
  
  Serial.println("\n========================================");
  Serial.println("=== ESP32-C3 Initialization ===");
  Serial.println("========================================");
  Serial.println("Serial communication: OK");
  Serial.printf("Baud rate: %d\n", 115200);
  
  setupPCA9685PowerControl();

  setupI2C();
  
  // Initialize RTC
  setupRTC();
  
  // Initialize EEPROM
  setupEEPROM();
  
  // Initialize PCA9685
  setupPCA9685();
  
  // Setup WiFi AP
  setupWiFiAP();
  
  // Setup Web Server
  server.on("/", handleRoot);
  server.on("/api/time", handleAPITime);
  server.on("/api/sync", handleAPISync);
  server.on("/api/eeprom-test", handleAPIEEPROMTest);
  server.on("/api/work-hours", handleAPIWorkHours);
  server.on("/api/pca9685-status", handleAPIPCA9685Status);
  server.on("/api/servo", handleAPIServo);
  server.on("/api/power", handleAPIPowerControl);
  server.on("/api/set-time-display", handleAPISetTimeDisplay);
  server.on("/api/time-display-delays", handleAPITimeDisplayDelays);
  server.on("/api/auto-update", handleAPIAutoUpdate);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("HTTP server started");
  
  Serial.println("\n========================================");
  Serial.println("=== SYSTEM STARTED SUCCESSFULLY ===");
  Serial.println("========================================");
  Serial.println("WiFi AP SSID: " + String(ssid));
  Serial.println("WiFi Password: " + String(password));
  Serial.println("Web Interface: http://192.168.4.1");
  Serial.println("Serial Monitor: OK (115200 baud)");
  Serial.println("RTC Module: Connected");
  Serial.println("EEPROM Module: Ready");
  if (pca9685Available) {
    Serial.printf("PCA9685 Module(s): %d connected\n", pca9685Count);
  } else {
    Serial.println("PCA9685 Module(s): Not found");
  }
  Serial.println("========================================");
  Serial.println(">>> SYSTEM READY - ALL SYSTEMS OK <<<");
  Serial.println("========================================\n");
  Serial.println("Serial commands: time(t) | status(s)");
  Serial.flush();
}

void loop() {
  server.handleClient();

  if (pca9685RailsAutoOffAt != 0 && (long)(millis() - pca9685RailsAutoOffAt) >= 0) {
    Serial.printf("[GPIO7] auto-off: hold done (%lu ms) -> rails OFF (GPIO HIGH)\n",
                  (unsigned long)PCA9685_RAILS_HOLD_MS);
    turnOffPCA9685();
  }

  // Once 8s after boot: remind user of serial commands (in case they opened monitor late)
  static bool serialPromptDone = false;
  if (!serialPromptDone && millis() > 8000) {
    Serial.println("\n>> Serial ready. Type s for status, t for time.");
    serialPromptDone = true;
  }
  
  // Serial commands (type and press Enter)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    // Echo immediately so you see something even if TX was buffered
    Serial.print(">> ");
    Serial.println(cmd.length() ? cmd : "(empty)");
    Serial.flush();
    if (cmd == "time" || cmd == "t" || cmd == "rtc") {
      if (!rtcAvailable) {
        Serial.println("[RTC] Not available");
      } else {
        DateTime now;
        bool ok = readRTCWithRetry(now, 5, true);
        if (ok && isRTCTimeValid(now)) {
          Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
                        now.year(), now.month(), now.day(),
                        now.hour(), now.minute(), now.second());
        } else {
          Serial.println("[RTC] Read failed (I2C error)");
        }
      }
      Serial.flush();
    } else if (cmd == "status" || cmd == "s") {
      Serial.println("\n--- System Status ---");
      Serial.println("Web: http://192.168.4.1  |  SSID: " + String(ssid));
      Serial.println("RTC: " + String(rtcAvailable ? "OK" : "Not found"));
      if (rtcAvailable) {
        DateTime now;
        if (readRTCWithRetry(now, 3, true) && isRTCTimeValid(now)) {
          Serial.printf("Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                        now.year(), now.month(), now.day(),
                        now.hour(), now.minute(), now.second());
        } else {
          Serial.println("Time: (read failed)");
        }
      }
      Serial.printf("PCA9685: %d module(s)", pca9685Count);
      if (pca9685Count > 0) {
        Serial.printf(" | PCA rails (GPIO%d): %s\n", PCA9685_POWER_GPIO,
                      pca9685RailsPowered ? "ON" : "OFF");
      } else {
        Serial.println();
      }
      Serial.println("Commands: time(t)  status(s)");
      Serial.println("---------------------------\n");
      Serial.flush();
    }
  }
  
  updateTimeDisplayAuto();
  
  delay(10);
}

void setupI2C() {
  Serial.println("\n--- I2C Setup ---");
  Wire.begin(I2C_SDA, I2C_SCL);
  // 100kHz: more reliable with DS3231 on ESP32 (avoids Wire Error 263 / timeout)
  Wire.setClock(100000);
  Wire.setTimeOut(2000);  // 2s I2C timeout (default can be too short for DS3231)
  Serial.printf("I2C initialized: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Serial.println("I2C Clock: 100kHz (for DS3231 stability)");
  Serial.println("I2C Timeout: 2000ms");
  Serial.println("I2C Status: OK");
  Serial.println("\nNOTE: I2C requires pull-up resistors (typically 4.7kΩ)");
  Serial.println("      Many modules have built-in pull-ups");
  
  // Scan I2C bus with detailed error reporting
  Serial.println("\nScanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  
  // First, check for expected devices at known addresses
  Serial.println("Checking expected devices (main I2C = GPIO5/6):");
  Serial.println("  PCA9685 Servo Driver: 0x40-0x7F (64-127 decimal)");
  Serial.println("  (DS3231 module: RTC 0x68 + EEPROM 0x57 on GPIO9/10)");
  Serial.println("");
  
  // Scan for PCA9685 modules (addresses 0x40-0x7F)
  Serial.println("Scanning for PCA9685 modules (0x40-0x7F):");
  int pca9685Found = 0;
  for (address = 0x40; address <= 0x7F; address++) {
    // Skip RTC (0x68) and EEPROM (0x57) - they are not PCA9685 modules
    if (address == 0x57 || address == 0x68) {
      continue;
    }
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  [FOUND] PCA9685 at address 0x%02X\n", address);
      pca9685Found++;
      nDevices++;
    }
  }
  if (pca9685Found == 0) {
    Serial.println("  [NOT FOUND] No PCA9685 modules detected");
  } else {
    Serial.printf("  Found %d PCA9685 module(s)\n", pca9685Found);
  }
  
  // Full scan of main bus (RTC 0x68 + EEPROM 0x57 + PCA9685 on GPIO 5/6)
  Serial.println("\nFull I2C bus scan (main bus, addresses 1-126):");
  int foundCount = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  Device found at address 0x%02X (%d decimal)\n", address, address);
      foundCount++;
    } else if (error == 4) {
      Serial.printf("  Unknown error at address 0x%02X\n", address);
    }
  }
  
  if (foundCount == 0) {
    Serial.println("\n[RESULT] No I2C devices found!");
    Serial.println("\nTroubleshooting tips:");
    Serial.println("  1. Check wiring connections:");
    Serial.printf("     - SDA -> GPIO%d\n", I2C_SDA);
    Serial.printf("     - SCL -> GPIO%d\n", I2C_SCL);
    Serial.println("     - VCC -> 3.3V (or 5V if module supports it)");
    Serial.println("     - GND -> GND");
    Serial.println("  2. Verify module has power (LED on module if present)");
    Serial.println("  3. Check if module has pull-up resistors");
    Serial.println("  4. Try different GPIO pins if available");
    Serial.println("  5. Verify module is not damaged");
    Serial.println("  6. AT24C32 works at 5V - try 5V power if module supports it");
  } else {
    Serial.printf("\n[RESULT] Found %d device(s) on I2C bus\n", foundCount);
  }
  
  nDevices = foundCount;
}

void setupRTC() {
  Serial.println("\n--- RTC Setup (main I2C: GPIO5=SDA, GPIO6=SCL) ---");
  delay(50);

  Serial.println("Attempting to connect to DS3231 RTC...");
  if (!rtc.begin(&Wire)) {
    Serial.println("[WARNING] Couldn't find RTC (DS3231)!");
    Serial.println("[WARNING] System will continue without RTC");
    Serial.println("Check RTC I2C wiring (same bus as servos):");
    Serial.printf("  SDA -> GPIO%d\n", I2C_SDA);
    Serial.printf("  SCL -> GPIO%d\n", I2C_SCL);
    Serial.println("  VCC -> 3.3V, GND -> GND");
    Serial.println("[INFO] Continuing initialization...");
    rtcAvailable = false;
    return;
  }
  
  rtcAvailable = true;
  Serial.println("DS3231 RTC found!");
  Serial.println("RTC Status: OK");
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  byte eepErr = Wire.endTransmission();
  if (eepErr == 0) Serial.println("AT24C32 EEPROM found on same module (0x57)");
  
  // Check if RTC lost power or has invalid time (use retry so one bad I2C read doesn't trigger reset)
  bool needsTimeSet = false;
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, will set default time");
    needsTimeSet = true;
  } else {
    DateTime now;
    if (!readRTCWithRetry(now, 5, true) || !isRTCTimeValid(now)) {
      Serial.println("[WARNING] RTC read failed or invalid time, will set default time");
      needsTimeSet = true;
    } else {
      Serial.printf("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
                    now.year(), now.month(), now.day(),
                    now.hour(), now.minute(), now.second());
      snprintf(rtcTimeCache, sizeof(rtcTimeCache), "%04d-%02d-%02d %02d:%02d:%02d",
               now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
  }
  
  if (needsTimeSet) {
    Serial.println("Setting RTC to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(300);
    DateTime verify;
    if (readRTCWithRetry(verify, 5, true) && isRTCTimeValid(verify)) {
      Serial.printf("RTC time set to: %04d-%02d-%02d %02d:%02d:%02d\n",
                    verify.year(), verify.month(), verify.day(),
                    verify.hour(), verify.minute(), verify.second());
      snprintf(rtcTimeCache, sizeof(rtcTimeCache), "%04d-%02d-%02d %02d:%02d:%02d",
               verify.year(), verify.month(), verify.day(), verify.hour(), verify.minute(), verify.second());
    } else {
      Serial.println("[INFO] RTC set; use 'Sync with Browser' in web interface for accurate time.");
    }
  }
  
  setupDS3231SQW();
}

void setupDS3231SQW() {
  if (!rtcAvailable) return;
  Serial.println("\n--- DS3231 Alarm Setup ---");
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(DS3231_ALARM2_MINUTES);
  Wire.write(0x80);
  Wire.endTransmission();
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(DS3231_ALARM2_HOURS);
  Wire.write(0x80);
  Wire.endTransmission();
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(DS3231_ALARM2_DAYDATE);
  Wire.write(0x80);
  Wire.endTransmission();
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(DS3231_CONTROL_REG);
  Wire.write(0x06);
  byte err = Wire.endTransmission();
  if (err == 0) Serial.println("DS3231 Alarm 2 configured");
  else Serial.printf("DS3231 Alarm config failed (Error: %d)\n", err);
}

void enterDeepSleep() {
  if (rtcAvailable) {
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(DS3231_STATUS_REG);
    Wire.write(0x00);
    Wire.endTransmission();
  }
  Serial.println("\n--- Entering Deep Sleep ---");
  Serial.flush();
  esp_deep_sleep_start();
}

static bool at24c32ReadByte(uint16_t memAddr, uint8_t* out) {
  if (!out) return false;
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(memAddr >> 8));
  Wire.write((int)(memAddr & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  uint8_t n = Wire.requestFrom((int)EEPROM_I2C_ADDRESS, 1);
  if (n != 1 || !Wire.available()) return false;
  *out = Wire.read();
  return true;
}

static bool at24c32WriteByte(uint16_t memAddr, uint8_t data) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(memAddr >> 8));
  Wire.write((int)(memAddr & 0xFF));
  Wire.write(data);
  return Wire.endTransmission() == 0;
}

static bool workHoursRangeOk(uint8_t s, uint8_t t) {
  return s <= 23 && t <= 23 && s < t;
}

void loadWorkHoursFromEeprom() {
  uint8_t s = 0, t = 0;
  if (at24c32ReadByte(EEPROM_ADDR_WORK_START, &s) &&
      at24c32ReadByte(EEPROM_ADDR_WORK_STOP, &t) &&
      workHoursRangeOk(s, t)) {
    workHoursStartHour = s;
    workHoursStopHour = t;
    Serial.printf("Works hours (EEPROM): %02d:00 – %02d:59\n", s, t);
  } else {
    workHoursStartHour = 9;
    workHoursStopHour = 22;
    Serial.println("Works hours: default 09–22 (EEPROM empty or invalid)");
  }
}

static bool saveWorkHoursToEeprom(uint8_t s, uint8_t t) {
  if (!workHoursRangeOk(s, t)) return false;
  if (!at24c32WriteByte(EEPROM_ADDR_WORK_START, s)) return false;
  delay(10);
  if (!at24c32WriteByte(EEPROM_ADDR_WORK_STOP, t)) return false;
  delay(10);
  workHoursStartHour = s;
  workHoursStopHour = t;
  return true;
}

void setupEEPROM() {
  Serial.println("\n--- EEPROM Setup ---");
  Serial.println("AT24C32 EEPROM on DS3231 module (I2C 0x57, main bus)");
  Serial.println("Size: 4KB (4096 bytes)");
  Serial.println("EEPROM Status: Ready");
  loadWorkHoursFromEeprom();
}

void setupPCA9685PowerControl() {
  Serial.println("\n--- PCA9685 Power Control Setup ---");
  pinMode(PCA9685_POWER_GPIO, OUTPUT);
  digitalWrite(PCA9685_POWER_GPIO, HIGH);  // active-LOW enable: HIGH = rails off
  pca9685RailsPowered = false;
  pca9685RailsAutoOffAt = 0;

  Serial.println("[GPIO7] boot: drive HIGH (rails off). Logic: LOW=ON | HIGH=OFF");
  Serial.printf("Power: GPIO%d — both boards. HW: if pin floats low at reset, add ~10k GPIO7 to 3.3V\n",
                PCA9685_POWER_GPIO);
  Serial.println("Power Control Status: OK");
}

static void cancelRailsAutoOff() {
  pca9685RailsAutoOffAt = 0;
}

static void armRailsAutoOff(uint32_t holdMs) {
  pca9685RailsAutoOffAt = millis() + holdMs;
  Serial.printf("[GPIO7] auto-off armed: ~%lu ms GPIO LOW (on) then HIGH (off)\n", (unsigned long)holdMs);
}

static void setPca9685RailsPower(bool on) {
  const int pinLevel = on ? LOW : HIGH;  // active-LOW: LOW = rail on, HIGH = rail off
  if (!on) cancelRailsAutoOff();

  if (pca9685RailsPowered != on) {
    Serial.printf("[GPIO7] rails %s -> write GPIO%d %s\n", on ? "ON" : "OFF", PCA9685_POWER_GPIO,
                  pinLevel == HIGH ? "HIGH" : "LOW");
  }
  digitalWrite(PCA9685_POWER_GPIO, pinLevel);
  bool risingEdge = on && !pca9685RailsPowered;
  pca9685RailsPowered = on;
  if (risingEdge) delay(120);
}

// Web API: only GPIO 7 is valid (controls both boards)
static void setGpioPower(int gpio, bool on) {
  if (gpio != PCA9685_POWER_GPIO) return;
  cancelRailsAutoOff();
  Serial.printf("[GPIO7] manual web/API: %s (auto timer cleared)\n", on ? "ON (GPIO LOW)" : "OFF (GPIO HIGH)");
  setPca9685RailsPower(on);
}

void turnOnPCA9685() {
  setPca9685RailsPower(true);
}

void turnOffPCA9685() {
  setPca9685RailsPower(false);  // clears timer; drives GPIO HIGH = rail off
}

void turnOnBoard(int boardIndex) {
  if (boardIndex >= 0 && boardIndex <= 1) setPca9685RailsPower(true);
}

void turnOffBoard(int boardIndex) {
  (void)boardIndex;
  setPca9685RailsPower(false);
}

bool isBoardPowered(int boardIndex) {
  (void)boardIndex;
  return pca9685RailsPowered;
}

bool isPCA9685Powered() {
  return pca9685RailsPowered;
}

void setupPCA9685() {
  Serial.println("\n--- PCA9685 Setup ---");
  pca9685Count = 0;

  Serial.println("Powering PCA9685 rails briefly for I2C scan...");
  turnOnPCA9685();
  delay(PCA9685_RAILS_PRE_MS);  // let boards & logic rise before ACK

  Serial.println("Scanning for PCA9685 modules...");
  
  // Scan for PCA9685 modules (addresses 0x40-0x7F)
  // PCA9685 base address is 0x40, can be changed with address jumpers
  for (byte addr = 0x40; addr <= 0x7F && pca9685Count < MAX_PCA9685_MODULES; addr++) {
    // Skip RTC (0x68) and EEPROM (0x57) - they are not PCA9685 modules
    if (addr == 0x57 || addr == 0x68) {
      continue;
    }
    
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      // Try to initialize PCA9685 at this address
      pca9685[pca9685Count] = new Adafruit_PWMServoDriver(addr);
      
      // begin() returns void, so we just call it
      pca9685[pca9685Count]->begin();
      pca9685[pca9685Count]->setOscillatorFrequency(27000000);  // 27MHz typical
      pca9685[pca9685Count]->setPWMFreq(50);  // 50Hz for servos
      pca9685Addresses[pca9685Count] = addr;  // Store the actual address
      Serial.printf("  [INITIALIZED] PCA9685 #%d at address 0x%02X\n", pca9685Count, addr);
      pca9685Count++;
      pca9685Available = true;
    }
  }
  
  if (pca9685Count > 0) {
    Serial.printf("PCA9685 Status: OK - %d module(s) initialized\n", pca9685Count);
    Serial.println("PWM Frequency: 50Hz (for servos)");
    Serial.println("Channels per module: 16");
    Serial.printf("Servos per board: %d\n", SERVOS_PER_BOARD);
    Serial.printf("Total channels available: %d\n", pca9685Count * 16);
  } else {
    Serial.println("PCA9685 Status: No modules found");
    Serial.println("[WARNING] Make sure boards are powered on and I2C addresses are correct");
    pca9685Available = false;
  }
  
  Serial.println("PCA9685 scan done — rails OFF (GPIO HIGH) until servos/time display need power");
  turnOffPCA9685();
}

void setupWiFiAP() {
  Serial.println("\n--- WiFi AP Setup ---");
  
  // Set WiFi mode to AP
  if (WiFi.mode(WIFI_AP) == false) {
    Serial.println("[ERROR] Failed to set WiFi mode to AP");
    return;
  }
  Serial.println("WiFi mode set to AP");
  
  // Start Access Point
  bool apStarted = WiFi.softAP(ssid, password);
  if (!apStarted) {
    Serial.println("[ERROR] Failed to start Access Point!");
    Serial.println("Trying again in 1 second...");
    delay(1000);
    apStarted = WiFi.softAP(ssid, password);
    if (!apStarted) {
      Serial.println("[ERROR] Failed to start Access Point after retry!");
      return;
    }
  }
  
  // Give AP time to start
  delay(500);
  
  IPAddress IP = WiFi.softAPIP();
  if (IP.toString() == "0.0.0.0") {
    Serial.println("[ERROR] AP started but IP address is invalid!");
    Serial.println("Waiting 2 seconds and retrying...");
    delay(2000);
    IP = WiFi.softAPIP();
  }
  
  Serial.printf("AP IP address: %s\n", IP.toString().c_str());
  Serial.printf("AP SSID: %s\n", ssid);
  Serial.printf("AP Password: %s\n", password);
  Serial.printf("AP MAC address: %s\n", WiFi.softAPmacAddress().c_str());
  Serial.println("WiFi AP Status: OK");
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Validate RTC time (catches I2C glitches like month=0, minute=96)
static bool isRTCTimeValid(const DateTime& now) {
  int y = now.year(), m = now.month(), d = now.day();
  int h = now.hour(), mi = now.minute(), s = now.second();
  return (y >= 2000 && y <= 2100) &&
         (m >= 1 && m <= 12) &&
         (d >= 1 && d <= 31) &&
         (h >= 0 && h <= 23) &&
         (mi >= 0 && mi <= 59) &&
         (s >= 0 && s <= 59);
}

// Reinit main I2C bus and RTC (same bus as PCA9685).
static void reinitI2CForRTC() {
  Wire.end();
  delay(50);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(3000);
  delay(50);
  rtc.begin(&Wire);
  delay(30);
}

// Read RTC on main I2C (GPIO 5/6, same bus as servos).
static bool readRTCWithRetry(DateTime& outNow, int maxAttempts, bool trySlowI2C) {
  delay(50);
  bool ok = false;
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    reinitI2CForRTC();
    delay(30);
    outNow = rtc.now();
    if (isRTCTimeValid(outNow)) {
      ok = true;
      break;
    }
    delay(attempt <= 2 ? 200 : 300);
  }
  return ok;
}

void handleAPITime() {
  String json = "{";
  json += "\"time\":\"";
  
  if (rtcAvailable) {
    DateTime now;
    bool readSuccess = readRTCWithRetry(now, 5, true);
    
    if (readSuccess) {
      int year = now.year(), month = now.month(), day = now.day();
      int hour = now.hour(), minute = now.minute(), second = now.second();
      char timeStr[24];
      sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
              year, month, day, hour, minute, second);
      strncpy(rtcTimeCache, timeStr, sizeof(rtcTimeCache) - 1);
      rtcTimeCache[sizeof(rtcTimeCache) - 1] = '\0';
      json += String(timeStr);
      json += "\",\"status\":\"Running\"";
    } else {
      if (rtcTimeCache[0] != '\0') {
        json += String(rtcTimeCache);
        json += "\",\"status\":\"Running (cached)\"";
      } else {
        json += "Invalid - Sync to fix\",\"status\":\"Invalid - click Sync with Browser\"";
      }
      static unsigned long lastRTCInvalidLog = 0;
      if (millis() - lastRTCInvalidLog > 60000) {
        Serial.println("[RTC] Invalid time read (I2C). Use 'Sync with Browser' to set time.");
        lastRTCInvalidLog = millis();
      }
    }
  } else {
    json += "RTC Not Connected\",\"status\":\"Not Available\"";
  }
  
  json += "}";
  server.send(200, "application/json", json);
}

void handleAPISync() {
  if (!rtcAvailable) {
    server.send(400, "application/json", "{\"error\":\"RTC not available\"}");
    return;
  }

  DateTime dt(2000, 1, 1, 0, 0, 0);
  bool useLocalWall = false;

  if (server.hasArg("local") && server.arg("local") == "1" &&
      server.hasArg("year") && server.hasArg("month") && server.hasArg("day") &&
      server.hasArg("hour") && server.hasArg("minute") && server.hasArg("second")) {
    int y = server.arg("year").toInt();
    int mo = server.arg("month").toInt();
    int d = server.arg("day").toInt();
    int h = server.arg("hour").toInt();
    int mi = server.arg("minute").toInt();
    int s = server.arg("second").toInt();
    if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) {
      server.send(400, "application/json", "{\"error\":\"Invalid local date/time fields\"}");
      return;
    }
    dt = DateTime((uint16_t)y, (uint8_t)mo, (uint8_t)d, (uint8_t)h, (uint8_t)mi, (uint8_t)s);
    useLocalWall = true;
    Serial.printf("[RTC] Sync (browser local wall): %04d-%02d-%02d %02d:%02d:%02d\n",
                  y, mo, d, h, mi, s);
  } else if (server.hasArg("timestamp")) {
    unsigned long timestamp = server.arg("timestamp").toInt();
    dt = DateTime(timestamp);
    Serial.println("[RTC] Sync (Unix UTC) — prefer \"Sync\" without DST skew: use browser local sync");
  } else {
    server.send(400, "application/json", "{\"error\":\"Use local=1&year&month&day&hour&minute&second or timestamp\"}");
    return;
  }

  bool pcaWasOn = (pca9685Available && pca9685Count > 0 && isPCA9685Powered());
  if (pcaWasOn) {
    turnOffPCA9685();
    delay(50);
  }

  rtc.adjust(dt);
  delay(300);

  DateTime verify;
  bool readOk = readRTCWithRetry(verify, 5, true);
  bool timeValid = readOk && isRTCTimeValid(verify);
  bool timeMatches = false;
  if (timeValid) {
    if (useLocalWall) {
      timeMatches = (verify.year() == dt.year() && verify.month() == dt.month() &&
                     verify.day() == dt.day() && verify.hour() == dt.hour() &&
                     verify.minute() == dt.minute() && abs((int)verify.second() - (int)dt.second()) <= 2);
    } else {
      timeMatches = (abs((long)(verify.unixtime() - dt.unixtime())) <= 15);
    }
  }

  if (pcaWasOn) turnOnPCA9685();

  if (timeValid && timeMatches) {
    rtcInvalidCount = 0;
    lastDisplayedMinute = -1;
    lastDisplayedHour = -1;
    String json = "{\"message\":\"Time synchronized successfully\",\"time\":\"";
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            verify.year(), verify.month(), verify.day(),
            verify.hour(), verify.minute(), verify.second());
    json += String(timeStr);
    json += "\"}";
    server.send(200, "application/json", json);
    Serial.printf("Time synchronized: %s\n", timeStr);
  } else {
    String json = "{\"message\":\"Time written to RTC. Click Refresh Time to confirm.\",\"time\":\"";
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    json += String(timeStr);
    json += "\"}";
    server.send(200, "application/json", json);
    Serial.printf("Sync: time written %s; verify read %s\n", timeStr, readOk ? "ok" : "failed");
  }
}

void handleAPIWorkHours() {
  bool hasS = server.hasArg("start");
  bool hasT = server.hasArg("stop");
  if (hasS != hasT) {
    server.send(400, "application/json", "{\"error\":\"Provide both start and stop (0–23)\"}");
    return;
  }
  if (hasS && hasT) {
    int s = server.arg("start").toInt();
    int t = server.arg("stop").toInt();
    if (s < 0 || s > 23 || t < 0 || t > 23) {
      server.send(400, "application/json", "{\"error\":\"Hours must be 0–23\"}");
      return;
    }
    if (s >= t) {
      server.send(400, "application/json", "{\"error\":\"Start hour must be less than stop hour\"}");
      return;
    }
    if (!saveWorkHoursToEeprom((uint8_t)s, (uint8_t)t)) {
      server.send(500, "application/json", "{\"error\":\"EEPROM write failed\"}");
      return;
    }
    String json = "{\"success\":true,\"start\":" + String((int)workHoursStartHour) +
                  ",\"stop\":" + String((int)workHoursStopHour) + "}";
    server.send(200, "application/json", json);
    Serial.printf("Works hours saved: %02d – %02d\n", workHoursStartHour, workHoursStopHour);
    return;
  }

  String json = "{\"start\":" + String((int)workHoursStartHour) +
                ",\"stop\":" + String((int)workHoursStopHour) + "}";
  server.send(200, "application/json", json);
}

void handleAPIEEPROMTest() {
  // Test AT24C32 EEPROM write/read
  byte testAddress = 0x00;
  byte testData = 0xAA;
  byte readData = 0;
  
  // EEPROM on DS3231 module (main I2C 0x57)
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(testAddress >> 8));
  Wire.write((int)(testAddress & 0xFF));
  Wire.write(testData);
  byte error = Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(testAddress >> 8));
  Wire.write((int)(testAddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
  if (Wire.available()) {
    readData = Wire.read();
  }
  
  String json;
  if (error == 0 && readData == testData) {
    json = "{\"message\":\"EEPROM test successful! Wrote 0x" + 
           String(testData, HEX) + ", Read 0x" + String(readData, HEX) + "\"}";
    server.send(200, "application/json", json);
    Serial.println("EEPROM test: PASSED");
  } else {
    json = "{\"message\":\"EEPROM test failed. Error: " + String(error) + 
           ", Expected: 0x" + String(testData, HEX) + ", Got: 0x" + String(readData, HEX) + "\"}";
    server.send(500, "application/json", json);
    Serial.println("EEPROM test: FAILED");
  }
}

void handleAPIPCA9685Status() {
  String json = "{\"count\":" + String(pca9685Count) + ",\"addresses\":[";
  
  for (int i = 0; i < pca9685Count; i++) {
    if (i > 0) json += ",";
    json += String(pca9685Addresses[i]);
  }
  
  json += "],\"power\":" + String(isPCA9685Powered() ? "true" : "false");
  json += ",\"power0\":" + String(pca9685RailsPowered ? "true" : "false");
  json += ",\"power1\":" + String(pca9685RailsPowered ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleAPIServo() {
  if (!pca9685Available || pca9685Count == 0) {
    server.send(400, "application/json", "{\"error\":\"PCA9685 not available\"}");
    return;
  }
  
  if (server.hasArg("module") && server.hasArg("channel") && server.hasArg("value")) {
    int module = server.arg("module").toInt();
    int channel = server.arg("channel").toInt();
    int value = server.arg("value").toInt();
    
    if (module >= 0 && module < pca9685Count && channel >= 0 && channel < 16) {
      bool openedRail = !isBoardPowered(module);
      if (openedRail) {
        turnOnBoard(module);
        delay(PCA9685_RAILS_PRE_MS);
      }

      if (value < 0) value = 0;
      if (value > SERVO_MAX_PWM) value = SERVO_MAX_PWM;
      
      pca9685[module]->setPWM(channel, 0, value);

      if (openedRail) armRailsAutoOff(PCA9685_RAILS_HOLD_MS);

      String json = "{\"success\":true,\"module\":" + String(module) +
                    ",\"channel\":" + String(channel) +
                    ",\"value\":" + String(value)  + 
                    ",\"powered\":" + String(isPCA9685Powered() ? "true" : "false") + "}";
      server.send(200, "application/json", json);

      Serial.printf("Servo: Module %d, Channel %d, Value %d\n", module, channel, value);
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid module or channel\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
  }
}

void handleAPIPowerControl() {
  // GPIO 7 only — switches both PCA9685 boards
  if (server.hasArg("gpio") && server.hasArg("state")) {
    String gpioStr = server.arg("gpio");
    String state = server.arg("state");
    if (gpioStr != "7") {
      server.send(400, "application/json", "{\"error\":\"Invalid gpio. Use 7 (powers both boards)\"}");
      return;
    }
    bool on = (state == "on");
    if (state != "on" && state != "off") {
      server.send(400, "application/json", "{\"error\":\"Invalid state. Use on or off\"}");
      return;
    }
    setGpioPower(7, on);
    String json = "{\"success\":true,\"gpio\":7,\"state\":\"" + state + "\"";
    json += ",\"power0\":" + String(pca9685RailsPowered ? "true" : "false");
    json += ",\"power1\":" + String(pca9685RailsPowered ? "true" : "false") + "}";
    server.send(200, "application/json", json);
    return;
  }
  if (server.hasArg("status")) {
    String json = "{\"powered\":" + String(isPCA9685Powered() ? "true" : "false");
    json += ",\"power0\":" + String(pca9685RailsPowered ? "true" : "false");
    json += ",\"power1\":" + String(pca9685RailsPowered ? "true" : "false") + "}";
    server.send(200, "application/json", json);
    return;
  }
  server.send(400, "application/json", "{\"error\":\"Missing parameters. Use gpio=7 and state=on|off\"}");
}

void reinitI2C() {
  Serial.println("\n--- I2C Reinitialization ---");
  
  // End current I2C communication
  Wire.end();
  delay(50);
  
  // Reinitialize I2C with same settings
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(2000);
  Serial.printf("I2C reinitialized: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Serial.println("I2C Clock: 100kHz");
  
  if (rtcAvailable) {
    if (rtc.begin(&Wire)) {
      rtcAvailable = true;
      Serial.println("DS3231 RTC reinitialized (main I2C)");
    } else {
      rtcAvailable = false;
      Serial.println("DS3231 RTC reinitialization failed");
    }
  }
  
  // Reinitialize PCA9685 modules if any board was powered
  if (pca9685Available && pca9685Count > 0 && isPCA9685Powered()) {
    bool railsWereOn = pca9685RailsPowered;
    turnOnPCA9685();  // Rails on so we can reinit both modules
    delay(100);
    for (int i = 0; i < pca9685Count; i++) {
      if (pca9685[i] != nullptr) {
        pca9685[i]->begin();
        pca9685[i]->setOscillatorFrequency(27000000);
        pca9685[i]->setPWMFreq(50);
        Serial.printf("PCA9685 #%d at 0x%02X reinitialized\n", i, pca9685Addresses[i]);
      }
    }
    setPca9685RailsPower(railsWereOn);
  }
  
  Serial.println("I2C reinitialization complete");
}

// Set a digit on a specific board segment
// boardIndex: 0 = 0x40 (hours), 1 = 0x41 (minutes)
// startChannel: 0 = second digit/ones (ch 0-6), 7 = first digit/tens (ch 7-13)
// digit: 0-9
void setDigitOnBoard(int boardIndex, int startChannel, int digit) {
  if (boardIndex < 0 || boardIndex >= pca9685Count || digit < 0 || digit > 9) {
    Serial.printf("Invalid parameters: boardIndex=%d, digit=%d\n", boardIndex, digit);
    return;
  }

  if (!isBoardPowered(boardIndex)) {
    turnOnBoard(boardIndex);
    delay(PCA9685_RAILS_PRE_MS);
  }

  if (startChannel != 0 && startChannel != 7) {
    Serial.printf("Invalid startChannel: %d (must be 0 or 7)\n", startChannel);
    return;
  }
  
  // Get the digit pattern
  const byte* pattern = digitPatterns[digit];
  
  // Set each servo according to the pattern
  for (int i = 0; i < 7; i++) {
    int channel = startChannel + i;
    int pwmValue = (pattern[i] == 0) ? SERVO_POS_0 : SERVO_POS_1;
    
    if (pca9685[boardIndex] != nullptr) {
      pca9685[boardIndex]->setPWM(channel, 0, pwmValue);
      Serial.printf("Board %d, Servo %d (digit %d, segment %d): %s (PWM %d)\n", 
                    boardIndex, channel, digit, i, 
                    pattern[i] == 0 ? "OFF" : "ON", pwmValue);
    }
    if (delayBetweenServosMs > 0) {
      delay(delayBetweenServosMs);
    }
  }
}

// Set time display on both boards
// Board 0 (0x40): hours — ch 7-13 = first digit (tens), ch 0-6 = second digit (ones)
// Board 1 (0x41): minutes — ch 7-13 = first digit (tens), ch 0-6 = second digit (ones)
void setTimeDisplay(int hours, int minutes) {
  if (!pca9685Available || pca9685Count < 2) {
    Serial.println("Error: Need at least 2 PCA9685 boards for time display");
    return;
  }

  // Validate and clamp before powering rails (stay HIGH / off if we bail)
  if (hours < 0 || hours > 23) {
    Serial.printf("Error: Invalid hours value %d, clamping to 0-23\n", hours);
    hours = (hours < 0) ? 0 : 23;
  }
  if (minutes < 0 || minutes > 59) {
    Serial.printf("Error: Invalid minutes value %d, clamping to 0-59\n", minutes);
    minutes = (minutes < 0) ? 0 : 59;
  }

  int minutesOnes = minutes % 10;
  int minutesTens = minutes / 10;
  int hoursOnes = hours % 10;
  int hoursTens = hours / 10;

  if (hoursTens > 2 || hoursOnes > 9 || minutesTens > 5 || minutesOnes > 9) {
    Serial.printf("Error: Invalid digit values - Hours: %d%d, Minutes: %d%d\n",
                  hoursTens, hoursOnes, minutesTens, minutesOnes);
    return;
  }

  turnOnPCA9685();
  delay(PCA9685_RAILS_PRE_MS);

  Serial.printf("\n=== Setting Time Display: %02d:%02d ===\n", hours, minutes);
  Serial.printf("0x40 (Hours): %d%d  |  0x41 (Minutes): %d%d\n", hoursTens, hoursOnes, minutesTens, minutesOnes);
  
  // Board 0 (0x40) - Hours: first digit ch 7-13, second digit ch 0-6
  setDigitOnBoard(0, 7, hoursTens);
  if (delayBetweenDigitsMs > 0) delay(delayBetweenDigitsMs);
  setDigitOnBoard(0, 0, hoursOnes);
  if (delayBetweenDigitsMs > 0) delay(delayBetweenDigitsMs);
  
  // Board 1 (0x41) - Minutes: first digit ch 7-13, second digit ch 0-6
  setDigitOnBoard(1, 7, minutesTens);
  if (delayBetweenDigitsMs > 0) delay(delayBetweenDigitsMs);
  setDigitOnBoard(1, 0, minutesOnes);
  if (delayBetweenDigitsMs > 0) delay(delayBetweenDigitsMs);

  Serial.println("=== Time Display Set ===\n");
  armRailsAutoOff(PCA9685_RAILS_HOLD_MS);
}

void handleAPISetTimeDisplay() {
  if (!pca9685Available || pca9685Count < 2) {
    server.send(400, "application/json", "{\"error\":\"Need at least 2 PCA9685 boards\"}");
    return;
  }
  
  if (server.hasArg("time")) {
    String timeStr = server.arg("time");
    
    // Optional: update delays from query params (ms)
    if (server.hasArg("delayDigits")) {
      int v = server.arg("delayDigits").toInt();
      if (v >= 0 && v <= 60000) delayBetweenDigitsMs = v;
    }
    if (server.hasArg("delayServos")) {
      int v = server.arg("delayServos").toInt();
      if (v >= 0 && v <= 5000) delayBetweenServosMs = v;
    }
    
    // Parse time string (format: HH:MM)
    int colonIndex = timeStr.indexOf(':');
    if (colonIndex == -1) {
      server.send(400, "application/json", "{\"error\":\"Invalid time format. Use HH:MM\"}");
      return;
    }
    
    int hours = timeStr.substring(0, colonIndex).toInt();
    int minutes = timeStr.substring(colonIndex + 1).toInt();
    
    // Validate time
    if (hours < 0 || hours > 23 || minutes < 0 || minutes > 59) {
      server.send(400, "application/json", "{\"error\":\"Invalid time values. Hours: 0-23, Minutes: 0-59\"}");
      return;
    }
    
    // Set the time display
    setTimeDisplay(hours, minutes);
    
    // Update last displayed minute and hour to prevent immediate auto-update
    lastDisplayedMinute = minutes;
    lastDisplayedHour = hours;
    
    String json = "{\"success\":true,\"time\":\"" + timeStr + 
                  "\",\"hours\":" + String(hours) + 
                  ",\"minutes\":" + String(minutes) + 
                  ",\"delayBetweenDigits\":" + String(delayBetweenDigitsMs) + 
                  ",\"delayBetweenServos\":" + String(delayBetweenServosMs) + "}";
    server.send(200, "application/json", json);
    
    Serial.printf("Time display updated via API: %02d:%02d\n", hours, minutes);
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing time parameter\"}");
  }
}

void handleAPITimeDisplayDelays() {
  // Optional: update delays from query params (ms)
  if (server.hasArg("delayDigits")) {
    int v = server.arg("delayDigits").toInt();
    if (v >= 0 && v <= 60000) delayBetweenDigitsMs = v;
  }
  if (server.hasArg("delayServos")) {
    int v = server.arg("delayServos").toInt();
    if (v >= 0 && v <= 5000) delayBetweenServosMs = v;
  }
  String json = "{\"delayBetweenDigits\":" + String(delayBetweenDigitsMs) + 
                ",\"delayBetweenServos\":" + String(delayBetweenServosMs) + "}";
  server.send(200, "application/json", json);
}

void handleAPIAutoUpdate() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      autoUpdateEnabled = true;
    } else if (state == "off") {
      autoUpdateEnabled = false;
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid state. Use on/off\"}");
      return;
    }
    String json = "{\"success\":true,\"enabled\":" + String(autoUpdateEnabled ? "true" : "false") + "}";
    server.send(200, "application/json", json);
    Serial.printf("[Auto Update] %s via web\n", autoUpdateEnabled ? "Enabled" : "Disabled");
    return;
  }

  String json = "{\"enabled\":" + String(autoUpdateEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// Auto-update time display from RTC every minute
void updateTimeDisplayAuto() {
  if (!autoUpdateEnabled) {
    return;
  }

  // Only update if RTC is available and we have PCA9685 boards
  if (!rtcAvailable || !pca9685Available || pca9685Count < 2) {
    return;
  }
  
  // If RTC has been invalid too many times, wait longer before retrying
  unsigned long currentTime = millis();
  if (rtcInvalidCount >= MAX_RTC_INVALID_COUNT) {
    // Wait 30 seconds before retrying if RTC was consistently invalid
    if (currentTime - lastRTCErrorTime < 30000) {
      return;
    }
    // Reset counter and try again
    rtcInvalidCount = 0;
    Serial.println("[Auto Update] Retrying RTC read after error timeout");
  }
  
  // Prevent rapid updates - minimum 2 seconds between checks
  if (currentTime - lastUpdateTime < 2000) {
    return;
  }
  lastUpdateTime = currentTime;
  
  // Get current time from RTC with retry and I2C recovery (same as handleAPITime)
  DateTime now;
  bool readSuccess = readRTCWithRetry(now, 5, true);
  
  if (!readSuccess) {
    rtcInvalidCount++;
    lastRTCErrorTime = currentTime;
    if (rtcInvalidCount >= MAX_RTC_INVALID_COUNT) {
      Serial.println("[Auto Update] RTC time consistently invalid - pausing auto-updates");
      Serial.println("[Auto Update] Please sync RTC time using web interface 'Sync with Browser' button");
    }
    return;
  }
  
  int currentMinute = now.minute();
  int currentHour = now.hour();
  
  // Validate time values (hours: 0-23, minutes: 0-59)
  if (currentHour < 0 || currentHour > 23 || currentMinute < 0 || currentMinute > 59) {
    rtcInvalidCount++;
    lastRTCErrorTime = currentTime;
    Serial.printf("[Auto Update] Invalid RTC time: %02d:%02d - skipping update (error count: %d/%d)\n", 
                  currentHour, currentMinute, rtcInvalidCount, MAX_RTC_INVALID_COUNT);
    
    if (rtcInvalidCount >= MAX_RTC_INVALID_COUNT) {
      Serial.println("[Auto Update] RTC time consistently invalid - pausing auto-updates");
      Serial.println("[Auto Update] Please sync RTC time using web interface 'Sync with Browser' button");
    }
    return;
  }
  
  // Valid time read - reset error counter
  if (rtcInvalidCount > 0) {
    Serial.printf("[Auto Update] RTC time is now valid - resuming auto-updates\n");
    rtcInvalidCount = 0;
  }

  if (currentHour < (int)workHoursStartHour || currentHour > (int)workHoursStopHour) {
    return;
  }
  
  // Only update if minute OR hour has changed (to handle hour rollover)
  if (currentMinute != lastDisplayedMinute || currentHour != lastDisplayedHour) {
    Serial.printf("\n[Auto Update] Time changed: %02d:%02d\n", currentHour, currentMinute);
    
    // Update the time display
    setTimeDisplay(currentHour, currentMinute);
    
    // Update last displayed minute and hour
    lastDisplayedMinute = currentMinute;
    lastDisplayedHour = currentHour;
    
    Serial.printf("[Auto Update] Time display updated to %02d:%02d\n", currentHour, currentMinute);
  }
}


