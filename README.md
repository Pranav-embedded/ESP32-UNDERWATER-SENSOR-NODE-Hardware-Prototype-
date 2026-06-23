# ESP32-CAM Underwater Sensor Node — Stage 2 Hardware Prototype

**Pelvyn Robotics — Stage 2 Hardware Prototype Submission**
Author: Pranav Duse · T.E. ECE, PES Modern College of Engineering (SPPU)

A working ESP32-CAM sensor node that reads IMU (MPU6050) and pressure/temperature
(BMP280) data, publishes live telemetry over MQTT, and streams live video — viewable
on a browser-based dashboard over the same WiFi network.

---

## 1. Hardware used

| Component        | Part                          | Interface         |
|-------------------|--------------------------------|---------------------|
| Main controller   | ESP32-CAM (AI-Thinker)         | —                    |
| IMU               | MPU6050 (GY-521 module)        | I2C (addr 0x68)      |
| Pressure/temp     | BMP280                          | I2C (addr 0x76/0x77) |
| Camera            | OV2640 (onboard)               | DCMI (fixed pins)    |
| Status LED        | Onboard GPIO4                  | GPIO                 |
| Connectivity      | WiFi (STA mode) + MQTT          | TCP/IP                |
| Programming       | External FTDI / USB-TTL adapter | UART                 |

> **No SD card or external pressure-rated sensor (MS5837) in this build** — both were
> not deliverable within the assessment window (see Section 6, Challenges). The system
> is fully documented as IMU + air-pressure telemetry over MQTT with live video,
> demonstrating the complete sensing -> publish -> dashboard pipeline that would carry
> over directly to a submersible-rated sensor in a production build.

## 2. Wiring

| Signal          | ESP32-CAM pin | Notes                                  |
|-----------------|---------------|------------------------------------------|
| I2C SDA         | GPIO15        | Shared bus — MPU6050 + BMP280            |
| I2C SCL         | GPIO14        | Shared bus — MPU6050 + BMP280            |
| Status LED      | GPIO4         | Onboard flash LED, reused as status      |
| Camera          | Fixed internal pins | Pre-wired on board, do not touch    |
| Programming     | U0R/U0T + GPIO0 (boot mode) | FTDI adapter, common GND     |

**Power:** 5V via FTDI VCC during bench testing, or a regulated 5V/1A supply for
standalone operation (camera + WiFi peak draw is ~250-300 mA).

## 3. System architecture

```
   MPU6050 ──┐
             ├── I2C (GPIO14/15) ──> ESP32-CAM ──> WiFi ──> MQTT broker
   BMP280  ──┘                         │                        │
                                        │                        ▼
                                   OV2640 camera          dashboard_mqtt.html
                                        │                  (WebSocket subscriber)
                                        ▼
                              MJPEG stream (port 80/stream)
                              served directly to the same dashboard's <img> tag
```

Two independent data paths run side by side:
- **MQTT (WebSocket over port 9001)** — sensor JSON published every 1 second to
  topics `sensors/node1/bmp280`, `sensors/node1/mpu6050`, `sensors/node1/camera`,
  `sensors/node1/status`. This is what the brief's MQTT requirement is built on.
- **Direct HTTP MJPEG stream (port 80/stream)** — live video pulled directly from
  the ESP32-CAM's own web server, embedded as an `<img>` tag in the same dashboard
  page, so video and MQTT telemetry sit side by side in one view.

## 4. Hardware selection rationale

| Component       | Choice              | Why |
|-------------------|---------------------|-----|
| Controller       | ESP32-CAM (AI-Thinker) | Satisfies ESP32 requirement, adds onboard camera at no extra hardware cost |
| IMU              | MPU6050 (GY-521)     | Cheap, well-documented, I2C, accelerometer-based roll/pitch sufficient for this prototype |
| Pressure/temp    | BMP280               | I2C, low cost, fast delivery; used as an air-pressure proxy for the depth-sensing pipeline (see Section 6) |
| Camera           | OV2640 (onboard)     | Built into the ESP32-CAM, zero extra wiring, demonstrates live visual telemetry |
| Broker transport | MQTT over WebSocket (port 9001) | Lets a browser dashboard subscribe directly via Paho MQTT JS, no backend server needed |
| Local broker     | Self-hosted (Mosquitto on laptop, port 1883/9001) | Full control for development/demo without depending on internet access |

## 5. Communication architecture

The firmware publishes four MQTT topics every second:

| Topic                      | Payload                                              |
|------------------------------|--------------------------------------------------------|
| `sensors/node1/bmp280`       | `{temperature, pressure, altitude, ok}`                |
| `sensors/node1/mpu6050`      | `{accelX/Y/Z, gyroX/Y/Z, roll, pitch, ok}`              |
| `sensors/node1/camera`       | `{camOK, sdOK, bmpOK, mpuOK, uptime, rssi}`             |
| `sensors/node1/status`       | Last-Will-and-Testament: `{status: online/offline, ip}` |

The dashboard (`dashboard_mqtt.html`) is a static HTML file — no backend required.
It connects directly to the broker over WebSocket (`Paho MQTT JS` library, loaded
from CDN), subscribes to `sensors/node1/#` (wildcard, all four topics), and updates
its UI live as messages arrive. Video is pulled separately and directly from the
ESP32-CAM's own `/stream` endpoint into an `<img>` tag — independent of MQTT, so
losing the MQTT connection doesn't interrupt video, and vice versa.

### Why a Last-Will-and-Testament (LWT) message
The MQTT connect call registers an LWT on `sensors/node1/status`: if the ESP32-CAM
loses its connection ungracefully (power loss, WiFi drop) without a clean disconnect,
the broker automatically publishes `{"status":"offline"}` on its behalf. The dashboard
can therefore distinguish "node went offline" from "node never connected" — a real
system-monitoring feature, not just nice-to-have.

## 6. Challenges faced

- **MPU6050 library initialization unreliable on this specific module:** the standard
  library's `initialize()` call did not reliably wake the sensor from sleep on this
  hardware. Resolved by writing directly to the `PWR_MGMT_1` register (0x6B) over raw
  I2C — explicit reset, wake, and clock-source configuration — then verifying via the
  `WHO_AM_I` register (0x75) before trusting the library layer at all.
- **BMP280 library `getMeasurements()` intermittently returned false** even after a
  successful `begin()`. Resolved with a permanent fallback path: after 3 consecutive
  library read failures, the firmware switches to raw register reads (burst-reading
  pressure/temperature directly from registers 0xF7-0xFC) using the sensor's own
  factory calibration coefficients, computed manually with the Bosch datasheet formula.
- **No SD card / no MS5837 available within the assessment window** — both were
  explicitly out of scope for this build; the architecture and firmware are structured
  so either could be added later (MS5837 uses the same I2C bus pattern as BMP280; SD
  logging would slot in alongside the existing MQTT publish call) without redesigning
  the system.
- **MJPEG streaming competing with sensor/MQTT loop for CPU time:** resolved by pinning
  the video stream handler to its own FreeRTOS task on Core 0, while sensor reads and
  MQTT publish run on the default Arduino loop on Core 1 — the two no longer block
  each other.
- **I2C address ambiguity (BMP280 at 0x76 vs 0x77, chip-ID mismatches on clone boards):**
  resolved with a boot-time I2C scanner and explicit chip-ID register read (0xD0) to
  confirm genuine BMP280 vs. BME280 clones before committing to a fixed address.

## 7. Future improvements

- Add MS5837 (true underwater-rated depth sensor) in place of BMP280 for genuine
  submersible deployment — same I2C bus, drop-in replacement at the firmware level.
- Add microSD logging alongside the existing MQTT publish, so data persists locally
  even during a network outage (current build is MQTT-only, no local backup).
- Move from a local/open broker to an authenticated, TLS-secured MQTT broker for any
  real deployment beyond bench testing.
- Add a hardware leak-detection sensor and emergency shutdown logic, carried over from
  the Stage 1 electronics architecture design.
- Add reconnect/backoff tuning and message queuing so brief WiFi dropouts don't lose
  sensor readings between reconnect attempts.

## 8. Repository contents

```
firmware/
├── esp32cam_mqtt_v2.ino     # complete ESP32-CAM firmware (camera, sensors, MQTT, HTTP stream)
└── dashboard_mqtt.html       # standalone browser dashboard (MQTT WebSocket + live video)
docs/
└── technical_report.pdf       # 3-page technical report (architecture, rationale, challenges)
images/
└── (hardware photos go here — see Section 9)
```

## 9. Setup instructions

1. **Flash the firmware:**
   - Open `esp32cam_mqtt_v2.ino` in Arduino IDE.
   - Install libraries: `BMP280_DEV` (Martin Lindup), `MPU6050` (Electronic Cats or
     i2cdevlib), `PubSubClient`.
   - Edit the top of the file: set `WIFI_SSID`, `WIFI_PASSWORD`, and `MQTT_BROKER`
     (the IP address of your MQTT broker — e.g. a laptop running Mosquitto).
   - Connect FTDI: `5V-5V, GND-GND, TX-U0R, RX-U0T`. Bridge GPIO0 to GND before
     powering on to enter flash mode.
   - Board: `AI Thinker ESP32-CAM`. Upload. Remove the GPIO0 bridge, reset.
   - Open Serial Monitor at 115200 baud — confirm I2C scan, sensor init, WiFi IP,
     and MQTT connection messages.

2. **Run a local MQTT broker** (if not already running) — e.g. Mosquitto with
   WebSocket support enabled on port 9001 (required for the browser dashboard to
   connect directly).

3. **Open the dashboard:**
   - Open `dashboard_mqtt.html` directly in any browser (just double-click it, or
     drag it into a browser window — no server needed).
   - Enter the broker's IP and WebSocket port (default 9001) in the config bar.
   - Enter the ESP32-CAM's IP address (shown in Serial Monitor) for video.
   - Click **Connect** — sensor data and video should appear within a few seconds.

## 10. Author confirmation

I confirm that all content in this submission, including firmware, dashboard, and
documentation, is my original work produced for the Pelvyn Robotics Stage 2 hardware
prototype assessment.

**Pranav Duse** · T.E. ECE, PES Modern College of Engineering (SPPU) · June 2026
