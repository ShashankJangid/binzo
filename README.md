# BINZO — Smart Autonomous Dustbin Robot

BINZO is a smart, autonomous robotic dustbin built with dual ESP32 microcontrollers. It follows black line tracks with infrared sensors, detects approaching people with an ESP32-CAM and ultrasonic proximity fallback, opens its motorized lid automatically, and communicates over ESP-NOW and Wi-Fi.

---

## 1. System Overview & Architecture

- **Main Controller:** 30-pin ESP32-WROOM DevKit (ESP32-D0WD-V3)
- **Vision Subsystem:** AI-Thinker ESP32-CAM (OV2640 + PSRAM)
- **Drivetrain:** 4× DC gear motors with mecanum wheels driven by 2× L298N dual H-bridges
- **Line Following:** 2× TCRT5000 IR reflectance sensors (PD controller with boot auto-calibration)
- **Actuation:** MG90S 9g metal-gear servo for automatic lid opening/closing
- **Proximity Fallback & Safety:** HC-SR04 ultrasonic distance sensor
- **Inter-Chip Networking:** 
  - Main creates a SoftAP (`Binzo-Bin`, password `binzo1234`, `192.168.4.1`)
  - CAM connects to AP as a Station (`192.168.4.200`) and serves a live diagnostics snapshot page at port 80
  - CAM transmits human detection states via **ESP-NOW** broadcast every 500 ms

---

## 2. Pinout & Wiring

### Main ESP32 (30-Pin DevKit)
| GPIO | Function | Description / Notes |
|:---:|:---|:---|
| **4**  | Rear-Right Motor IN1 | L298N #2 Direction |
| **13** | Rear-Right Motor IN2 | L298N #2 Direction |
| **14** | Front-Right Motor IN1 | L298N #1 Direction |
| **16** | Front-Right Motor IN2 | L298N #1 Direction |
| **17** | Front-Right Motor EN (PWM) | Remove ENB jumper on L298N #1 |
| **18** | MG90S Servo PWM | 50 Hz PWM signal (Orange wire) |
| **19** | HC-SR04 TRIG | 10 µs trigger pulse |
| **21** | Rear-Left Motor EN (PWM) | Remove ENA jumper on L298N #2 |
| **22** | Rear-Right Motor EN (PWM) | Remove ENB jumper on L298N #2 |
| **23** | Power Indicator LED | 220–330 Ω series resistor to GND |
| **25** | Front-Left Motor EN (PWM) | Remove ENA jumper on L298N #1 |
| **26** | Front-Left Motor IN1 | L298N #1 Direction |
| **27** | Front-Left Motor IN2 | L298N #1 Direction |
| **32** | Rear-Left Motor IN1 | L298N #2 Direction |
| **33** | Rear-Left Motor IN2 | L298N #2 Direction |
| **34** | Left IR OUT | Input-only; powered from 3.3V |
| **35** | Right IR OUT | Input-only; powered from 3.3V |
| **36** | Run/Stop Toggle Switch | Input-only; Active LOW (Requires external 10k pull-up to 3.3V) |
| **39** | HC-SR04 ECHO | Input-only; 1k Ω series + 2k Ω to GND voltage divider |

### AI-Thinker ESP32-CAM
Standard AI-Thinker pinout (OV2640 camera, PSRAM, Flash LED on GPIO 4, Status LED on GPIO 33, optional PIR on GPIO 13).

---

## 3. Power Architecture & Safety Rules

> [!CAUTION]
> **Never connect the outputs of the two 5V buck converters together.** Connect all ground wires (GND) together for a common reference.

- **5V Buck Converter #1 (3A):** Supplies Main ESP32 (VIN) and HC-SR04 sensor.
- **5V Buck Converter #2 (3A):** Supplies MG90S Servo and ESP32-CAM (with a 470 µF capacitor across CAM 5V and GND).
- **Motor Power (7.4V–12V Battery):** Feeds into L298N 12V inputs. Keep the onboard 5V regulator jumpers intact.
- **Line Sensors:** Powered exclusively from ESP32 **3.3V** (GPIO34 and GPIO35 are not 5V tolerant).
- **Ultrasonic Echo Protection:** HC-SR04 Echo output (5V logic) must pass through a 1k / 2k voltage divider to protect GPIO39 (3.3V maximum).

---

## 4. Finite State Machine (FSM)

```
 [BOOT] -> Auto-calibrates IR sensors for 2.5s
   |
   v
 [IDLE] <---------------------------------------------+
   | (Run switch ON: GPIO36 pulled LOW)               |
   v                                                  |
[FOLLOW] --(Obstacle < 12cm)---------> [OBSTACLE]     |
   |                                      |           |
   |                               (Clear > 25cm)     |
   |                                      v           |
   |                                  [FOLLOW]        |
   |                                                  |
   +--(Person Detected)--> [OPEN] -> [HOLD] -> [CLOSE]+
   |                                   (4s)
   |
   +--(Line Lost > 150ms)--> [SEARCH] --(Timeout > 1.2s)--> [LOST]
                                |
                         (Line Found)
                                v
                            [FOLLOW]
```

---

## 5. Directory Structure

```
binzo/
├── README.md
├── .gitignore
├── upload.ps1
└── Binzo/
    ├── README.md
    └── firmware/
        ├── Binzo_Mecanum/
        │   ├── Binzo_Mecanum.ino   # Main 4-wheel mecanum controller firmware
        │   └── config.h            # System pins, gains & tuning parameters
        ├── Binzo_Cam/
        │   ├── Binzo_Cam.ino       # ESP32-CAM vision, ESP-NOW & web server
        │   └── camera_pins.h       # AI-Thinker camera pin mappings
        └── Binzo_Main/
            └── Binzo_Main.ino      # 2-motor differential drive baseline
```

---

## 6. How to Flash

### Flashing Main ESP32
1. Open `Binzo/firmware/Binzo_Mecanum/Binzo_Mecanum.ino` in Arduino IDE.
2. Select Board: **ESP32 Dev Module**.
3. Select COM port and flash.

### Flashing ESP32-CAM
1. Open `Binzo/firmware/Binzo_Cam/Binzo_Cam.ino`.
2. Connect ESP32-CAM via FTDI or ESP32-CAM-MB with **GPIO 0 connected to GND**.
3. Select Board: **AI Thinker ESP32-CAM**.
4. Enable **PSRAM: Enabled**, Partition Scheme: **Huge APP (3MB No OTA / 1MB SPIFFS)**.
5. Flash, disconnect GPIO 0 from GND, and press RESET.


---

## 7. M5Stack Core2 4x4 Mecanum Drive & RadioMaster Pocket Controller

In branch `m5core2-mecanum-swerve`, BINZO also supports an **M5Stack Core2 + 4-Channel Encoder Motor Module (BL5617)** stack with real-time wireless UDP control via a **RadioMaster Pocket** RC transmitter (USB HID) connected through a laptop bridge.

### Features
- **Holonomic 2D Planar Swerve:** Right joystick controls 2D strafe ($V_x$) and forward/backward ($V_y$) translation; Left joystick controls master speed throttle and yaw ($\omega$) in-place rotation.
- **Ultra-low latency UDP:** 40 Hz packet streaming with 1 ms Windows high-resolution multimedia timer.
- **M5 Display Dashboard & Touch Safety:** Live 2D vector indicator, individual motor PWM gauges, WiFi/battery telemetry, and hardened capacitive emergency stop banner.
- **Live Debug Telemetry (UDP Port 4212):** Live battery voltage, RSSI, loop latency, brownout detection, and failsafe diagnostics streamed back to the laptop HUD and logged to `session_debug.log`.

### Directory Structure
- **Firmware:** `Binzo/firmware/Binzo_M5Core2/` and `m5core2_firmware/` (PlatformIO + Arduino)
- **Laptop Bridge:** `laptop_controller/radio_bridge.py`
- **Quick Launch:** `run_bridge.bat` (controller bridge) and `run_tester.bat` (axis calibration tester)
- **Setup Guide:** `docs/SETUP_GUIDE.md`
