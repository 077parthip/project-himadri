<div align="center">

# 🧊 Project Himadri
### Bio-Aware Thermal Cold Storage for Rural India

*A solar-powered, **battery-less** mini cold room that stores cold in glycol instead of electricity in lithium.*

![SIH 2026](https://img.shields.io/badge/SIH-2026-orange?style=for-the-badge)
![Category](https://img.shields.io/badge/Category-Hardware-blue?style=for-the-badge)
![PS ID](https://img.shields.io/badge/PS%20ID-SIH26005-success?style=for-the-badge)
![MCU](https://img.shields.io/badge/MCU-ESP32--S3-red?style=flat-square&logo=espressif)
![RTOS](https://img.shields.io/badge/FreeRTOS-multitasking-green?style=flat-square)
![Dashboard](https://img.shields.io/badge/Dashboard-Streamlit-ff4b4b?style=flat-square&logo=streamlit)
![Power](https://img.shields.io/badge/Power-100%25%20Solar-yellow?style=flat-square)

</div>

---

## 📌 Summary
Post-harvest losses hurt small farmers most where grid power is unreliable and lithium
batteries are unaffordable. **Himadri** replaces the battery with a **thermal battery**:
panels filled with propylene-glycol PCM are frozen to **−2 °C** by a solar-driven compressor
during the day. At night the compressor is OFF and the frozen walls passively hold the room at
**4 – 8 °C**.

On top of that, Himadri is **bio-aware**. Rotting produce respires and releases CO₂, heat and
ethylene. An NDIR CO₂ sensor spots the spike early; the controller opens **only the affected
zone's damper** to vent it, and texts the farmer through a GSM module – so one bad tomato does
not spoil the whole room.

| Problem | Himadri's answer |
|---|---|
| Lithium batteries are costly and short-lived | Glycol PCM thermal battery – cold is stored, not electricity |
| Spoilage spreads silently | CO₂ spike detection per zone |
| No internet in villages | SMS alerts over 2G (SIM800L) |
| Farmers not present 24×7 | Autonomous fail-safe control |
| Fan energy wasted at fixed speed | PI-controlled zonal fan speed (P ∝ speed³) with measured energy savings |

## 🏗 System Architecture
```
                       ┌───────────────────────────────┐
   ☀ Solar PV ────────►│  Day / Night detector (ADC)   │
                       └──────────────┬────────────────┘
                                      │
 ┌──────────────┐     ┌───────────────▼───────────────┐     ┌────────────────────┐
 │ SHT35 ×4     │ I²C │        ESP32-S3 (FreeRTOS)    │ UART│ SIM800L GSM        │──► 📱 SMS
 │ SCD30 ×4     ├────►│  sensorTask → controlTask     ├────►│ (AT commands)      │
 │ (via TCA9548A│     │              ↓ alertQueue     │     └────────────────────┘
 │  I²C mux)    │     │          telemetryTask        │
 └──────────────┘     └───┬──────────┬─────────────┬──┘
                          │PWM       │GPIO         │ADC
                   ┌──────▼─────┐ ┌──▼──────────┐ ┌▼───────────────────┐
                   │ Servo      │ │ Compressor  │ │ NTC on glycol wall │
                   │ dampers ×4 │ │ relay       │ │ (freeze status)    │
                   └────────────┘ └─────────────┘ └────────────────────┘
```

### Operating modes
| Mode | Trigger | Compressor | Room temp maintained by |
|---|---|---|---|
| ☀ **Day** | Solar voltage > 12 V | ON until glycol ≤ −2 °C | Compressor + PCM charging |
| 🌙 **Night** | Solar voltage low | OFF | Frozen glycol walls (passive) |
| 🚨 **Bio-alert** | CO₂ > 1000 ppm **or** T > 8 °C | unchanged | Zone damper opens, SMS sent |

## 🔄 Data Flow
```
[SHT35 / SCD30 per zone] ──I²C──► sensorTask ──► shared state (mutex)
                                                        │
                        ┌───────────────────────────────┴───────────────┐
                        ▼                                               ▼
                  controlTask (500 ms)                     Streamlit dashboard
                  ├─ Day? → compressor relay                (MQTT / simulation)
                  ├─ CO₂>1000 / T>8 → servo damper
                  └─ push Alert ──► alertQueue
                                        │
                                        ▼
                                telemetryTask
                                AT+CMGF=1 → AT+CMGS → text → Ctrl+Z
                                        │
                                        ▼
                                  📱 Farmer's phone
```

## 🧰 Hardware & Software Stack
**Hardware:** ESP32-S3 · SHT35 (temp/RH) · SCD30 (NDIR CO₂) · TCA9548A I²C mux · SIM800L ·
SG90/MG90S servos (dampers) · 4× 4-wire PWM fans (tach feedback) · INA219 (fan power) · relay module ·
glycol PCM panels · solar PV + compressor · NTC 10 k

**Software:** C++ / Arduino-ESP32 on **FreeRTOS** (PlatformIO) · single-file HTML/JS **HMI** (process station
style, no dependencies) · optional Python/Streamlit dashboard

## 🌬 Energy-Optimised Fan Control
Each zone has its own fan, tach feedback and PI loop (`fanController()` in `firmware/main.cpp`).

| Element | Behaviour |
|---|---|
| Temperature PI | error = T − 6 °C, ±0.3 °C deadband, integral clamped (anti-windup) |
| Humidity trim | RH < 85 % → slower (protect produce) · RH > 95 % → faster (avoid condensation) |
| Limits | 25 % floor (air mixing), 50 % cap at night, 10 %/s slew limit |
| Bio-alert override | damper open (rot / warm) → 100 % to flush the zone |
| Fan fault | commanded > 30 % but tach < 200 rpm for 4 s → alarm + SMS |
| Energy accounting | INA219 power integrated (Wh) vs a fixed-100 % baseline → "energy saved %" on the HMI |

Fan power ≈ speed³, so 50 % speed uses ≈ 12.5 % power, and less fan-motor heat inside the room means the
compressor works less and the thermal battery lasts longer at night. Gains and RH limits are starting values:
tune per crop on the real plant.

## 📁 Repository Structure
```
project-himadri/
├── README.md
├── platformio.ini          # build envs: simulation | hardware
├── firmware/
│   └── main.cpp            # ESP32-S3 FreeRTOS firmware (sensing, dampers, PI fans, SMS)
├── docs/
│   └── index.html          # HMI / process station (GitHub Pages)
└── dashboard/
    ├── app.py              # Streamlit IoT dashboard
    ├── requirements.txt
    └── .streamlit/config.toml   # dark theme
```

## 🚀 Installation for Evaluators

### A. HMI process station (no install at all)
Open `docs/index.html` in any browser, or the GitHub Pages link once enabled
(Settings → Pages → Branch `main`, folder `/docs`). It runs a built-in simulation:
watch Zone C rot, its damper open and fan go to 100 %, the fans throttle the rest of the time,
and the **Energy saved** KPI climb. Live data: `index.html?api=http://<esp32-ip>/data.json`.

### A2. Streamlit dashboard (optional, no hardware needed)
```bash
git clone https://github.com/<your-username>/project-himadri.git
cd project-himadri/dashboard
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
streamlit run app.py
```
Open http://localhost:8501. Zone C starts to "rot" after ~1 minute: watch CO₂ cross the red
limit line, the tile turn red (**Rot Warning! Damper Open**), then recover as venting works.

### B. Firmware
```bash
pip install platformio
cd project-himadri
pio run -e simulation -t upload     # runs with simulated sensors, prints to serial
pio device monitor                  # 115200 baud
pio run -e hardware -t upload       # real SHT35 / SCD30 / SIM800L build
```
Before flashing the hardware build, set `SMS_RECIPIENT` and verify pin numbers in `firmware/main.cpp`.

## 🛡 Safety design choices
- Compressor relay and all dampers default to a safe state at boot.
- Control never acts on a failed/stale sensor reading.
- Hysteresis on both thresholds stops damper chatter.
- SMS is rate-limited (1 per zone per 10 min) and runs in its own low-priority task so a slow
  network can never block the control loop.

## 🗺 Roadmap
- Publish telemetry over Wi-Fi/MQTT from the ESP32 (dashboard already subscribes).
- Ethylene sensor per zone; adaptive CO₂ thresholds by crop.
- Field pilot with a farmer cooperative.

## 📜 License
MIT (add a `LICENSE` file before publishing).
