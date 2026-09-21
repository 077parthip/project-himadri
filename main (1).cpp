/**
 * ============================================================================
 *  PROJECT HIMADRI  -  Bio-Aware Thermal Cold Storage        (SIH 2026 | SIH26005)
 *  Firmware v2: ESP32-S3 + FreeRTOS + energy-optimised zonal fan control
 * ============================================================================
 *  Tasks and shared state:
 *
 *   [sensorTask]  --writes-->  g_zones[] (mutex)  --read by-->  [controlTask]
 *    SHT35/SCD30, tach, INA219                                        |
 *                                                     alertQueue (FreeRTOS queue)
 *                                                                    v
 *                                                            [telemetryTask]
 *                                                      (SIM800L AT-command SMS)
 *
 *  CONTROL LOOPS (per zone, every 500 ms)
 *   1. Safety / bio-aware : CO2 > 1000 ppm or T > 8 C  -> damper OPEN + fan 100 %
 *   2. Fan speed (energy) : PI on temperature + humidity trim, 25 % floor,
 *                           night cap 50 %, slew-rate limited.
 *   3. Compressor         : ON only in Day Mode and only until the glycol wall <= -2 C.
 *
 *  WHY VARIABLE FAN SPEED SAVES ENERGY
 *   Fan power ~ speed^3 (affinity law). 50 % speed = ~12.5 % power, and every watt
 *   not turned into fan-motor heat is cold the thermal battery keeps for the night.
 *   Firmware integrates measured fan energy (INA219) against a fixed-100 % baseline
 *   and reports the saving on the HMI.
 *
 *  BUILD MODES
 *   -DSIMULATE_SENSORS=1 (default) : no hardware, closed-loop plant simulation.
 *   -DSIMULATE_SENSORS=0           : real drivers (Adafruit SHT31, SparkFun SCD30,
 *                                    Adafruit INA219, ESP32Servo).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <HardwareSerial.h>   // ESP32 has spare hardware UARTs -> more reliable than SoftwareSerial
#include <ESP32Servo.h>

#ifndef SIMULATE_SENSORS
#define SIMULATE_SENSORS 1
#endif

#if !SIMULATE_SENSORS
  #include <Adafruit_SHT31.h>   // SHT35 is register-compatible with SHT31
  #include <SparkFun_SCD30_Arduino_Library.h>
  #include <Adafruit_INA219.h>  // measures total fan power on the 12 V fan rail
#endif

// Arduino-ESP32 core 3.x changed the LEDC (PWM) API. Support both.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define HIMADRI_CORE3 1
#else
  #define HIMADRI_CORE3 0
#endif

// ----------------------------------------------------------------------------
//  1. CONFIGURATION  (all tunables in one place -> easy to explain / change)
// ----------------------------------------------------------------------------
constexpr uint8_t  NUM_ZONES          = 4;       // Zones A, B, C, D
const char         ZONE_NAME[NUM_ZONES] = {'A', 'B', 'C', 'D'};

// Safety thresholds (from problem statement)
constexpr float    TEMP_LIMIT_C       = 8.0f;    // above this = warm alarm
constexpr float    CO2_LIMIT_PPM      = 1000.0f; // above this = probable rotting
constexpr float    HYSTERESIS_TEMP_C  = 1.0f;    // prevents damper "chatter"
constexpr float    HYSTERESIS_CO2_PPM = 100.0f;

// Thermal battery (glycol PCM wall) limits
constexpr float    PCM_FROZEN_C       = -2.0f;   // fully "charged"
constexpr float    PCM_WARM_C         = 6.0f;    // fully "discharged"

// Day / Night detection from solar panel voltage (via divider on ADC)
constexpr float    SOLAR_DAY_VOLT     = 12.0f;   // above -> enough sun for compressor

// ---- Fan control (tune these on the real plant) ------------------------------
constexpr float FAN_SETPOINT_C   = 6.0f;    // target air temperature in each zone
constexpr float FAN_DEADBAND_C   = 0.3f;    // no action inside +/- this band
constexpr float FAN_KP           = 25.0f;   // % speed per degC of error
constexpr float FAN_KI           = 1.5f;    // % speed per (degC * s)
constexpr float FAN_BIAS_PCT     = 30.0f;   // feed-forward speed at zero error
constexpr float FAN_INT_MIN      = -20.0f;  // anti-windup clamp on the integral term
constexpr float FAN_INT_MAX      =  40.0f;
constexpr float FAN_MIN_PCT      = 25.0f;   // floor: keeps air mixed so sensors read true
constexpr float FAN_MAX_DAY_PCT  = 100.0f;
constexpr float FAN_MAX_NIGHT_PCT= 50.0f;   // night = passive cooling, gentle circulation
constexpr float FAN_SLEW_PCT_S   = 10.0f;   // max change per second (no hunting / noise)
constexpr float RH_DRY_PCT       = 85.0f;   // below: slow fans (produce loses moisture)
constexpr float RH_WET_PCT       = 95.0f;   // above: speed up (avoid condensation / mould)
constexpr float FAN_FAILSAFE_PCT = 60.0f;   // used when a zone's sensor is invalid

// Fan hardware (4-wire PWM fans)
constexpr float    FAN_RATED_W   = 2.4f;    // per fan at 100 % (datasheet)
constexpr float    FAN_MAX_RPM   = 3000.0f;
constexpr uint8_t  FAN_TACH_PPR  = 2;       // tach pulses per revolution
constexpr uint32_t FAN_PWM_HZ    = 25000;   // Intel 4-wire fan spec
constexpr uint8_t  FAN_PWM_BITS  = 10;
constexpr float    FAN_FAIL_RPM  = 200.0f;  // commanded > 30 % but below this = fault
constexpr uint8_t  FAN_FAIL_CYCLES = 8;     // consecutive control cycles (4 s)

// Pins (ESP32-S3-DevKitC). Adjust to your PCB.
constexpr int PIN_I2C_SDA      = 8;
constexpr int PIN_I2C_SCL      = 9;
constexpr int PIN_RELAY_COMP   = 4;              // compressor relay (active HIGH)
constexpr int PIN_SERVO[NUM_ZONES]    = {5, 6, 7, 15};
constexpr int PIN_FAN_PWM[NUM_ZONES]  = {10, 11, 12, 13};   // PWM to fan pin 4
constexpr int PIN_FAN_TACH[NUM_ZONES] = {14, 21, 38, 39};   // tach from fan pin 3
constexpr int PIN_SIM_TX       = 17;             // ESP32 TX -> SIM800L RX
constexpr int PIN_SIM_RX       = 18;             // ESP32 RX <- SIM800L TX
constexpr int PIN_ADC_SOLAR    = 1;              // solar voltage divider
constexpr int PIN_ADC_PCM_NTC  = 2;              // NTC thermistor glued to glycol wall
constexpr uint8_t TCA9548A_ADDR = 0x70;          // I2C mux: 1 SHT35+SCD30 pair per zone

constexpr int SERVO_CLOSED_DEG = 0;
constexpr int SERVO_OPEN_DEG   = 90;

// SMS
const char SMS_RECIPIENT[]     = "+91XXXXXXXXXX"; // farmer's number
constexpr uint32_t SMS_COOLDOWN_MS = 10UL * 60UL * 1000UL; // max 1 SMS / zone / 10 min

// Task timing
constexpr uint32_t SENSOR_PERIOD_MS  = 2000;     // SCD30 updates every 2 s
constexpr uint32_t CONTROL_PERIOD_MS = 500;

// ----------------------------------------------------------------------------
//  2. SHARED DATA TYPES
// ----------------------------------------------------------------------------
struct ZoneData {
  float tempC   = NAN;
  float humPct  = NAN;
  float co2ppm  = NAN;
  float fanPct  = FAN_MIN_PCT;   // commanded fan speed
  float fanRpm  = 0;             // measured (tach)
  bool  damperOpen = false;
  bool  valid   = false;         // false until the first successful read
};

enum AlertType : uint8_t { ALERT_ROT, ALERT_WARM, ALERT_FAN_FAIL };

struct Alert {
  AlertType type;
  uint8_t   zone;
  float     tempC;
  float     co2ppm;
};

struct FanPI { float integ = 0; float out = FAN_MIN_PCT; };

static ZoneData          g_zones[NUM_ZONES];
static FanPI             g_fanPI[NUM_ZONES];
static float             g_pcmTempC   = 4.0f;    // thermal-battery wall temperature
static bool              g_dayMode    = false;
static bool              g_compressorOn = false;
static float             g_fanPowerW  = 0;       // total fan power (measured / modelled)
static double            g_fanEnergyWh   = 0;    // integrated actual fan energy
static double            g_fanBaselineWh = 0;    // what fixed 100 % fans would have used
static SemaphoreHandle_t g_stateMutex;           // guards everything above
static QueueHandle_t     g_alertQueue;           // control -> telemetry

static Servo             g_servo[NUM_ZONES];
static HardwareSerial    SimSerial(1);           // UART1 for SIM800L
static volatile uint32_t g_tachPulses[NUM_ZONES] = {0};

// ----------------------------------------------------------------------------
//  3. FAN HARDWARE HELPERS
// ----------------------------------------------------------------------------
/** Tach interrupt: one count per falling edge. `arg` carries the zone index. */
static void IRAM_ATTR tachIsr(void *arg) {
  g_tachPulses[(uint32_t)(uintptr_t)arg]++;
}

static void fanHwInit() {
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
#if HIMADRI_CORE3
    ledcAttach(PIN_FAN_PWM[z], FAN_PWM_HZ, FAN_PWM_BITS);
#else
    // Channels 4-7 for fans; ESP32Servo takes the low channels (attach servos first).
    ledcSetup(4 + z, FAN_PWM_HZ, FAN_PWM_BITS);
    ledcAttachPin(PIN_FAN_PWM[z], 4 + z);
#endif
    pinMode(PIN_FAN_TACH[z], INPUT_PULLUP);
    attachInterruptArg(PIN_FAN_TACH[z], tachIsr, (void *)(uintptr_t)z, FALLING);
  }
}

/** Convert 0-100 % to a 25 kHz PWM duty. */
static void setFanPwm(uint8_t z, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  uint32_t duty = (uint32_t)(pct / 100.0f * ((1u << FAN_PWM_BITS) - 1));
#if HIMADRI_CORE3
  ledcWrite(PIN_FAN_PWM[z], duty);
#else
  ledcWrite(4 + z, duty);
#endif
}

// ----------------------------------------------------------------------------
//  4. SENSOR ABSTRACTION  (real drivers OR simulator, same interface)
// ----------------------------------------------------------------------------
#if !SIMULATE_SENSORS
static Adafruit_SHT31  sht35;
static SCD30           scd30;
static Adafruit_INA219 ina219;
#endif

/** Route the I2C bus to one zone's sensor pair via the TCA9548A multiplexer. */
static void selectMuxChannel(uint8_t ch) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/**
 * Read one zone. Returns false if the sensors did not answer.
 * `prev` = last published zone data (fan speed, damper state), `pcm` = wall temp.
 * The simulator uses them to close the loop: fans really cool the zone in simulation.
 */
static bool readZone(uint8_t z, const ZoneData &prev, float pcm,
                     float &t, float &h, float &co2) {
#if SIMULATE_SENSORS
  const float dt = SENSOR_PERIOD_MS / 1000.0f;
  static float simT[NUM_ZONES] = {6, 6, 6, 6};
  static float rot = 0;                              // Zone C: extra CO2 from rotting produce

  // First-order thermal plant: heat leaks in from 12 C ambient, moving air over the
  // cold glycol wall takes it out. More fan = more heat removed (and more fan power).
  float rotHeat = (z == 2 && millis() > 60000) ? 0.02f : 0.0f;
  simT[z] += dt * (0.010f * (12.0f - simT[z])
                   - 0.021f * (prev.fanPct / 100.0f) * (simT[z] - pcm) + rotHeat);
  t = simT[z] + random(-8, 8) / 100.0f;

  h = 90.0f - 0.03f * prev.fanPct + random(-10, 10) / 10.0f;   // faster air dries slightly

  co2 = 600.0f + 40.0f * z + random(-15, 15);
  if (z == 2 && millis() > 30000) {                  // rot starts after 30 s
    rot = max(0.0f, rot + (prev.damperOpen ? -14.0f : 8.0f) * dt);
    co2 += rot;                                      // venting flushes it, source keeps producing
  }
  return true;
#else
  selectMuxChannel(z);
  t = sht35.readTemperature();
  h = sht35.readHumidity();
  if (isnan(t) || isnan(h)) return false;
  if (!scd30.dataAvailable()) {              // SCD30 refreshes every ~2 s;
    co2 = prev.co2ppm;                       // reuse last value if no new sample.
    return !isnan(co2);
  }
  co2 = scd30.getCO2();
  return true;
#endif
}

/** Thermal-battery wall temperature from NTC on ADC (simulated if needed). */
static float readPcmTemp() {
#if SIMULATE_SENSORS
  static float t = 4.0f;
  t += g_compressorOn ? -0.05f : +0.01f;
  t = constrain(t, -3.0f, 7.0f);
  return t;
#else
  int raw = analogRead(PIN_ADC_PCM_NTC);
  float r  = 10000.0f * raw / (4095.0f - raw);
  float k  = 1.0f / (logf(r / 10000.0f) / 3950.0f + 1.0f / 298.15f);
  return k - 273.15f;
#endif
}

/** Day mode = solar panel producing enough voltage to run the compressor. */
static bool readDayMode() {
#if SIMULATE_SENSORS
  return ((millis() / 60000UL) % 2) == 0;    // flip day/night each minute in demo
#else
  float v = analogReadMilliVolts(PIN_ADC_SOLAR) / 1000.0f * 11.0f; // 1:11 divider
  return v > SOLAR_DAY_VOLT;
#endif
}

// ----------------------------------------------------------------------------
//  5. TASK 1 - SENSOR READING (+ fan tach + fan power + energy integration)
// ----------------------------------------------------------------------------
static void sensorTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t   lastMs = millis();
  [[maybe_unused]] uint32_t lastPulses[NUM_ZONES] = {0};

  for (;;) {
    // Snapshot what the simulator / power model needs (one short lock).
    ZoneData snap[NUM_ZONES];
    float    pcmSnap;
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    for (uint8_t z = 0; z < NUM_ZONES; z++) snap[z] = g_zones[z];
    pcmSnap = g_pcmTempC;
    xSemaphoreGive(g_stateMutex);

    // Slow I2C reads happen OUTSIDE the lock so the control task is never blocked.
    float t[NUM_ZONES], h[NUM_ZONES], c[NUM_ZONES]; bool ok[NUM_ZONES];
    for (uint8_t z = 0; z < NUM_ZONES; z++) ok[z] = readZone(z, snap[z], pcmSnap, t[z], h[z], c[z]);

    // Fan RPM from tach pulses counted since the last cycle.
    uint32_t now = millis();
    float dtS = max(1u, now - lastMs) / 1000.0f;
    lastMs = now;
    float rpm[NUM_ZONES];
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
#if SIMULATE_SENSORS
      rpm[z] = snap[z].fanPct > 1 ? FAN_MAX_RPM * snap[z].fanPct / 100.0f * (1.0f + random(-15, 15) / 1000.0f) : 0;
#else
      uint32_t p = g_tachPulses[z];
      rpm[z] = (p - lastPulses[z]) * 60.0f / (FAN_TACH_PPR * dtS);
      lastPulses[z] = p;
#endif
    }

    // Total fan power: INA219 on the fan rail, or the cube-law model in simulation.
    float powerW = 0, baselineW = NUM_ZONES * FAN_RATED_W;
#if SIMULATE_SENSORS
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
      float s = snap[z].fanPct / 100.0f;
      powerW += FAN_RATED_W * s * s * s;
    }
#else
    powerW = ina219.getPower_mW() / 1000.0f;
#endif

    float pcm = readPcmTemp();
    bool  day = readDayMode();

    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
      g_zones[z].valid = ok[z];
      if (ok[z]) { g_zones[z].tempC = t[z]; g_zones[z].humPct = h[z]; g_zones[z].co2ppm = c[z]; }
      g_zones[z].fanRpm = rpm[z];
    }
    g_pcmTempC = pcm;
    g_dayMode  = day;
    g_fanPowerW = powerW;
    g_fanEnergyWh   += powerW    * dtS / 3600.0;   // W * s -> Wh
    g_fanBaselineWh += baselineW * dtS / 3600.0;
    xSemaphoreGive(g_stateMutex);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));   // fixed-rate loop
  }
}

// ----------------------------------------------------------------------------
//  6. FAN CONTROLLER  (PI on temperature + humidity trim)
// ----------------------------------------------------------------------------
/**
 * Returns the commanded fan speed (%) for one zone.
 *  - ventOverride : damper open (rot / warm alarm) -> full speed to flush the zone
 *  - PI           : error = T - setpoint; deadband; integral clamped (anti-windup)
 *  - humidity trim: dry air -> slower (protect produce); wet air -> faster
 *  - limits       : 25 % floor, 50 % cap at night, 10 %/s slew limit
 */
static float fanController(uint8_t z, float temp, float rh, bool day,
                           bool ventOverride, float dt) {
  FanPI &c = g_fanPI[z];
  if (ventOverride) { c.out = 100.0f; return c.out; }

  float err = temp - FAN_SETPOINT_C;
  if (fabsf(err) < FAN_DEADBAND_C) err = 0;
  c.integ = constrain(c.integ + err * dt, FAN_INT_MIN, FAN_INT_MAX);

  float target = FAN_BIAS_PCT + FAN_KP * err + FAN_KI * c.integ;
  if      (rh < RH_DRY_PCT) target *= 0.7f;
  else if (rh > RH_WET_PCT) target += 15.0f;

  target = constrain(target, FAN_MIN_PCT, day ? FAN_MAX_DAY_PCT : FAN_MAX_NIGHT_PCT);

  float step = FAN_SLEW_PCT_S * dt;                       // slew-rate limit
  c.out += constrain(target - c.out, -step, step);
  return c.out;
}

// ----------------------------------------------------------------------------
//  7. TASK 2 - LOGIC CONTROL  (the "bio-aware" brain + fans)
// ----------------------------------------------------------------------------
static void setDamper(uint8_t z, bool open) {
  g_servo[z].write(open ? SERVO_OPEN_DEG : SERVO_CLOSED_DEG);
}

static void controlTask(void *) {
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  uint32_t lastSms[NUM_ZONES]    = {0};        // per-zone alert rate limiters
  uint32_t lastFanSms[NUM_ZONES] = {0};
  bool     alarmLatched[NUM_ZONES] = {false};
  uint8_t  fanFailCnt[NUM_ZONES] = {0};
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    uint32_t now = millis();

    // ---- A. COMPRESSOR / THERMAL BATTERY (Day mode only) -------------------
    bool wantCompressor = g_dayMode && (g_pcmTempC > PCM_FROZEN_C);
    if (wantCompressor != g_compressorOn) {
      g_compressorOn = wantCompressor;
      digitalWrite(PIN_RELAY_COMP, g_compressorOn ? HIGH : LOW);
    }

    for (uint8_t z = 0; z < NUM_ZONES; z++) {
      ZoneData &zd = g_zones[z];

      // ---- B. ZONAL BIO-AWARE VENTING --------------------------------------
      if (zd.valid) {                      // never act on stale/failed data
        bool rot  = zd.co2ppm > CO2_LIMIT_PPM;
        bool warm = zd.tempC  > TEMP_LIMIT_C;
        bool clearRot  = zd.co2ppm < (CO2_LIMIT_PPM - HYSTERESIS_CO2_PPM);
        bool clearWarm = zd.tempC  < (TEMP_LIMIT_C  - HYSTERESIS_TEMP_C);

        if ((rot || warm) && !alarmLatched[z]) alarmLatched[z] = true;
        if (alarmLatched[z] && clearRot && clearWarm) alarmLatched[z] = false;

        if (alarmLatched[z] != zd.damperOpen) {
          zd.damperOpen = alarmLatched[z];
          setDamper(z, zd.damperOpen);     // vent heat + ethylene from THIS zone only
        }

        if (alarmLatched[z] && (lastSms[z] == 0 || now - lastSms[z] > SMS_COOLDOWN_MS)) {
          Alert a{ rot ? ALERT_ROT : ALERT_WARM, z, zd.tempC, zd.co2ppm };
          if (xQueueSend(g_alertQueue, &a, 0) == pdTRUE) lastSms[z] = now;
        }
        if (!alarmLatched[z]) lastSms[z] = 0;   // re-arm after recovery

        // ---- C. ENERGY-OPTIMISED FAN SPEED ---------------------------------
        zd.fanPct = fanController(z, zd.tempC, zd.humPct, g_dayMode, zd.damperOpen, dt);
      } else {
        zd.fanPct = FAN_FAILSAFE_PCT;      // sensor lost: safe moderate airflow
      }
      setFanPwm(z, zd.fanPct);

      // ---- D. FAN FAULT DETECTION (tach feedback) --------------------------
      // Commanded well above zero but the tach shows (almost) no rotation.
      if (zd.fanPct >= 30.0f && zd.fanRpm < FAN_FAIL_RPM) {
        if (fanFailCnt[z] < 255) fanFailCnt[z]++;
      } else fanFailCnt[z] = 0;
      if (fanFailCnt[z] >= FAN_FAIL_CYCLES &&
          (lastFanSms[z] == 0 || now - lastFanSms[z] > SMS_COOLDOWN_MS)) {
        Alert a{ ALERT_FAN_FAIL, z, zd.tempC, zd.co2ppm };
        if (xQueueSend(g_alertQueue, &a, 0) == pdTRUE) lastFanSms[z] = now;
      }
    }

    xSemaphoreGive(g_stateMutex);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// ----------------------------------------------------------------------------
//  8. TASK 3 - TELEMETRY (SIM800L SMS via AT commands)
// ----------------------------------------------------------------------------
static bool waitFor(const char *expect, uint32_t timeoutMs) {
  String buf;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (SimSerial.available()) buf += (char)SimSerial.read();
    if (buf.indexOf(expect) >= 0) return true;
    vTaskDelay(pdMS_TO_TICKS(20));          // yield to other tasks while waiting
  }
  return false;
}

static bool sendAt(const char *cmd, const char *expect = "OK", uint32_t to = 2000) {
  SimSerial.println(cmd);
  return waitFor(expect, to);
}

/** Send one text-mode SMS. Standard sequence: AT+CMGF=1 -> AT+CMGS -> text -> Ctrl+Z */
static bool sendSms(const String &text) {
  if (!sendAt("AT"))            return false;
  if (!sendAt("AT+CMGF=1"))     return false;
  SimSerial.print("AT+CMGS=\"");
  SimSerial.print(SMS_RECIPIENT);
  SimSerial.println("\"");
  if (!waitFor(">", 3000))      return false;
  SimSerial.print(text);
  SimSerial.write(0x1A);                        // Ctrl+Z = "send"
  return waitFor("+CMGS", 15000);
}

static String buildMessage(const Alert &a) {
  String m = "HIMADRI ALERT Zone ";
  m += ZONE_NAME[a.zone];
  if (a.type == ALERT_FAN_FAIL) {
    m += ": FAN FAULT (no tach). Please inspect.";
    return m;
  }
  m += a.type == ALERT_ROT ? ": ROT DETECTED. " : ": TEMP HIGH. ";
  m += "CO2=" + String((int)a.co2ppm) + "ppm, T=" + String(a.tempC, 1) + "C. ";
  m += "Damper opened. Please inspect.";
  return m;                                     // kept < 160 chars = single SMS
}

static void telemetryTask(void *) {
  SimSerial.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  Alert a;
  for (;;) {
    if (xQueueReceive(g_alertQueue, &a, portMAX_DELAY) == pdTRUE) {
      String msg = buildMessage(a);
      Serial.println("[SMS] " + msg);
#if SIMULATE_SENSORS
      Serial.println("[SMS] (simulation: modem not used)");
#else
      for (int tries = 0; tries < 3; tries++) {   // retry: weak signal is common
        if (sendSms(msg)) { Serial.println("[SMS] sent"); break; }
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
#endif
    }
  }
}

// ----------------------------------------------------------------------------
//  9. SETUP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY_COMP, OUTPUT);
  digitalWrite(PIN_RELAY_COMP, LOW);            // fail-safe: compressor OFF at boot

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  analogReadResolution(12);

  for (uint8_t z = 0; z < NUM_ZONES; z++) {     // servos first (they claim low LEDC channels)
    g_servo[z].attach(PIN_SERVO[z]);
    setDamper(z, false);                        // all dampers closed at boot
  }
  fanHwInit();
  for (uint8_t z = 0; z < NUM_ZONES; z++) setFanPwm(z, FAN_FAILSAFE_PCT);  // safe airflow at boot

#if !SIMULATE_SENSORS
  sht35.begin(0x44);
  scd30.begin(Wire);
  scd30.setMeasurementInterval(2);
  ina219.begin();
#endif

  g_stateMutex = xSemaphoreCreateMutex();
  g_alertQueue = xQueueCreate(8, sizeof(Alert));

  // Priorities: control (3) > sensor (2) > telemetry (1). Safety always wins.
  xTaskCreatePinnedToCore(sensorTask,    "sensor",    4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(controlTask,   "control",   4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 6144, nullptr, 1, nullptr, 0);

  Serial.println("Himadri firmware v2 started");
}

void loop() {
  // Everything runs in FreeRTOS tasks. loop() only prints a 5 s status line.
  vTaskDelay(pdMS_TO_TICKS(5000));
  xSemaphoreTake(g_stateMutex, portMAX_DELAY);
  Serial.printf("[%s] PCM=%.1fC comp=%d | fan %.2fW  used %.3fWh vs %.3fWh fixed-100%% (saved %.0f%%)\n",
                g_dayMode ? "DAY" : "NIGHT", g_pcmTempC, g_compressorOn, g_fanPowerW,
                g_fanEnergyWh, g_fanBaselineWh,
                g_fanBaselineWh > 0 ? 100.0 * (1.0 - g_fanEnergyWh / g_fanBaselineWh) : 0.0);
  for (uint8_t z = 0; z < NUM_ZONES; z++)
    Serial.printf("   %c: %.1fC %.0f%%RH %.0fppm | fan %.0f%% %.0frpm | damper %s\n",
                  ZONE_NAME[z], g_zones[z].tempC, g_zones[z].humPct, g_zones[z].co2ppm,
                  g_zones[z].fanPct, g_zones[z].fanRpm, g_zones[z].damperOpen ? "OPEN" : "ok");
  xSemaphoreGive(g_stateMutex);
}
