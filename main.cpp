/**
 * ============================================================================
 *  PROJECT HIMADRI  -  Bio-Aware Thermal Cold Storage        (SIH 2026 | SIH26005)
 *  Firmware: ESP32-S3 + FreeRTOS
 * ============================================================================
 *  Three concurrent tasks, one shared state:
 *
 *   [sensorTask]  --writes-->  g_zones[] (mutex)  --read by-->  [controlTask]
 *                                                                    |
 *                                              alertQueue (FreeRTOS queue)
 *                                                                    v
 *                                                            [telemetryTask]
 *                                                      (SIM800L AT-command SMS)
 *
 *  Why FreeRTOS?  A blocking SMS (can take 5-10 s on a weak rural network)
 *  must NEVER delay the safety loop that opens dampers / switches the compressor.
 *  Separate tasks give us deterministic control + slow telemetry in parallel.
 *
 *  BUILD MODES
 *   -DSIMULATE_SENSORS=1  (default) : no hardware needed, fake sensor data.
 *                                     Ideal for jury demos & unit testing.
 *   -DSIMULATE_SENSORS=0            : real drivers. Needs libraries:
 *                                     Adafruit SHT31 Library (works for SHT35, 0x44),
 *                                     SparkFun SCD30 Arduino Library, ESP32Servo.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>   // ESP32 has spare hardware UARTs -> more reliable than
                              // SoftwareSerial at SIM800L's baud rate.
#include <ESP32Servo.h>

#ifndef SIMULATE_SENSORS
#define SIMULATE_SENSORS 1
#endif

#if !SIMULATE_SENSORS
  #include <Adafruit_SHT31.h>   // SHT35 is register-compatible with SHT31
  #include <SparkFun_SCD30_Arduino_Library.h>
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

// Pins (ESP32-S3-DevKitC). Adjust to your PCB.
constexpr int PIN_I2C_SDA      = 8;
constexpr int PIN_I2C_SCL      = 9;
constexpr int PIN_RELAY_COMP   = 4;              // compressor relay (active HIGH)
constexpr int PIN_SERVO[NUM_ZONES] = {5, 6, 7, 15};
constexpr int PIN_SIM_TX       = 17;             // ESP32 TX -> SIM800L RX
constexpr int PIN_SIM_RX       = 18;             // ESP32 RX <- SIM800L TX
constexpr int PIN_ADC_SOLAR    = 1;              // solar voltage divider
constexpr int PIN_ADC_PCM_NTC  = 2;              // NTC thermistor glued to glycol wall
constexpr uint8_t TCA9548A_ADDR = 0x70;          // I2C mux: 1 SHT35+SCD30 pair per zone
                                                  // (both parts have fixed I2C addresses)

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
  bool  damperOpen = false;
  bool  valid   = false;   // false until the first successful read
};

enum AlertType : uint8_t { ALERT_ROT, ALERT_WARM, ALERT_SENSOR_FAIL };

struct Alert {
  AlertType type;
  uint8_t   zone;
  float     tempC;
  float     co2ppm;
};

static ZoneData          g_zones[NUM_ZONES];
static float             g_pcmTempC   = 4.0f;    // thermal-battery wall temperature
static bool              g_dayMode    = false;
static bool              g_compressorOn = false;
static SemaphoreHandle_t g_stateMutex;           // guards g_zones + globals above
static QueueHandle_t     g_alertQueue;           // control -> telemetry

static Servo             g_servo[NUM_ZONES];
static HardwareSerial    SimSerial(1);           // UART1 for SIM800L

// ----------------------------------------------------------------------------
//  3. SENSOR ABSTRACTION  (real drivers OR simulator, same interface)
// ----------------------------------------------------------------------------
#if !SIMULATE_SENSORS
static Adafruit_SHT31 sht35;
static SCD30          scd30;
#endif

/** Route the I2C bus to one zone's sensor pair via the TCA9548A multiplexer. */
static void selectMuxChannel(uint8_t ch) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/** Read one zone. Returns false if the sensors did not answer. */
static bool readZone(uint8_t z, float &t, float &h, float &co2) {
#if SIMULATE_SENSORS
  // Simulator: gentle drift + Zone C slowly "rots" after ~60 s so you can demo
  // the full detect -> vent -> SMS chain without any hardware.
  float base = 5.0f + 0.5f * sinf(millis() / 20000.0f + z);
  t   = base + (random(-10, 10) / 50.0f);
  h   = 88.0f + random(-20, 20) / 10.0f;
  co2 = 600.0f + random(-30, 30);
  if (z == 2 && millis() > 60000) {
    co2 += (millis() - 60000) / 30.0f;      // climbing CO2 (respiration of rot)
    t   += 1.5f;                             // microbial heat
  }
  return true;
#else
  selectMuxChannel(z);
  t = sht35.readTemperature();
  h = sht35.readHumidity();
  if (isnan(t) || isnan(h)) return false;
  if (!scd30.dataAvailable()) {              // SCD30 only refreshes every ~2 s;
    co2 = g_zones[z].co2ppm;                 // reuse last value if no new sample.
    return !isnan(co2);
  }
  co2 = scd30.getCO2();
  return true;
#endif
}

/** Thermal-battery wall temperature from NTC on ADC (simulated if needed). */
static float readPcmTemp() {
#if SIMULATE_SENSORS
  // Fake charge/discharge: falls when compressor runs, rises when it is off.
  static float t = 4.0f;
  t += g_compressorOn ? -0.05f : +0.01f;
  t = constrain(t, -3.0f, 7.0f);
  return t;
#else
  // 10k NTC, Beta=3950, 10k series resistor, 12-bit ADC (Steinhart-Hart simplified)
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
//  4. TASK 1 - SENSOR READING
// ----------------------------------------------------------------------------
static void sensorTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
      float t, h, c;
      bool ok = readZone(z, t, h, c);

      xSemaphoreTake(g_stateMutex, portMAX_DELAY);
      g_zones[z].valid = ok;
      if (ok) { g_zones[z].tempC = t; g_zones[z].humPct = h; g_zones[z].co2ppm = c; }
      xSemaphoreGive(g_stateMutex);
    }

    float pcm = readPcmTemp();
    bool  day = readDayMode();
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_pcmTempC = pcm;
    g_dayMode  = day;
    xSemaphoreGive(g_stateMutex);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));   // fixed-rate loop
  }
}

// ----------------------------------------------------------------------------
//  5. TASK 2 - LOGIC CONTROL  (the "bio-aware" brain)
// ----------------------------------------------------------------------------
static void setDamper(uint8_t z, bool open) {
  g_servo[z].write(open ? SERVO_OPEN_DEG : SERVO_CLOSED_DEG);
}

static void controlTask(void *) {
  uint32_t lastSms[NUM_ZONES] = {0};       // per-zone alert rate limiter
  bool     alarmLatched[NUM_ZONES] = {false};
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);

    // ---- A. COMPRESSOR / THERMAL BATTERY (Day mode only) -------------------
    // Battery-less design: the compressor runs ONLY when solar is available,
    // and only until the glycol wall is fully frozen (<= -2 C). At night it is
    // always OFF and the frozen wall passively holds 4-8 C.
    bool wantCompressor = g_dayMode && (g_pcmTempC > PCM_FROZEN_C);
    if (wantCompressor != g_compressorOn) {
      g_compressorOn = wantCompressor;
      digitalWrite(PIN_RELAY_COMP, g_compressorOn ? HIGH : LOW);
    }

    // ---- B. ZONAL BIO-AWARE VENTING ---------------------------------------
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
      ZoneData &zd = g_zones[z];
      if (!zd.valid) continue;             // never act on stale/failed data

      bool rot  = zd.co2ppm > CO2_LIMIT_PPM;
      bool warm = zd.tempC  > TEMP_LIMIT_C;

      // Hysteresis: open at the limit, close only when safely below it.
      bool clearRot  = zd.co2ppm < (CO2_LIMIT_PPM - HYSTERESIS_CO2_PPM);
      bool clearWarm = zd.tempC  < (TEMP_LIMIT_C  - HYSTERESIS_TEMP_C);

      if ((rot || warm) && !alarmLatched[z]) alarmLatched[z] = true;
      if (alarmLatched[z] && clearRot && clearWarm) alarmLatched[z] = false;

      if (alarmLatched[z] != zd.damperOpen) {
        zd.damperOpen = alarmLatched[z];
        setDamper(z, zd.damperOpen);       // vent heat + ethylene from THIS zone only
      }

      // ---- C. RAISE ALERT (rate-limited) --------------------------------
      uint32_t now = millis();
      if (alarmLatched[z] && (lastSms[z] == 0 || now - lastSms[z] > SMS_COOLDOWN_MS)) {
        Alert a{ rot ? ALERT_ROT : ALERT_WARM, z, zd.tempC, zd.co2ppm };
        if (xQueueSend(g_alertQueue, &a, 0) == pdTRUE) lastSms[z] = now;
      }
      if (!alarmLatched[z]) lastSms[z] = 0;   // re-arm after recovery
    }

    xSemaphoreGive(g_stateMutex);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// ----------------------------------------------------------------------------
//  6. TASK 3 - TELEMETRY (SIM800L SMS via AT commands)
// ----------------------------------------------------------------------------

/** Wait until `expect` appears on the modem UART, or time out. */
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
  if (!sendAt("AT"))            return false;   // modem alive?
  if (!sendAt("AT+CMGF=1"))     return false;   // text mode
  SimSerial.print("AT+CMGS=\"");
  SimSerial.print(SMS_RECIPIENT);
  SimSerial.println("\"");
  if (!waitFor(">", 3000))      return false;   // modem prompt for message body
  SimSerial.print(text);
  SimSerial.write(0x1A);                        // Ctrl+Z = "send"
  return waitFor("+CMGS", 15000);               // network can be slow in rural areas
}

static String buildMessage(const Alert &a) {
  String m = "HIMADRI ALERT Zone ";
  m += ZONE_NAME[a.zone];
  m += a.type == ALERT_ROT ? ": ROT DETECTED. " : ": TEMP HIGH. ";
  m += "CO2=" + String((int)a.co2ppm) + "ppm, T=" + String(a.tempC, 1) + "C. ";
  m += "Damper opened. Please inspect.";
  return m;                                     // kept < 160 chars = single SMS
}

static void telemetryTask(void *) {
  SimSerial.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  Alert a;
  for (;;) {
    // Block here (zero CPU) until the control task queues an alert.
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
//  7. SETUP  (create hardware, mutex, queue, tasks)
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY_COMP, OUTPUT);
  digitalWrite(PIN_RELAY_COMP, LOW);            // fail-safe: compressor OFF at boot

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  analogReadResolution(12);

  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    g_servo[z].attach(PIN_SERVO[z]);
    setDamper(z, false);                        // all dampers closed at boot
  }

#if !SIMULATE_SENSORS
  sht35.begin(0x44);
  scd30.begin(Wire);
  scd30.setMeasurementInterval(2);
#endif

  g_stateMutex = xSemaphoreCreateMutex();
  g_alertQueue = xQueueCreate(8, sizeof(Alert));

  // Priorities: control (3) > sensor (2) > telemetry (1). Safety always wins.
  xTaskCreatePinnedToCore(sensorTask,    "sensor",    4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(controlTask,   "control",   4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 6144, nullptr, 1, nullptr, 0);

  Serial.println("Himadri firmware started");
}

void loop() {
  // Everything runs in FreeRTOS tasks. loop() only prints a 5 s status line.
  vTaskDelay(pdMS_TO_TICKS(5000));
  xSemaphoreTake(g_stateMutex, portMAX_DELAY);
  Serial.printf("[%s] PCM=%.1fC comp=%d |", g_dayMode ? "DAY" : "NIGHT",
                g_pcmTempC, g_compressorOn);
  for (uint8_t z = 0; z < NUM_ZONES; z++)
    Serial.printf(" %c: %.1fC %.0fppm %s |", ZONE_NAME[z], g_zones[z].tempC,
                  g_zones[z].co2ppm, g_zones[z].damperOpen ? "OPEN" : "ok");
  Serial.println();
  xSemaphoreGive(g_stateMutex);
}
