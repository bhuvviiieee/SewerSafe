/*
 * ================================================================
 *   ESP32 AIR QUALITY MONITOR  —  Full PPM Edition
 *   Sensors : BME680 (Temp / Humidity / Pressure)
 *             MICS6814 Breakout (NH3 / CO / VOC)
 *             Winsen MP-4 Breakout (CH4 / Ethanol analog proxy)
 *
 *   OUTPUTS:
 *     GREEN  LED  → GPIO25  — All safe
 *     YELLOW LED  → GPIO26  — Humidity out of range (sensors unreliable)
 *     RED    LED  → GPIO14  — Gas alarm (blinks WITH buzzer, then stays solid)
 *     Buzzer      → GPIO27  — Via 2N2222 NPN + 1kΩ resistor
 *
 *   CIRCUIT CONNECTIONS (matches schematic):
 *   ─────────────────────────────────────────────────────────────
 *   BME680 Breakout:
 *     VCC → 3.3V  |  GND → GND
 *     SDA → GPIO21 (ESP32 #21)
 *     SCL → GPIO22 (ESP32 #22)
 *
 *   MICS6814 Breakout:
 *     VCC     → 3.3V  |  GND → GND
 *     NH3_OUT → GPIO35  (ADC1 CH7 — input only pin)
 *     CO_OUT  → GPIO34  (ADC1 CH6 — input only pin)
 *     VOC_OUT → GPIO32  (ADC1 CH4)
 *
 *   Winsen MP-4 Breakout:
 *     VCC  → 3.3V  |  GND → GND
 *     AOUT → GPIO33  (ADC1 CH5 — matches schematic)
 *
 *   Buzzer (via 2N2222 NPN):
 *     GPIO27 → R1 1kΩ → Base of Q2 (2N2222)
 *     Collector → Buzzer (–)
 *     Buzzer (+) → 5V (VIN rail)
 *     Emitter → GND
 *
 *   LEDs (add to free GPIO pins with 220Ω resistor to GND each):
 *     GPIO25 → 220Ω → Green  LED anode → cathode → GND
 *     GPIO26 → 220Ω → Yellow LED anode → cathode → GND
 *     GPIO14 → 220Ω → Red    LED anode → cathode → GND
 *
 *   VOLTAGE SAFETY CHECK:
 *     All sensor VCC = 3.3V  (ESP32 GPIO logic = 3.3V)
 *     Buzzer powered from 5V via NPN switch — GPIO never sees 5V
 *     ADC1 pins only (32–39) — no ADC2 conflict with WiFi
 *     GPIO33/34/35 are INPUT-ONLY — never set as OUTPUT
 *     TP4056 OUT+ → ESP32 VIN (5V rail)
 *
 *   LIBRARIES (Arduino Library Manager):
 *     Adafruit BME680 Library
 *     Adafruit Unified Sensor
 * ================================================================
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// ─── PIN DEFINITIONS ──────────────────────────────────────────
#define PIN_BUZZER       27
#define PIN_LED_GREEN    25
#define PIN_LED_YELLOW   26
#define PIN_LED_RED      14

#define PIN_NH3_OUT      35   // MICS6814 NH3  (ADC1 CH7)
#define PIN_CO_OUT       34   // MICS6814 CO   (ADC1 CH6)
#define PIN_VOC_OUT      32   // MICS6814 VOC  (ADC1 CH4)
#define PIN_MP4_AOUT     33   // Winsen MP-4   (ADC1 CH5)

// ─── MICS6814 CALIBRATION CONSTANTS ──────────────────────────
/*
 *  The MICS6814 outputs a voltage from a resistor divider:
 *    Vout = VCC * RL / (Rs + RL)   where RL = 10kΩ (on breakout board)
 *
 *  Sensor resistance:  Rs = RL * (VCC / Vout - 1)
 *
 *  PPM from sensitivity curve (log-log from datasheet):
 *    ppm = A * pow(Rs/R0, B)
 *
 *  R0 = sensor resistance measured in CLEAN AIR after 30-min warm-up.
 *  UPDATE the three R0 values after running calibration (see bottom notes).
 */
#define VCC_SENSOR   3.3f    // Sensor supply voltage (V)
#define RL_KOHM      10.0f   // Load resistor on MICS6814 breakout (kΩ)

// R0 in clean air (kΩ) — MEASURE THESE after warm-up and update!
float R0_NH3 = 30.0f;   // Typical range: 20–80  kΩ
float R0_CO  = 75.0f;   // Typical range: 50–100 kΩ
float R0_VOC = 25.0f;   // Typical range: 10–60  kΩ

// ─── MP-4 CALIBRATION (Ethanol proxy, linear mapping) ────────
/*
 *  MP-4 AOUT voltage rises with gas concentration.
 *  Map: ADC_CLEAN (clean air) → 0 ppm
 *       ADC_MAX               → MP4_PPM_MAX
 *  Adjust MP4_ADC_CLEAN after observing baseline in fresh air.
 */
#define MP4_ADC_CLEAN   400      // ADC count in clean air — calibrate!
#define MP4_ADC_MAX    3800      // ADC count at MP4_PPM_MAX
#define MP4_PPM_MAX    1000.0f   // ppm at ADC_MAX

// ─── HUMAN-HEALTH THRESHOLDS (ppm) ───────────────────────────
/*
 *  Sources: OSHA PEL, NIOSH REL/STEL, ACGIH TLV, WHO
 *
 *  NH3 (Ammonia):
 *    25 ppm  — OSHA PEL 8-hr TWA         → WARN
 *    35 ppm  — NIOSH STEL 15-min limit   → ALARM
 *   300 ppm  — IDLH (life threatening)
 *
 *  CO (Carbon Monoxide):
 *    35 ppm  — OSHA PEL 8-hr TWA         → WARN
 *   200 ppm  — NIOSH STEL                → ALARM
 *  1200 ppm  — IDLH
 *
 *  Ethanol vapor / VOC:
 *   100 ppm  — OSHA PEL ethanol 8-hr TWA → WARN
 *   400 ppm  — Eye/nose irritation onset  → ALARM
 *  3300 ppm  — IDLH ethanol
 *
 *  Humidity (sensor accuracy, not direct health risk):
 *    >85% RH  — MICS6814 accuracy degrades → YELLOW
 *    <10% RH  — Below operating range      → YELLOW
 */

// NH3 ppm thresholds
#define NH3_WARN_PPM      25.0f
#define NH3_ALARM_PPM     35.0f

// CO ppm thresholds
#define CO_WARN_PPM       35.0f
#define CO_ALARM_PPM     200.0f

// Ethanol / VOC ppm thresholds
#define ETHANOL_WARN_PPM  100.0f
#define ETHANOL_ALARM_PPM 400.0f

// Humidity % thresholds
#define HUMIDITY_HIGH     85.0f
#define HUMIDITY_LOW      10.0f

// ─── TIMING ───────────────────────────────────────────────────
#define READ_INTERVAL_MS  2000   // Full read cycle every 2 s
#define BEEP_ON_MS         200   // Buzzer / LED on-time per pulse
#define BEEP_OFF_MS        300   // Gap between pulses

// ─── OBJECTS & STATE ──────────────────────────────────────────
Adafruit_BME680 bme;
bool          bmeReady = false;
unsigned long lastMs   = 0;

// ─── FORWARD DECLARATIONS ─────────────────────────────────────
int   averageADC(uint8_t pin, int n);
float adcToVoltage(int raw);
float voltageToRs(float v);
float rsToPpm_NH3(float rs);
float rsToPpm_CO(float rs);
float rsToPpm_VOC(float rs);
float adcToEthanolPpm(int raw);
void  setStatusSafe();
void  setStatusYellow();
void  setStatusAlarm(int beeps);
void  blinkLED(uint8_t pin, int n, int ms);
void  printGasRow(const char* lbl, int raw, float v, float rs,
                  float ppm, float wPpm, float aPpm);

// =============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n================================================"));
  Serial.println(F("  ESP32 Air Quality Monitor  —  PPM Edition"));
  Serial.println(F("================================================"));

  // Output pins
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);

  // Analog input pins
  pinMode(PIN_NH3_OUT,  INPUT);
  pinMode(PIN_CO_OUT,   INPUT);
  pinMode(PIN_VOC_OUT,  INPUT);
  pinMode(PIN_MP4_AOUT, INPUT);

  // 12-bit ADC, 0–3.3V full scale
  // 12-bit ADC (0-4095)
  analogReadResolution(12); 

  // This sets the 0-3.3V range (11dB attenuation)
  analogSetAttenuation(ADC_11db);

  // All outputs off
  digitalWrite(PIN_BUZZER,     LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);

  // ── LED + Buzzer self-test ──────────────────────────────────
  Serial.println(F("[INIT] Self-test: Green → Yellow → Red+Buzzer"));
  blinkLED(PIN_LED_GREEN,  2, 200);
  blinkLED(PIN_LED_YELLOW, 2, 200);
  // Red LED and buzzer together (alarm preview)
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER,  HIGH);
    delay(BEEP_ON_MS);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER,  LOW);
    delay(BEEP_OFF_MS);
  }
  Serial.println(F("[INIT] Self-test OK."));

  // ── BME680 init ────────────────────────────────────────────
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  if (!bme.begin(0x77) && !bme.begin(0x76)) {
    Serial.println(F("[ERROR] BME680 not found! Check SDA=GPIO21, SCL=GPIO22."));
    while (true) { blinkLED(PIN_LED_RED, 5, 100); delay(500); }
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);   // 320°C heater, 150 ms
  bmeReady = true;
  Serial.println(F("[INIT] BME680 initialized OK."));
  Serial.println(F("\n[NOTE] Allow 30-60 min warm-up for stable MICS6814 readings."));
  Serial.println(F("[NOTE] Calibrate R0_NH3 / R0_CO / R0_VOC in clean air first.\n"));
}

// =============================================================
void loop() {
  unsigned long now = millis();
  if (now - lastMs < (unsigned long)READ_INTERVAL_MS) return;
  lastMs = now;

  // ── 1. Read BME680 ─────────────────────────────────────────
  float temperature = 0, humidity = 0, pressure = 0;
  bool  bmeOk = false;
  if (bmeReady && bme.performReading()) {
    temperature = bme.temperature;
    humidity    = bme.humidity;
    pressure    = bme.pressure / 100.0f;
    bmeOk       = true;
  }

  // ── 2. Read raw ADC (10-sample average per channel) ────────
  int rawNH3     = averageADC(PIN_NH3_OUT,  10);
  int rawCO      = averageADC(PIN_CO_OUT,   10);
  int rawVOC     = averageADC(PIN_VOC_OUT,  10);
  int rawEthanol = averageADC(PIN_MP4_AOUT, 10);

  // ── 3. Convert → Voltage → Rs → ppm ───────────────────────
  float vNH3 = adcToVoltage(rawNH3);
  float vCO  = adcToVoltage(rawCO);
  float vVOC = adcToVoltage(rawVOC);

  float rsNH3 = voltageToRs(vNH3);
  float rsCO  = voltageToRs(vCO);
  float rsVOC = voltageToRs(vVOC);

  float ppmNH3     = rsToPpm_NH3(rsNH3);
  float ppmCO      = rsToPpm_CO(rsCO);
  float ppmVOC     = rsToPpm_VOC(rsVOC);
  float ppmEthanol = adcToEthanolPpm(rawEthanol);

  // ── 4. Serial output table ─────────────────────────────────
  Serial.println(F("────────────────────────────────────────────────────────────"));
  if (bmeOk) {
    Serial.printf("[BME680] Temp: %.1f C  |  Humidity: %.1f %%  |  Pressure: %.1f hPa\n",
                  temperature, humidity, pressure);
  } else {
    Serial.println(F("[BME680] Read failed — check I2C wiring."));
  }
  Serial.println(F(""));
  Serial.println(F("  Gas   | ADC  | Voltage |   Rs kΩ |    ppm   | Status"));
  Serial.println(F("  ------+------+---------+---------+----------+--------"));
  printGasRow("  NH3  ", rawNH3,     vNH3, rsNH3, ppmNH3,     NH3_WARN_PPM,     NH3_ALARM_PPM);
  printGasRow("  CO   ", rawCO,      vCO,  rsCO,  ppmCO,      CO_WARN_PPM,      CO_ALARM_PPM);
  printGasRow("  VOC  ", rawVOC,     vVOC, rsVOC, ppmVOC,     ETHANOL_WARN_PPM, ETHANOL_ALARM_PPM);
  Serial.printf("  ETH  | %4d |    —    |    —    | %8.1f | %s\n",
                rawEthanol, ppmEthanol,
                ppmEthanol >= ETHANOL_ALARM_PPM ? "ALARM" :
                ppmEthanol >= ETHANOL_WARN_PPM  ? "WARN " : "OK   ");
  Serial.println(F(""));

  // ── 5. Determine status level ──────────────────────────────
  /*
   *  PRIORITY (highest to lowest):
   *
   *  LEVEL 3 — ALARM  : Any gas >= ALARM ppm threshold
   *    Red LED blinks 3x IN SYNC with 3 buzzer beeps → Red stays ON solid
   *
   *  LEVEL 2 — WARN   : Any gas between WARN and ALARM ppm
   *    Red LED blinks 1x with 1 buzzer beep → Red stays ON solid
   *
   *  LEVEL 1 — HUM    : Humidity out of safe range (sensors unreliable)
   *    Yellow LED solid + 1 short buzzer beep
   *
   *  LEVEL 0 — SAFE   : All gases below WARN thresholds, humidity normal
   *    Green LED solid, all others OFF, buzzer silent
   */

  bool gasAlarm = (ppmNH3     >= NH3_ALARM_PPM)     ||
                  (ppmCO      >= CO_ALARM_PPM)       ||
                  (ppmEthanol >= ETHANOL_ALARM_PPM);

  bool gasWarn  = !gasAlarm && (
                  (ppmNH3     >= NH3_WARN_PPM)       ||
                  (ppmCO      >= CO_WARN_PPM)         ||
                  (ppmEthanol >= ETHANOL_WARN_PPM));

  bool humWarn  = bmeOk && (humidity > HUMIDITY_HIGH || humidity < HUMIDITY_LOW);

  if (gasAlarm) {
    // ── ALARM: dangerous gas levels ────────────────────────
    Serial.println(F("[STATUS] DANGER — Gas exceeds safe human exposure limit!"));
    if (ppmNH3     >= NH3_ALARM_PPM)
      Serial.printf("         NH3     = %.1f ppm  (ALARM > %.0f ppm — NIOSH STEL)\n",
                    ppmNH3, NH3_ALARM_PPM);
    if (ppmCO      >= CO_ALARM_PPM)
      Serial.printf("         CO      = %.1f ppm  (ALARM > %.0f ppm — NIOSH STEL)\n",
                    ppmCO, CO_ALARM_PPM);
    if (ppmEthanol >= ETHANOL_ALARM_PPM)
      Serial.printf("         Ethanol = %.1f ppm  (ALARM > %.0f ppm — irritation level)\n",
                    ppmEthanol, ETHANOL_ALARM_PPM);
    // Red LED blinks 3 times WITH buzzer, then red stays solid
    setStatusAlarm(3);

  } else if (gasWarn) {
    // ── WARN: approaching danger zone ──────────────────────
    Serial.println(F("[STATUS] WARNING — Gas approaching exposure limit."));
    if (ppmNH3     >= NH3_WARN_PPM)
      Serial.printf("         NH3     = %.1f ppm  (WARN > %.0f ppm — OSHA PEL)\n",
                    ppmNH3, NH3_WARN_PPM);
    if (ppmCO      >= CO_WARN_PPM)
      Serial.printf("         CO      = %.1f ppm  (WARN > %.0f ppm — OSHA PEL)\n",
                    ppmCO, CO_WARN_PPM);
    if (ppmEthanol >= ETHANOL_WARN_PPM)
      Serial.printf("         Ethanol = %.1f ppm  (WARN > %.0f ppm — OSHA PEL)\n",
                    ppmEthanol, ETHANOL_WARN_PPM);
    // Red LED blinks 1 time WITH buzzer, then red stays solid
    setStatusAlarm(1);

  } else if (humWarn) {
    // ── HUMIDITY: sensors unreliable ───────────────────────
    Serial.printf("[STATUS] HUMIDITY WARNING — %.1f%% RH. Sensor accuracy affected.\n",
                  humidity);
    setStatusYellow();

  } else {
    // ── SAFE ───────────────────────────────────────────────
    Serial.println(F("[STATUS] SAFE — All gases within normal limits."));
    setStatusSafe();
  }

  Serial.println(F(""));
}

// =============================================================
//  PPM CONVERSION FUNCTIONS
// =============================================================

// ADC 12-bit raw → voltage (0–3.3V)
float adcToVoltage(int raw) {
  return raw * (VCC_SENSOR / 4095.0f);
}

// Voltage divider inverse: Rs = RL * (VCC/Vout - 1)
float voltageToRs(float v) {
  if (v <= 0.001f) v = 0.001f;
  return RL_KOHM * ((VCC_SENSOR / v) - 1.0f);
}

/*
 *  Sensitivity curve coefficients (from MICS6814 datasheet, log-log fit):
 *
 *  NH3  :  ppm = 102.2  * (Rs/R0)^(-2.473)   valid 10–300 ppm
 *  CO   :  ppm = 526.9  * (Rs/R0)^(-1.958)   valid 1–1000 ppm
 *  VOC  :  ppm = 14.0   * (Rs/R0)^(-1.515)   valid 10–500 ppm (ethanol)
 *
 *  These are estimates from curve-fitting the published datasheet graphs.
 *  For lab-grade accuracy: use certified calibration gas to build your own curve.
 */
float rsToPpm_NH3(float rs) {
  float ratio = rs / R0_NH3;
  if (ratio <= 0.0f) ratio = 0.001f;
  return constrain(102.2f * pow(ratio, -2.473f), 0.0f, 1000.0f);
}

float rsToPpm_CO(float rs) {
  float ratio = rs / R0_CO;
  if (ratio <= 0.0f) ratio = 0.001f;
  return constrain(526.9f * pow(ratio, -1.958f), 0.0f, 2000.0f);
}

float rsToPpm_VOC(float rs) {
  float ratio = rs / R0_VOC;
  if (ratio <= 0.0f) ratio = 0.001f;
  return constrain(14.0f * pow(ratio, -1.515f), 0.0f, 1000.0f);
}

// MP-4: linear interpolation from clean-air baseline to max ppm
float adcToEthanolPpm(int raw) {
  int   span = MP4_ADC_MAX - MP4_ADC_CLEAN;
  float ppm  = ((float)(raw - MP4_ADC_CLEAN) / (float)span) * MP4_PPM_MAX;
  return constrain(ppm, 0.0f, MP4_PPM_MAX);
}

// =============================================================
//  STATUS FUNCTIONS
// =============================================================

// GREEN solid — all clear
void setStatusSafe() {
  digitalWrite(PIN_LED_GREEN,  HIGH);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     LOW);
}

// YELLOW solid + 1 short beep — humidity warning
void setStatusYellow() {
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, HIGH);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     HIGH);
  delay(BEEP_ON_MS);
  digitalWrite(PIN_BUZZER, LOW);
}

// RED blinks N times IN SYNC with N buzzer beeps → RED stays solid after
void setStatusAlarm(int beeps) {
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  for (int i = 0; i < beeps; i++) {
    digitalWrite(PIN_LED_RED, HIGH);   // Red ON
    digitalWrite(PIN_BUZZER,  HIGH);   // Buzzer ON  — both together
    delay(BEEP_ON_MS);
    digitalWrite(PIN_BUZZER, LOW);     // Buzzer OFF
    if (i < beeps - 1) {
      digitalWrite(PIN_LED_RED, LOW);  // Red OFF between beeps
      delay(BEEP_OFF_MS);
    }
  }
  // After last beep: Red LED stays ON solid as a persistent visual alarm
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_BUZZER,  LOW);
}

// =============================================================
//  UTILITIES
// =============================================================

int averageADC(uint8_t pin, int n) {
  long s = 0;
  for (int i = 0; i < n; i++) { s += analogRead(pin); delay(2); }
  return (int)(s / n);
}

void blinkLED(uint8_t pin, int n, int ms) {
  for (int i = 0; i < n; i++) {
    digitalWrite(pin, HIGH); delay(ms);
    digitalWrite(pin, LOW);  delay(ms);
  }
}

void printGasRow(const char* lbl, int raw, float v, float rs,
                 float ppm, float wPpm, float aPpm) {
  const char* st = (ppm >= aPpm) ? "ALARM" :
                   (ppm >= wPpm) ? "WARN " : "OK   ";
  Serial.printf("%s| %4d | %5.3fV  | %7.2f | %8.1f | %s\n",
                lbl, raw, v, rs, ppm, st);
}

/*
 * ════════════════════════════════════════════════════════════
 *  HUMAN HEALTH REFERENCE  (built into threshold constants)
 * ════════════════════════════════════════════════════════════
 *
 *  NH3 (Ammonia)
 *  ┌───────────┬─────────────────────────────────────────────┐
 *  │   25 ppm  │ OSHA PEL  8-hr TWA  →  WARN threshold      │
 *  │   35 ppm  │ NIOSH STEL 15-min   →  ALARM threshold      │
 *  │   50 ppm  │ Immediate eye/throat irritation             │
 *  │  300 ppm  │ IDLH — Immediately Dangerous to Life/Health │
 *  └───────────┴─────────────────────────────────────────────┘
 *
 *  CO (Carbon Monoxide)
 *  ┌───────────┬─────────────────────────────────────────────┐
 *  │   35 ppm  │ OSHA PEL  8-hr TWA  →  WARN threshold      │
 *  │  200 ppm  │ NIOSH STEL          →  ALARM threshold      │
 *  │  800 ppm  │ Dizziness/nausea within 45 min              │
 *  │ 1200 ppm  │ IDLH                                        │
 *  └───────────┴─────────────────────────────────────────────┘
 *
 *  Ethanol / VOC vapour
 *  ┌───────────┬─────────────────────────────────────────────┐
 *  │  100 ppm  │ OSHA PEL  ethanol 8-hr TWA  →  WARN        │
 *  │  400 ppm  │ Eye/nose irritation onset   →  ALARM        │
 *  │ 1000 ppm  │ Strong irritation, headache                 │
 *  │ 3300 ppm  │ IDLH ethanol                                │
 *  └───────────┴─────────────────────────────────────────────┘
 *
 *  Humidity (sensor accuracy)
 *  ┌───────────┬─────────────────────────────────────────────┐
 *  │  < 10 %   │ Below MICS6814 operating range → YELLOW     │
 *  │  > 85 %   │ MICS6814 accuracy degrades     → YELLOW     │
 *  └───────────┴─────────────────────────────────────────────┘
 *
 * ════════════════════════════════════════════════════════════
 *  CALIBRATION PROCEDURE
 * ════════════════════════════════════════════════════════════
 *
 *  Step 1 — First power-on burn-in:
 *    Run sensor for 48 hours to stabilise resistance baseline.
 *
 *  Step 2 — R0 measurement (clean outdoor air, 30-min warm-up):
 *    Open Serial Monitor at 115200 baud.
 *    Note the Rs(kΩ) values printed for NH3, CO, VOC.
 *    Update R0_NH3, R0_CO, R0_VOC with those values.
 *    Re-upload — ppm should now read near 0 in clean air.
 *
 *  Step 3 — MP-4 baseline:
 *    In clean air, observe the rawEthanol ADC value.
 *    Set MP4_ADC_CLEAN to that value and re-upload.
 *
 *  Step 4 — Optional precision calibration:
 *    Use a certified calibration gas source at a known ppm.
 *    Adjust the A & B coefficients in rsToPpm_XXX() functions
 *    until the displayed ppm matches the known concentration.
 * ════════════════════════════════════════════════════════════
 */
