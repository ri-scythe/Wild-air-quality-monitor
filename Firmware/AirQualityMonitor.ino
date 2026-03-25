/*
 ============================================================
  DIY Air Quality Monitor
  Board  : Arduino Pro Mini 5V / 16MHz
  Display: Nextion 2.8" (NX3224T028_011  –  320×240 px)

  Sensors & wiring
  ─────────────────────────────────────────────────────────
  MH-Z19B  (CO2)   RX→D3  TX→D2   5 V / GND
  PMS5003  (PM2.5) RX→D5  TX→D4   5 V / GND  *see note*
  DHT22    (Temp)  DATA→D6         3.3–5 V / GND
  DS3231   (RTC)   SDA→A4 SCL→A5  3.3–5 V / GND
  Nextion  (LCD)   TX→D0  RX→D1   5 V / GND

  * PMS5003 UART is 3.3 V logic. Put a 1k resistor between
    Arduino D5 (TX) and PMS RX to avoid damaging the sensor.

  Libraries needed (install via Library Manager)
  ─────────────────────────────────────────────────────────
  DHT sensor library  – Adafruit
  RTClib              – Adafruit
  SoftwareSerial      – built-in

  Nextion picture IDs (add images in Nextion Editor)
  ─────────────────────────────────────────────────────────
  ID 0 → bunny_happy.bmp      (normal air)
  ID 1 → bunny_neutral.bmp    (slightly degraded)
  ID 2 → bunny_worried.bmp    (bad air)
  ID 3 → bunny_danger.bmp     (dangerous air)
 ============================================================
*/

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

// ── Pin definitions ──────────────────────────────────────────
#define DHT_PIN      6
#define DHT_TYPE     DHT22
#define CO2_RX       2    // Arduino RX ← MH-Z19B TX
#define CO2_TX       3    // Arduino TX → MH-Z19B RX
#define PMS_RX       4    // Arduino RX ← PMS5003 TX
#define PMS_TX       5    // Arduino TX → PMS5003 RX  (1k resistor!)
// Nextion on hardware Serial pins 0/1
// DS3231 on I2C A4/A5

// ── Objects ─────────────────────────────────────────────────
SoftwareSerial co2Serial(CO2_RX, CO2_TX);
SoftwareSerial pmsSerial(PMS_RX, PMS_TX);
DHT            dht(DHT_PIN, DHT_TYPE);
RTC_DS3231     rtc;

// ── Bunny state picture IDs ──────────────────────────────────
#define BUNNY_HAPPY    0
#define BUNNY_NEUTRAL  1
#define BUNNY_WORRIED  2
#define BUNNY_DANGER   3

// ── Air-quality thresholds ───────────────────────────────────
// CO2 ppm │ PM2.5 µg/m³ │ state
//  < 800  │  < 12       │ HAPPY
//  < 1200 │  < 35       │ NEUTRAL
//  < 1800 │  < 55       │ WORRIED
//  ≥ 1800 │  ≥ 55       │ DANGER

// ── Sensor data ─────────────────────────────────────────────
int   g_co2_ppm   = 0;
int   g_pm25_ugm3 = 0;
float g_temp_c    = 0.0;
uint8_t g_lastBunnyId = 255;   // forces first refresh

// ── Timing ──────────────────────────────────────────────────
unsigned long tCO2  = 0;
unsigned long tPMS  = 0;
unsigned long tDHT  = 0;
unsigned long tTime = 0;

const unsigned long INTERVAL_CO2  = 5000UL;
const unsigned long INTERVAL_PMS  = 3000UL;
const unsigned long INTERVAL_DHT  = 6000UL;
const unsigned long INTERVAL_TIME = 10000UL;

// ════════════════════════════════════════════════════════════
//  NEXTION HELPERS
// ════════════════════════════════════════════════════════════

// Send a command followed by the three 0xFF terminator bytes
void nxSend(const String &cmd) {
  Serial.print(cmd);
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)0xFF);
}

void nxSetText(const char *comp, const String &val) {
  nxSend(String(comp) + ".txt=\"" + val + "\"");
}

void nxSetPic(uint8_t picId) {
  nxSend("p0.pic=" + String(picId));
}

// ════════════════════════════════════════════════════════════
//  MH-Z19B  (CO2)
// ════════════════════════════════════════════════════════════
static const byte CO2_CMD[9] = {
  0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79
};

// Returns CO2 ppm, or -1 on error
int readCO2() {
  co2Serial.listen();
  while (co2Serial.available()) co2Serial.read();   // flush old bytes

  for (int i = 0; i < 9; i++) co2Serial.write(CO2_CMD[i]);

  // Wait up to 200 ms for response
  unsigned long t = millis();
  while (co2Serial.available() < 9 && millis() - t < 200);
  if (co2Serial.available() < 9) return -1;

  byte resp[9];
  for (int i = 0; i < 9; i++) resp[i] = co2Serial.read();

  if (resp[0] != 0xFF || resp[1] != 0x86) return -1;

  // Verify checksum
  byte chk = 0;
  for (int i = 1; i < 8; i++) chk += resp[i];
  chk = 0xFF - chk + 1;
  if (chk != resp[8]) return -1;

  return (resp[2] << 8) | resp[3];
}

// ════════════════════════════════════════════════════════════
//  PMS5003  (PM2.5)
// ════════════════════════════════════════════════════════════
struct PMSData {
  uint16_t pm10_std, pm25_std, pm100_std;   // factory
  uint16_t pm10_env, pm25_env, pm100_env;   // atmospheric
  uint16_t p03, p05, p10, p25, p50, p100;  // particle counts
};

// Returns true on valid frame; fills d.pm25_env with µg/m³
bool readPMS(PMSData &d) {
  pmsSerial.listen();

  unsigned long t = millis();
  while (millis() - t < 1500) {
    if (!pmsSerial.available()) continue;

    if (pmsSerial.read() != 0x42) continue;

    // Wait for second start byte
    unsigned long t2 = millis();
    while (!pmsSerial.available() && millis() - t2 < 100);
    if (!pmsSerial.available()) continue;
    if (pmsSerial.read() != 0x4D) continue;

    // Read 30 remaining bytes (frame length 28 + 2 checksum)
    byte buf[30];
    uint8_t idx = 0;
    unsigned long t3 = millis();
    while (idx < 30 && millis() - t3 < 500) {
      if (pmsSerial.available()) buf[idx++] = pmsSerial.read();
    }
    if (idx < 30) continue;

    // Checksum covers start bytes + first 28 data bytes
    uint16_t sum = 0x42 + 0x4D;
    for (int i = 0; i < 28; i++) sum += buf[i];
    uint16_t check = ((uint16_t)buf[28] << 8) | buf[29];
    if (sum != check) continue;

    d.pm10_std  = ((uint16_t)buf[2]  << 8) | buf[3];
    d.pm25_std  = ((uint16_t)buf[4]  << 8) | buf[5];
    d.pm100_std = ((uint16_t)buf[6]  << 8) | buf[7];
    d.pm10_env  = ((uint16_t)buf[8]  << 8) | buf[9];
    d.pm25_env  = ((uint16_t)buf[10] << 8) | buf[11];
    d.pm100_env = ((uint16_t)buf[12] << 8) | buf[13];
    return true;
  }
  return false;
}

// ════════════════════════════════════════════════════════════
//  BUNNY LOGIC
// ════════════════════════════════════════════════════════════
uint8_t calcBunnyState() {
  if (g_co2_ppm >= 1800 || g_pm25_ugm3 >= 55) return BUNNY_DANGER;
  if (g_co2_ppm >= 1200 || g_pm25_ugm3 >= 35) return BUNNY_WORRIED;
  if (g_co2_ppm >=  800 || g_pm25_ugm3 >= 12) return BUNNY_NEUTRAL;
  return BUNNY_HAPPY;
}

void refreshBunny() {
  uint8_t state = calcBunnyState();
  if (state != g_lastBunnyId) {
    nxSetPic(state);
    g_lastBunnyId = state;
  }
}

// ════════════════════════════════════════════════════════════
//  DISPLAY UPDATE HELPERS
// ════════════════════════════════════════════════════════════
void displayCO2() {
  if (g_co2_ppm > 0)
    nxSetText("t_co2", String(g_co2_ppm) + " ppm");
  else
    nxSetText("t_co2", "--- ppm");
  refreshBunny();
}

void displayPM() {
  if (g_pm25_ugm3 >= 0)
    nxSetText("t_pm", String(g_pm25_ugm3) + " ug/m3");
  else
    nxSetText("t_pm", "--- ug/m3");
  refreshBunny();
}

void displayTemp() {
  if (!isnan(g_temp_c)) {
    char buf[8];
    dtostrf(g_temp_c, 4, 1, buf);
    nxSetText("t_temp", String(buf) + " C");
  } else {
    nxSetText("t_temp", "--.- C");
  }
}

void displayTime() {
  DateTime now = rtc.now();
  char buf[6];
  sprintf(buf, "%02u:%02u", now.hour(), now.minute());
  nxSetText("t_time", buf);
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);       // Nextion (hardware serial, pins 0/1)
  co2Serial.begin(9600);    // MH-Z19B
  pmsSerial.begin(9600);    // PMS5003
  dht.begin();
  Wire.begin();

  if (!rtc.begin()) {
    // RTC not found – time will show 00:00
    // You can also set time here once:
    //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Nextion init
  delay(600);
  nxSend("bkcmd=0");   // suppress ACK bytes from Nextion
  nxSend("page 0");
  delay(150);

  // Show dashes while sensors warm up
  nxSetText("t_co2", "--- ppm");
  nxSetText("t_pm",  "--- ug/m3");
  nxSetText("t_temp","--.- C");
  nxSetPic(BUNNY_NEUTRAL);
  g_lastBunnyId = BUNNY_NEUTRAL;
  displayTime();

  // MH-Z19B needs ~60 s to warm up; show neutral bunny
  // (readings will auto-correct once valid data arrives)
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── CO2 ────────────────────────────────────────────────────
  if (now - tCO2 >= INTERVAL_CO2) {
    tCO2 = now;
    int val = readCO2();
    if (val > 0 && val < 10000) {
      g_co2_ppm = val;
      displayCO2();
    }
  }

  // ── PM2.5 ──────────────────────────────────────────────────
  if (now - tPMS >= INTERVAL_PMS) {
    tPMS = now;
    PMSData pms;
    if (readPMS(pms)) {
      g_pm25_ugm3 = (int)pms.pm25_env;
      displayPM();
    }
  }

  // ── Temperature (DHT22) ────────────────────────────────────
  if (now - tDHT >= INTERVAL_DHT) {
    tDHT = now;
    float t = dht.readTemperature();
    if (!isnan(t)) {
      g_temp_c = t;
      displayTemp();
    }
  }

  // ── Clock ──────────────────────────────────────────────────
  if (now - tTime >= INTERVAL_TIME) {
    tTime = now;
    displayTime();
  }
}
