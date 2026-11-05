/*
===============================================================================
  BENCH TEST: GY-521 / MPU6050 on I2C (A4 = SDA, A5 = SCL)
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  BEFORE FITTING THE NANO: with 12 V on and the Nano out of its socket,
  measure SDA and SCL to GND. ~3.3 V = OK. ~5 V = STOP (the GY-521 pull-ups
  go to its 5 V VCC and would put 5 V on the ESP32). See BRINGUP_CHECKLIST.md.

  Steps run at boot:
    1. I2C bus scan (expects exactly one device at 0x68; AD0 is unconnected
       on this PCB, so the GY-521's AD0 pull-down selects 0x68)
    2. WHO_AM_I (register 0x75) must read 0x68
    3. Wake + configure (+/-4 g, +/-500 dps, 21 Hz filter) with direct
       register writes, then read back to confirm
  Then 'g' streams raw + filtered values at 10 Hz. No library needed.

  Needs 12 V on: the GY-521 VCC comes from the board's 5 V converter.
  Serial Monitor: 115200 baud. Type ? for help.
===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_I2C_SDA           = A4;  // GPIO11  net Net-(A1-A4{slash}SDA) -> U2.4 SDA
static const uint8_t PIN_I2C_SCL           = A5;  // GPIO12  net Net-(A1-A5{slash}SCL) -> U2.3 SCL
static const uint8_t IMU_I2C_ADDR          = 0x68;
static const uint8_t PIN_UPDOWN_EN         = D5;
static const uint8_t PIN_LEFTRIGHT_EN      = D2;
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12;
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11;
static const uint8_t PIN_LED_COCKPIT_FRONT = D10;
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;
static const uint8_t PIN_PWM_DATA          = D8;

static const float ACCEL_LSB_PER_G  = 8192.0f;   // +/-4 g
static const float GYRO_LSB_PER_DPS = 65.5f;     // +/-500 dps
static const float ALPHA            = 0.98f;     // same complementary filter as kayak_working.ino
static const float RAD2DEG          = 57.2957795f;

static bool     g_ok = false;
static bool     g_stream = false;
static uint32_t g_lastSampleMs = 0, g_lastPrintMs = 0;
static float    g_biasX = 0, g_biasY = 0, g_biasZ = 0;   // dps
static float    g_roll = 0, g_pitch = 0, g_yaw = 0;
static bool     g_seeded = false;
static uint32_t g_errors = 0, g_samples = 0;

static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
}

static bool WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool ReadRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_I2C_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// raw sample in physical units: accel g, gyro dps, temp C
static bool ReadSample(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &t) {
  uint8_t b[14];
  if (!ReadRegs(0x3B, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]) / ACCEL_LSB_PER_G;
  ay = (int16_t)((b[2] << 8) | b[3]) / ACCEL_LSB_PER_G;
  az = (int16_t)((b[4] << 8) | b[5]) / ACCEL_LSB_PER_G;
  t  = (int16_t)((b[6] << 8) | b[7]) / 340.0f + 36.53f;
  gx = (int16_t)((b[8] << 8) | b[9]) / GYRO_LSB_PER_DPS;
  gy = (int16_t)((b[10] << 8) | b[11]) / GYRO_LSB_PER_DPS;
  gz = (int16_t)((b[12] << 8) | b[13]) / GYRO_LSB_PER_DPS;
  return true;
}

static void Scan() {
  Serial.println("[1] I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    device at 0x%02X%s\n", a, a == 0x68 ? "  (MPU6050, AD0 low)" : a == 0x69 ? "  (MPU6050 with AD0 HIGH -- unexpected)" : "");
      found++;
    }
  }
  if (!found) Serial.println("    NOTHING FOUND: GY-521 unpowered (12 V on?), SDA/SCL swapped, or no pull-ups");
}

static void Calibrate() {
  Serial.println("[c] gyro bias: keep the board STILL for 1 s...");
  float sx = 0, sy = 0, sz = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < 100; i++) {
    float ax, ay, az, gx, gy, gz, t;
    if (ReadSample(ax, ay, az, gx, gy, gz, t)) { sx += gx; sy += gy; sz += gz; n++; }
    delay(10);
  }
  if (n == 0) { Serial.println("    calibration FAILED (no samples)"); return; }
  g_biasX = sx / n; g_biasY = sy / n; g_biasZ = sz / n;
  g_yaw = 0; g_seeded = false;
  Serial.printf("    bias %.3f %.3f %.3f dps (%u samples). |bias| < ~5 dps is normal.\n",
                (double)g_biasX, (double)g_biasY, (double)g_biasZ, n);
}

static void Init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Scan();

  uint8_t who = 0;
  const bool whoOk = ReadRegs(0x75, &who, 1);
  Serial.printf("[2] WHO_AM_I = %s0x%02X  => %s\n", whoOk ? "" : "(read failed) ", who,
                (whoOk && who == 0x68) ? "PASS" : "FAIL (MPU6050 returns 0x68; clones may return 0x70/0x72/0x98)");

  bool cfgOk = WriteReg(0x6B, 0x01);   // PWR_MGMT_1: wake, clock = gyro X PLL
  delay(50);
  cfgOk &= WriteReg(0x1A, 0x04);       // CONFIG: DLPF ~21 Hz
  cfgOk &= WriteReg(0x1B, 0x08);       // GYRO_CONFIG: +/-500 dps
  cfgOk &= WriteReg(0x1C, 0x08);       // ACCEL_CONFIG: +/-4 g
  uint8_t rb[3] = { 0, 0, 0 };
  cfgOk &= ReadRegs(0x1A, rb, 3);
  cfgOk &= (rb[0] & 0x07) == 0x04 && (rb[1] & 0x18) == 0x08 && (rb[2] & 0x18) == 0x08;
  Serial.printf("[3] configure + read-back: CONFIG 0x%02X GYRO 0x%02X ACCEL 0x%02X  => %s\n",
                rb[0], rb[1], rb[2], cfgOk ? "PASS" : "FAIL");
  g_ok = whoOk && cfgOk;
  if (g_ok) Calibrate();
}

static void Help() {
  Serial.println("--- test_imu ---");
  Serial.println("g   start/stop 10 Hz stream (raw + filtered)");
  Serial.println("c   re-calibrate gyro bias (keep still)");
  Serial.println("z   zero yaw");
  Serial.println("a   re-run scan + WHO_AM_I + config");
  Serial.println("?   this help");
  Serial.println("Checks: flat & still -> accel ~ 0, 0, +1.00 g; roll/pitch ~0; yaw drifts slowly.");
  Serial.println("        Rotate 90 deg about vertical -> yaw changes ~90.");
}

void setup() {
  SafeOutputs();
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }
  Serial.println("\n=== test_imu ===");
  Init();
  Help();
}

void loop() {
  const uint32_t now = millis();
  if (g_ok && (now - g_lastSampleMs) >= 10) {          // 100 Hz filter update
    const float dt = (g_lastSampleMs == 0) ? 0.01f : (now - g_lastSampleMs) / 1000.0f;
    g_lastSampleMs = now;
    float ax, ay, az, gx, gy, gz, t;
    if (!ReadSample(ax, ay, az, gx, gy, gz, t)) {
      g_errors++;
    } else {
      g_samples++;
      gx -= g_biasX; gy -= g_biasY; gz -= g_biasZ;
      const float rollAcc  = atan2f(ay, az) * RAD2DEG;
      const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
      if (!g_seeded) { g_roll = rollAcc; g_pitch = pitchAcc; g_seeded = true; }
      g_roll  = ALPHA * (g_roll  + gx * dt) + (1 - ALPHA) * rollAcc;
      g_pitch = ALPHA * (g_pitch + gy * dt) + (1 - ALPHA) * pitchAcc;
      float cp = cosf(g_pitch / RAD2DEG); if (cp < 0.2f) cp = 0.2f;
      const float rr = g_roll / RAD2DEG;
      g_yaw += (gy * sinf(rr) + gz * cosf(rr)) / cp * dt;

      if (g_stream && (now - g_lastPrintMs) >= 100) {
        g_lastPrintMs = now;
        const float aMag = sqrtf(ax * ax + ay * ay + az * az);
        Serial.printf("acc %+.2f %+.2f %+.2f g |%.2f|  gyro %+7.2f %+7.2f %+7.2f dps  "
                      "roll %+6.1f pitch %+6.1f yaw %+7.1f  T %.1f C  err %lu\n",
                      (double)ax, (double)ay, (double)az, (double)aMag, (double)gx, (double)gy, (double)gz,
                      (double)g_roll, (double)g_pitch, (double)g_yaw, (double)t, (unsigned long)g_errors);
      }
    }
  }

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'g') { g_stream = !g_stream; Serial.printf("stream %s\n", g_stream ? "ON" : "off"); }
    else if (c == 'c') Calibrate();
    else if (c == 'z') { g_yaw = 0; Serial.println("yaw zeroed"); }
    else if (c == 'a') Init();
    else if (c == '?') Help();
  }
}
