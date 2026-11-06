/*
===============================================================================
  BENCH TEST: ONE stepper motor (lift OR pan), via its TMC2209
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  Pick the axis below (TEST_AXIS). The OTHER driver is never enabled.

  Safety (same rules as kayak_working.ino):
    - Run current 800 mA RMS (<= 1 A motor rating), IHOLD 30 %, automatic
      standstill reduction. VREF trimpot ignored (I_scale_analog = 0).
    - The driver is enabled ONLY after its UART config is verified (IFCNT
      write count + register read-back). If not: RED LED, nothing moves.
    - Health check every 250 ms: if the driver stops answering or loses its
      config (12 V dropout / reset) it is disabled immediately.
    - Moves are limited to +/- TEST_TRAVEL_STEPS around the power-on
      position. Start with the motor OFF the mechanism, or hand-position the
      axis mid-travel first. There are no end-stops.

  Prerequisite: test_tmc_uart passes (R10 fitted with 1 kOhm).
  Never plug/unplug the motor with 12 V on.
  Serial Monitor: 115200 baud, line ending "Newline". Type ? for help.
===============================================================================
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ======================= CHOOSE THE AXIS =======================
// 0 = UP/DOWN lift  (TMC2209 A2, motor on J3, UART address 0)
// 1 = LEFT/RIGHT pan (TMC2209 A3, motor on J4, UART address 1)
#ifndef TEST_AXIS
#define TEST_AXIS 0
#endif
// ===============================================================

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_UPDOWN_EN         = D5;  // GPIO8   net Net-(A2-~{EN})
static const uint8_t PIN_UPDOWN_STEP       = D6;  // GPIO9   net Net-(A2-STEP)
static const uint8_t PIN_UPDOWN_DIR        = D7;  // GPIO10  net Net-(A2-DIR)
static const uint8_t PIN_UPDOWN_DIAG       = A6;  // GPIO13  net Net-(A1-PadA6)
static const uint8_t PIN_LEFTRIGHT_EN      = D2;  // GPIO5   net Net-(A3-~{EN})
static const uint8_t PIN_LEFTRIGHT_STEP    = D3;  // GPIO6   net Net-(A3-STEP)
static const uint8_t PIN_LEFTRIGHT_DIR     = D4;  // GPIO7   net Net-(A3-DIR)
static const uint8_t PIN_LEFTRIGHT_DIAG    = A7;  // GPIO14  net Net-(A1-PadA7)
static const uint8_t PIN_TMC_UART_RX       = D0;  // GPIO44  net Net-(A1-D0{slash}RX)
static const uint8_t PIN_TMC_UART_TX       = D1;  // GPIO43  net Net-(A1-D1{slash}TX)
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12; // GPIO47
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11; // GPIO38
static const uint8_t PIN_LED_COCKPIT_FRONT = D10; // GPIO21
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;  // GPIO18
static const uint8_t PIN_PWM_DATA          = D8;  // GPIO17
static const uint8_t PIN_STATUS_LED_R      = LED_RED;
static const uint8_t PIN_STATUS_LED_G      = LED_GREEN;
static const uint8_t PIN_STATUS_LED_B      = LED_BLUE;

static const uint8_t TMC_ADDR_UPDOWN       = 0;
static const uint8_t TMC_ADDR_LEFTRIGHT    = 1;

#if TEST_AXIS == 0
static const char   *AXIS_NAME = "UP/DOWN lift (A2, J3, addr 0)";
static const uint8_t AX_EN = PIN_UPDOWN_EN, AX_STEP = PIN_UPDOWN_STEP, AX_DIR = PIN_UPDOWN_DIR,
                     AX_DIAG = PIN_UPDOWN_DIAG, AX_ADDR = TMC_ADDR_UPDOWN;
#elif TEST_AXIS == 1
static const char   *AXIS_NAME = "LEFT/RIGHT pan (A3, J4, addr 1)";
static const uint8_t AX_EN = PIN_LEFTRIGHT_EN, AX_STEP = PIN_LEFTRIGHT_STEP, AX_DIR = PIN_LEFTRIGHT_DIR,
                     AX_DIAG = PIN_LEFTRIGHT_DIAG, AX_ADDR = TMC_ADDR_LEFTRIGHT;
#else
#error "TEST_AXIS must be 0 (lift) or 1 (pan)"
#endif

// ---------------- MOTOR / DRIVER SETTINGS (same as kayak_working.ino) ----------------
static constexpr uint16_t TMC_RUN_CURRENT_MA  = 800;
static constexpr float    TMC_HOLD_MULTIPLIER = 0.30f;
static_assert(TMC_RUN_CURRENT_MA <= 800, "TMC run current must stay <= 800 mA (1 A motors)");
static const float    TMC_R_SENSE         = 0.11f;
static const uint32_t TMC_UART_BAUD       = 115200;
static const uint16_t TMC_MICROSTEPS      = 16;
static const uint8_t  TMC_TOFF            = 3;
static const uint8_t  TMC_IHOLDDELAY      = 6;
static const uint8_t  TMC_TPOWERDOWN      = 20;
static const uint16_t TMC_HEALTH_PERIOD_MS = 250;

// ---------------- TEST MOTION LIMITS ----------------
// 3200 microsteps = 1 motor revolution (200 full steps x 16).
static const long  TEST_TRAVEL_STEPS   = 800;     // +/- 1/4 motor rev around the start point
static const float TEST_MAX_SPEED      = 2000.0f; // steps/s ceiling for 'v'
static const float TEST_ACCEL          = 4000.0f;

TMC2209Stepper driver(&Serial1, TMC_R_SENSE, AX_ADDR);
AccelStepper   stepper(AccelStepper::DRIVER, AX_STEP, AX_DIR);

static bool     g_configOk = false;
static bool     g_enabled  = false;
static long     g_stepSize = 200;
static float    g_speed    = 600.0f;
static bool     g_wasRunning = false;
static uint32_t g_lastHealthMs = 0;
static uint8_t  g_outBackPhase = 0;   // 'o' out-and-back: 0 idle, 1 out, 2 back
static long     g_outBackOrigin = 0;

// ---------------- SAFE OUTPUTS ----------------
static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }   // both drivers disabled
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  digitalWrite(PIN_STATUS_LED_R, HIGH); digitalWrite(PIN_STATUS_LED_G, HIGH); digitalWrite(PIN_STATUS_LED_B, HIGH);
  pinMode(PIN_STATUS_LED_R, OUTPUT); pinMode(PIN_STATUS_LED_G, OUTPUT); pinMode(PIN_STATUS_LED_B, OUTPUT);
}

static void StatusLed(bool r, bool g, bool b) {
  digitalWrite(PIN_STATUS_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_B, b ? LOW : HIGH);
}

static void SetEnabled(bool en) {
  if (en && !g_configOk) { Serial.println("refused: driver config not verified"); return; }
  if (!en) {
    stepper.setCurrentPosition(stepper.currentPosition());   // stop the profile before EN HIGH
    g_outBackPhase = 0;
  }
  digitalWrite(AX_EN, en ? LOW : HIGH);
  g_enabled = en;
}

// ---------------- UART CONFIG (same procedure as kayak_working.ino) ----------------
static bool ReadIfcnt(uint8_t &value, uint8_t &extraWrites) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    value = driver.IFCNT();
    if (!driver.CRCerror) return true;
    driver.TPOWERDOWN(TMC_TPOWERDOWN);
    extraWrites++;
  }
  return false;
}

static bool ExpectedVsense(uint16_t mA, float rSense) {
  const float cs = 32.0f * 1.41421f * (float)mA / 1000.0f * (rSense + 0.02f) / 0.325f - 1.0f;
  return (uint8_t)cs < 16;
}

static bool ConfigureDriver() {
  g_configOk = false;
  const uint8_t conn = driver.test_connection();
  if (conn != 0) {
    Serial.printf("test_connection = %u (%s) -- run test_tmc_uart\n", conn,
                  conn == 2 ? "NO REPLY" : "read 0xFFFFFFFF");
    return false;
  }
  const uint32_t gconf = driver.GCONF(), chop = driver.CHOPCONF(), pwm = driver.PWMCONF();
  if (chop == 0 || pwm == 0) { Serial.println("register read failed"); return false; }

  uint8_t before = 0, extraB = 0;
  if (!ReadIfcnt(before, extraB)) { Serial.println("IFCNT read failed"); return false; }
  uint8_t writes = 0;
  driver.GCONF(gconf);                 writes++;
  driver.CHOPCONF(chop);               writes++;
  driver.PWMCONF(pwm);                 writes++;
  driver.pdn_disable(true);            writes++;
  driver.mstep_reg_select(true);       writes++;
  driver.I_scale_analog(false);        writes++;
  driver.internal_Rsense(false);       writes++;
  driver.multistep_filt(true);         writes++;
  driver.en_spreadCycle(false);        writes++;
  driver.toff(TMC_TOFF);               writes++;
  driver.intpol(true);                 writes++;
  driver.microsteps(TMC_MICROSTEPS);   writes++;
  driver.pwm_autoscale(true);          writes++;
  driver.pwm_autograd(true);           writes++;
  driver.rms_current(TMC_RUN_CURRENT_MA, TMC_HOLD_MULTIPLIER); writes += 3;
  driver.iholddelay(TMC_IHOLDDELAY);   writes++;
  driver.TPOWERDOWN(TMC_TPOWERDOWN);   writes++;
  driver.TCOOLTHRS(0);                 writes++;
  driver.SGTHRS(0);                    writes++;
  uint8_t after = 0, extraA = 0;
  const bool afterOk = ReadIfcnt(after, extraA);
  writes += extraA;
  const uint8_t delta = (uint8_t)(after - before);

  const bool gOk = driver.pdn_disable() && driver.mstep_reg_select() &&
                   !driver.I_scale_analog() && !driver.en_spreadCycle();
  const bool cOk = (driver.microsteps() == TMC_MICROSTEPS) && (driver.toff() == TMC_TOFF) &&
                   (driver.vsense() == ExpectedVsense(TMC_RUN_CURRENT_MA, TMC_R_SENSE)) && !driver.CRCerror;
  const bool pOk = driver.pwm_autoscale();
  g_configOk = afterOk && (delta == writes) && gOk && cOk && pOk;
  Serial.printf("config: writes %u/%u acked, GCONF %s, CHOPCONF %s, PWMCONF %s -> %s (IRUN %u mA RMS)\n",
                delta, writes, gOk ? "ok" : "BAD", cOk ? "ok" : "BAD", pOk ? "ok" : "BAD",
                g_configOk ? "VERIFIED" : "FAILED", driver.rms_current());
  return g_configOk;
}

static void BringUp() {
  SetEnabled(false);
  StatusLed(false, false, true);
  const bool ok = ConfigureDriver();
  if (ok) { SetEnabled(true); StatusLed(false, true, false); Serial.println("driver ENABLED"); }
  else    { StatusLed(true, false, false); Serial.println("driver left DISABLED"); }
}

static void CheckHealth(uint32_t nowMs) {
  if (!g_enabled || (nowMs - g_lastHealthMs) < TMC_HEALTH_PERIOD_MS) return;
  g_lastHealthMs = nowMs;
  const uint32_t g = driver.GCONF();
  const bool ok = !driver.CRCerror && (g & (1UL << 6)) && (g & (1UL << 7)) &&
                  !(g & (1UL << 0)) && !(g & (1UL << 2));
  if (!ok) {
    SetEnabled(false);
    g_configOk = false;
    StatusLed(true, false, false);
    Serial.println("FAULT: driver stopped answering or lost config (12 V dropout/reset) -> DISABLED. 'r' to retry.");
  }
}

// ---------------- MOTION (always clamped to +/- TEST_TRAVEL_STEPS) ----------------
static void MoveTo(long target) {
  if (!g_enabled) { Serial.println("driver disabled -- 'e' or 'r'"); return; }
  const long t = constrain(target, -TEST_TRAVEL_STEPS, TEST_TRAVEL_STEPS);
  if (t != target) Serial.printf("clamped to %ld (test window +/-%ld)\n", t, TEST_TRAVEL_STEPS);
  stepper.moveTo(t);
}

static void Dump() {
  const uint8_t conn = driver.test_connection();
  Serial.printf("== %s ==\n", AXIS_NAME);
  Serial.printf("  test_connection %u  config %s  ~EN %s  DIAG %s\n", conn,
                g_configOk ? "VERIFIED" : "NOT VERIFIED", digitalRead(AX_EN) ? "HIGH (off)" : "LOW (on)",
                digitalRead(AX_DIAG) ? "HIGH" : "LOW");
  Serial.printf("  position %ld (window +/-%ld)  target %ld  speed %.0f  step %ld\n",
                stepper.currentPosition(), TEST_TRAVEL_STEPS, stepper.targetPosition(),
                (double)g_speed, g_stepSize);
  if (conn != 0) return;
  Serial.printf("  DRV_STATUS 0x%08lX  stst=%u stealth=%u cs_actual=%u (run ~25, hold ~7)\n",
                (unsigned long)driver.DRV_STATUS(), driver.stst(), driver.stealth(), driver.cs_actual());
  Serial.printf("  ot=%u otpw=%u s2ga=%u s2gb=%u ola=%u olb=%u  TSTEP=%lu SG_RESULT=%u\n",
                driver.ot(), driver.otpw(), driver.s2ga(), driver.s2gb(), driver.ola(), driver.olb(),
                (unsigned long)driver.TSTEP(), driver.SG_RESULT());
}

static void Help() {
  Serial.printf("--- test_motor: %s ---\n", AXIS_NAME);
  Serial.println("+ / -      move one step size forward / back");
  Serial.println("o          out-and-back: +step then return");
  Serial.println("g <pos>    go to absolute position (clamped to the test window)");
  Serial.println("h          go home (position 0 = power-on position)");
  Serial.println("x          stop (decelerate)");
  Serial.println("n <steps>  set step size");
  Serial.println("v <sps>    set speed, steps/s (<= 2000)");
  Serial.println("d          dump driver status (cs_actual shows run/hold current)");
  Serial.println("e          toggle driver enable (enable only if verified)");
  Serial.println("r          re-run UART config");
  Serial.println("?          this help");
}

static char    g_line[24];
static uint8_t g_len = 0;

static void Execute(const char *line) {
  while (*line == ' ') line++;
  const char cmd = *line;
  const char *arg = line + 1;
  const long val = strtol(arg, nullptr, 10);
  switch (cmd) {
    case '+': MoveTo(stepper.targetPosition() + g_stepSize); break;
    case '-': MoveTo(stepper.targetPosition() - g_stepSize); break;
    case 'o': g_outBackOrigin = stepper.currentPosition(); g_outBackPhase = 1;
              MoveTo(g_outBackOrigin + g_stepSize); break;
    case 'g': MoveTo(val); break;
    case 'h': MoveTo(0); break;
    case 'x': stepper.stop(); g_outBackPhase = 0; break;
    case 'n': if (val > 0 && val <= TEST_TRAVEL_STEPS) { g_stepSize = val; Serial.printf("step %ld\n", g_stepSize); }
              else Serial.printf("n: 1..%ld\n", TEST_TRAVEL_STEPS); break;
    case 'v': if (val > 0 && val <= (long)TEST_MAX_SPEED) { g_speed = (float)val; stepper.setMaxSpeed(g_speed);
                Serial.printf("speed %.0f steps/s\n", (double)g_speed); }
              else Serial.println("v: 1..2000"); break;
    case 'd': Dump(); break;
    case 'e': SetEnabled(!g_enabled); Serial.printf("driver %s\n", g_enabled ? "ENABLED" : "disabled (no holding torque)"); break;
    case 'r': BringUp(); break;
    case '?': Help(); break;
    case '\0': break;
    default: Serial.printf("unknown '%c' -- ? for help\n", cmd); break;
  }
}

void setup() {
  SafeOutputs();
  pinMode(PIN_UPDOWN_DIAG, INPUT_PULLDOWN);
  pinMode(PIN_LEFTRIGHT_DIAG, INPUT_PULLDOWN);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }

  Serial.printf("\n=== test_motor: %s ===\n", AXIS_NAME);
  Serial.println("Position 0 = where the motor is now. Moves are limited to +/-800 steps (1/4 rev).");
  stepper.setMaxSpeed(g_speed);
  stepper.setAcceleration(TEST_ACCEL);
  stepper.setCurrentPosition(0);

  Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_UART_RX, PIN_TMC_UART_TX);
  driver.begin();
  BringUp();
  Help();
}

void loop() {
  // No LED strips in this test, so AccelStepper can be polled from loop().
  if (g_enabled) stepper.run();

  const bool running = stepper.isRunning();
  if (g_wasRunning && !running) {
    Serial.printf("stopped at %ld\n", stepper.currentPosition());
    if (g_outBackPhase == 1) { g_outBackPhase = 2; MoveTo(g_outBackOrigin); }
    else if (g_outBackPhase == 2) { g_outBackPhase = 0; Serial.println("out-and-back done"); }
  }
  g_wasRunning = running;

  // Health check also runs while moving: a reset driver must be caught at
  // once. Costs a ~5 ms pause in stepping every 250 ms (audible tick, no
  // lost position). The main firmware avoids this by stepping on core 0.
  CheckHealth(millis());

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      g_line[g_len] = '\0';
      if (g_len) Execute(g_line);
      g_len = 0;
    } else if (c == '+' || c == '-' || c == 'x') {
      if (g_len == 0) { char one[2] = { (char)c, 0 }; Execute(one); }   // act immediately, no Enter needed
      else if (g_len < sizeof(g_line) - 1) g_line[g_len++] = (char)c;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = (char)c;
    }
  }
}
