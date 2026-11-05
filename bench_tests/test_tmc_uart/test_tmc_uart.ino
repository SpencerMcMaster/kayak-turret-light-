/*
===============================================================================
  BENCH TEST: TMC2209 UART bus probe  (NO MOTION -- drivers are never enabled)
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  Run this FIRST, before test_motor. It answers:
    1. Does D1/TX actually reach the drivers' PDN_UART line? (the R10 issue:
       BTT TMC2209 V1.3 modules ship with R10 not fitted, so the "TX" header
       pin is connected to nothing)
    2. Does each driver answer at its address (lift A2 = 0, pan A3 = 1)?
    3. Do writes land (IFCNT increments)?

  Setup: 12 V ON (the TMC2209 logic is powered from VS), both modules
  fitted, motors may be connected or not. ~EN stays HIGH the whole time.

  Serial Monitor: 115200 baud, line ending "Newline". Type ? for help.
===============================================================================
*/

#include <Arduino.h>
#include <TMCStepper.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_UPDOWN_EN         = D5;  // GPIO8   net Net-(A2-~{EN})
static const uint8_t PIN_LEFTRIGHT_EN      = D2;  // GPIO5   net Net-(A3-~{EN})
static const uint8_t PIN_TMC_UART_RX       = D0;  // GPIO44  net Net-(A1-D0{slash}RX) -> A2.5, A3.5 UART_RX (= PDN_UART)
static const uint8_t PIN_TMC_UART_TX       = D1;  // GPIO43  net Net-(A1-D1{slash}TX) -> A2.4, A3.4 UART_TX (-> R10)
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12; // GPIO47  net Net-(A1-D12_MISO)
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11; // GPIO38  net Net-(A1-D11_MOSI)
static const uint8_t PIN_LED_COCKPIT_FRONT = D10; // GPIO21  net Net-(U1-A2)
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;  // GPIO18  net Net-(U1-A3)
static const uint8_t PIN_PWM_DATA          = D8;  // GPIO17  net Net-(J2-Pin_5)
static const uint8_t PIN_STATUS_LED_R      = LED_RED;
static const uint8_t PIN_STATUS_LED_G      = LED_GREEN;
static const uint8_t PIN_STATUS_LED_B      = LED_BLUE;

static const uint8_t TMC_ADDR_UPDOWN       = 0;   // A2: MS1 = GND, MS2 = GND
static const uint8_t TMC_ADDR_LEFTRIGHT    = 1;   // A3: MS1 = +3V3, MS2 = GND
static const float   TMC_R_SENSE           = 0.11f;
static const uint32_t TMC_UART_BAUD        = 115200;

TMC2209Stepper driverUpDown   (&Serial1, TMC_R_SENSE, TMC_ADDR_UPDOWN);
TMC2209Stepper driverLeftRight(&Serial1, TMC_R_SENSE, TMC_ADDR_LEFTRIGHT);

// ---------------- SAFE OUTPUTS ----------------
// Drivers disabled, LED data lines LOW, light bar off -- whatever this test does.
static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
}

static void StatusLed(bool r, bool g, bool b) {   // onboard RGB, active-LOW
  digitalWrite(PIN_STATUS_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_B, b ? LOW : HIGH);
}

// ---------------- TEST 1: TX -> PDN_UART path, with plain GPIO ----------------
// Before Serial1 owns the pins: drive D1 HIGH/LOW and read D0 (internal
// pull-down). D0 follows D1 only if something (R10 fitted with 1k, or a
// bridge) connects the module TX pin to PDN_UART. Safe: PDN_UART is an input
// on the TMC2209 except while it is answering a read, and nothing reads here.
// Returns true if the path exists.
static bool TestTxPath() {
  Serial1.end();
  pinMode(PIN_TMC_UART_RX, INPUT_PULLDOWN);
  pinMode(PIN_TMC_UART_TX, OUTPUT);

  digitalWrite(PIN_TMC_UART_TX, HIGH);
  delay(5);
  const int whenHigh = digitalRead(PIN_TMC_UART_RX);
  digitalWrite(PIN_TMC_UART_TX, LOW);
  delay(5);
  const int whenLow = digitalRead(PIN_TMC_UART_RX);
  digitalWrite(PIN_TMC_UART_TX, HIGH);           // UART idle level

  const bool path = (whenHigh == HIGH) && (whenLow == LOW);
  Serial.printf("[1] TX->RX path: D1 HIGH -> D0 reads %s, D1 LOW -> D0 reads %s  => %s\n",
                whenHigh ? "HIGH" : "LOW", whenLow ? "HIGH" : "LOW",
                path ? "PATH PRESENT (R10 fitted / bridged)"
                     : "NO PATH: D1 does not reach PDN_UART (R10 not fitted?) -- drivers can never receive commands");
  if (path) {
    Serial.println("    NOTE: this cannot tell a 1k resistor from a solder bridge. A bridge (0 ohm) is NOT OK;");
    Serial.println("          check R10 visually: it must be a 1 kOhm resistor, not a solder blob.");
  }
  return path;
}

// ---------------- TEST 2/3: UART probe per driver ----------------
static const char *ConnText(uint8_t r) {
  switch (r) {
    case 0:  return "OK";
    case 1:  return "read 0xFFFFFFFF (line stuck HIGH?)";
    case 2:  return "NO REPLY (timeout/CRC -> 0)";
    default: return "?";
  }
}

static bool ProbeDriver(TMC2209Stepper &d, uint8_t addr, const char *name) {
  const uint8_t conn = d.test_connection();
  Serial.printf("[2] %s addr %u: test_connection = %u (%s)\n", name, addr, conn, ConnText(conn));
  if (conn != 0) return false;

  const uint8_t ver = d.version();   // IOIN[31:24]; TMC2209 = 0x21
  Serial.printf("    IOIN version = 0x%02X %s\n", ver, ver == 0x21 ? "(TMC2209 OK)" : "(UNEXPECTED, TMC2209 = 0x21)");
  Serial.printf("    GCONF 0x%08lX  CHOPCONF 0x%08lX  PWMCONF 0x%08lX  DRV_STATUS 0x%08lX\n",
                (unsigned long)d.GCONF(), (unsigned long)d.CHOPCONF(),
                (unsigned long)d.PWMCONF(), (unsigned long)d.DRV_STATUS());
  Serial.printf("    ola=%u olb=%u (1 = open load: normal with no motor)  ot=%u otpw=%u s2ga=%u s2gb=%u\n",
                d.ola(), d.olb(), d.ot(), d.otpw(), d.s2ga(), d.s2gb());

  // Write test: TPOWERDOWN is write-only and harmless (20 = power-on default)
  const uint8_t before = d.IFCNT();
  const bool readOk1 = !d.CRCerror;
  d.TPOWERDOWN(20);
  const uint8_t after = d.IFCNT();
  const bool readOk2 = !d.CRCerror;
  const bool writeOk = readOk1 && readOk2 && (uint8_t)(after - before) == 1;
  Serial.printf("[3] %s write test: IFCNT %u -> %u  => %s\n", name, before, after,
                writeOk ? "WRITE ACKNOWLEDGED" : "WRITE NOT SEEN (TX path, baud, or IFCNT CRC quirk -- retry 'a')");
  return writeOk && (ver == 0x21);
}

static void RunAll() {
  StatusLed(false, false, true);
  Serial.println("\n================ TMC2209 UART PROBE ================");
  Serial.println("~EN on both drivers is HIGH (disabled) for the whole test.");
  const bool path = TestTxPath();

  Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_UART_RX, PIN_TMC_UART_TX);
  delay(10);
  driverUpDown.begin();       // writes GCONF pdn_disable + mstep_reg_select only
  driverLeftRight.begin();

  const bool okUD = ProbeDriver(driverUpDown,    TMC_ADDR_UPDOWN,    "UP/DOWN (A2)");
  const bool okLR = ProbeDriver(driverLeftRight, TMC_ADDR_LEFTRIGHT, "LEFT/RIGHT (A3)");

  Serial.println("---------------- RESULT ----------------");
  Serial.printf("TX path %s | UP/DOWN %s | LEFT/RIGHT %s\n",
                path ? "ok" : "MISSING", okUD ? "PASS" : "FAIL", okLR ? "PASS" : "FAIL");
  if (!path) Serial.println("=> Fit a 1 kOhm resistor at R10 on one module (NOT a solder bridge). See BRINGUP_CHECKLIST.md open item 1.");
  if (path && !(okUD && okLR)) Serial.println("=> Path exists but a driver did not answer: is 12 V on VS? Module seated? MS1/MS2 address?");
  StatusLed(!(okUD && okLR), okUD && okLR, false);
}

static void Help() {
  Serial.println("--- test_tmc_uart ---");
  Serial.println("a   run all tests again");
  Serial.println("s   scan addresses 0..3 (test_connection only)");
  Serial.println("?   this help");
}

void setup() {
  SafeOutputs();
  digitalWrite(PIN_STATUS_LED_R, HIGH); digitalWrite(PIN_STATUS_LED_G, HIGH); digitalWrite(PIN_STATUS_LED_B, HIGH);
  pinMode(PIN_STATUS_LED_R, OUTPUT); pinMode(PIN_STATUS_LED_G, OUTPUT); pinMode(PIN_STATUS_LED_B, OUTPUT);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }
  RunAll();
  Help();
}

void loop() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'a') RunAll();
    else if (c == '?') Help();
    else if (c == 's') {
      for (uint8_t addr = 0; addr < 4; addr++) {
        TMC2209Stepper probe(&Serial1, TMC_R_SENSE, addr);
        const uint8_t r = probe.test_connection();
        Serial.printf("addr %u: %u (%s)\n", addr, r, ConnText(r));
      }
    }
  }
}
