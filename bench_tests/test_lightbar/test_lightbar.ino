/*
===============================================================================
  BENCH TEST: light-bar PWM output (D8 -> J2.5 "PWM CONTROL" -> MOSFET module)
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  LEDC hardware PWM, 8-bit, default 5 kHz (the MOSFET module accepts
  0-20 kHz). Starts at duty 0. The main firmware only turns the bars on
  with the lift at its UP limit; this test has no such gate.

  Step 1: with NOTHING on J2.5, check D8 with a DMM:
          duty 64 -> ~0.83 V DC, 128 -> ~1.65 V, 255 -> ~3.3 V; Hz mode ~5000.
  Step 2: connect the MOSFET module input (bars + 12 V on) and repeat.

  Serial Monitor: 115200 baud, line ending "Newline". Type ? for help.
===============================================================================
*/

#include <Arduino.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_PWM_DATA          = D8;  // GPIO17  net Net-(J2-Pin_5) -> J2.5 "PWM CONTROL"
static const uint8_t PIN_UPDOWN_EN         = D5;
static const uint8_t PIN_LEFTRIGHT_EN      = D2;
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12;
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11;
static const uint8_t PIN_LED_COCKPIT_FRONT = D10;
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;

static const uint8_t  LIGHTBAR_LEDC_CHANNEL = 0;
static const uint8_t  LIGHTBAR_PWM_RES_BITS = 8;
static const uint32_t LIGHTBAR_PWM_FREQ_HZ  = 5000;   // same as kayak_working.ino
static const uint32_t MAX_FREQ_HZ           = 20000;  // MOSFET module limit

static uint8_t  g_duty = 0;
static uint32_t g_freq = LIGHTBAR_PWM_FREQ_HZ;
static bool     g_ramp = false;
static int16_t  g_rampDuty = 0;
static int8_t   g_rampDir = 1;
static uint32_t g_lastRampMs = 0;

static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
}

static void SetDuty(uint8_t d) {
  g_duty = d;
  ledcWrite(LIGHTBAR_LEDC_CHANNEL, d);
}

static void SetFreq(uint32_t hz) {
  const double actual = ledcSetup(LIGHTBAR_LEDC_CHANNEL, hz, LIGHTBAR_PWM_RES_BITS);
  ledcWrite(LIGHTBAR_LEDC_CHANNEL, g_duty);
  g_freq = hz;
  Serial.printf("frequency %lu Hz (LEDC actual %.1f Hz)\n", (unsigned long)hz, actual);
}

static void Help() {
  Serial.println("--- test_lightbar ---");
  Serial.println("d <0-255>   set duty (expected DMM DC ~ duty/255 x 3.3 V)");
  Serial.println("0           off");
  Serial.println("1..4        main-firmware brightness levels (64/128/191/255)");
  Serial.println("r           ramp 0 -> 255 -> 0 continuously (r again to stop)");
  Serial.println("f <hz>      PWM frequency, 100..20000 (default 5000)");
  Serial.println("?           this help");
}

static char    g_line[16];
static uint8_t g_len = 0;

static void Execute(const char *line) {
  const char c = line[0];
  const long v = strtol(line + 1, nullptr, 10);
  switch (c) {
    case 'd': if (v >= 0 && v <= 255) { g_ramp = false; SetDuty((uint8_t)v);
                Serial.printf("duty %ld/255 -> expect ~%.2f V DC on D8\n", v, v / 255.0 * 3.3); }
              else Serial.println("d: 0..255"); break;
    case '0': g_ramp = false; SetDuty(0); Serial.println("off"); break;
    case '1': case '2': case '3': case '4': {
      static const uint8_t lv[4] = { 64, 128, 191, 255 };
      g_ramp = false; SetDuty(lv[c - '1']);
      Serial.printf("level %c: duty %u -> expect ~%.2f V DC\n", c, g_duty, g_duty / 255.0 * 3.3);
      break;
    }
    case 'r': g_ramp = !g_ramp; g_rampDuty = 0; g_rampDir = 1; Serial.printf("ramp %s\n", g_ramp ? "ON" : "off");
              if (!g_ramp) SetDuty(0); break;
    case 'f': if (v >= 100 && v <= (long)MAX_FREQ_HZ) SetFreq((uint32_t)v);
              else Serial.println("f: 100..20000"); break;
    case '?': Help(); break;
    default: Serial.printf("unknown '%c'\n", c); break;
  }
}

void setup() {
  SafeOutputs();                                     // D8 LOW first
  ledcSetup(LIGHTBAR_LEDC_CHANNEL, g_freq, LIGHTBAR_PWM_RES_BITS);
  ledcWrite(LIGHTBAR_LEDC_CHANNEL, 0);               // duty 0 before attaching
  ledcAttachPin(PIN_PWM_DATA, LIGHTBAR_LEDC_CHANNEL);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }
  Serial.println("\n=== test_lightbar: duty 0 ===");
  Help();
}

void loop() {
  if (g_ramp && (millis() - g_lastRampMs) >= 10) {   // full 0->255->0 cycle ~5 s
    g_lastRampMs = millis();
    g_rampDuty += g_rampDir;
    if (g_rampDuty >= 255) { g_rampDuty = 255; g_rampDir = -1; }
    if (g_rampDuty <= 0)   { g_rampDuty = 0;   g_rampDir = 1; }
    SetDuty((uint8_t)g_rampDuty);
  }
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      g_line[g_len] = '\0';
      if (g_len) Execute(g_line);
      g_len = 0;
    } else if (g_len == 0 && (c == '0' || c == 'r' || c == '?' || (c >= '1' && c <= '4'))) {
      char one[2] = { (char)c, 0 };
      Execute(one);                                  // single-key commands act immediately
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = (char)c;
    }
  }
}
