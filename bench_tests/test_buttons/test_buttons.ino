/*
===============================================================================
  BENCH TEST: the four J6 push-buttons
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  Prints every debounced press / release with hold time, the raw pin levels
  on request, and what the main firmware would do with the combination
  (pan jog, lift chord, brightness click, mode click / long press).
  No motor, strip or light bar is driven: all outputs are held safe.

  Wiring (netlist): momentary to J6.5 COMMON GND, INPUT_PULLUP, no resistors.
    J6.1 BRIGHTNESS -> A3   J6.2 MODE -> A2   J6.3 LEFT -> A1   J6.4 RIGHT -> A0

  Serial Monitor: 115200 baud. Type ? for help.
===============================================================================
*/

#include <Arduino.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_PB_BRIGHTNESS     = A3;  // GPIO4   net Net-(J6-Pin_1) -> J6.1 "PB1 BRIGHTNESS"
static const uint8_t PIN_PB_MODE           = A2;  // GPIO3   net Net-(J6-Pin_2) -> J6.2 "PB1 MODE"
static const uint8_t PIN_PB_TURN_LEFT      = A1;  // GPIO2   net Net-(J6-Pin_3) -> J6.3 "PB1 LEFT"
static const uint8_t PIN_PB_TURN_RIGHT     = A0;  // GPIO1   net Net-(J6-Pin_4) -> J6.4 "PB1 RIGHT"
static const uint8_t PIN_UPDOWN_EN         = D5;
static const uint8_t PIN_LEFTRIGHT_EN      = D2;
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12;
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11;
static const uint8_t PIN_LED_COCKPIT_FRONT = D10;
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;
static const uint8_t PIN_PWM_DATA          = D8;

static const uint16_t BUTTON_DEBOUNCE_MS = 25;     // same as kayak_working.ino
static const uint16_t MODE_LONG_PRESS_MS = 1000;   // same as kayak_working.ino

struct Btn {
  const char *name;
  uint8_t  pin;
  bool     stable;       // debounced level (HIGH = released)
  bool     raw;
  uint32_t changeMs;
  uint32_t pressedMs;
  uint32_t presses;
  uint32_t bounces;      // raw changes that did not survive the debounce
};
static Btn g_btn[4] = {
  { "BRIGHTNESS (J6.1, A3)", PIN_PB_BRIGHTNESS, HIGH, HIGH, 0, 0, 0, 0 },
  { "MODE       (J6.2, A2)", PIN_PB_MODE,       HIGH, HIGH, 0, 0, 0, 0 },
  { "LEFT       (J6.3, A1)", PIN_PB_TURN_LEFT,  HIGH, HIGH, 0, 0, 0, 0 },
  { "RIGHT      (J6.4, A0)", PIN_PB_TURN_RIGHT, HIGH, HIGH, 0, 0, 0, 0 },
};
enum { B_BRIGHT = 0, B_MODE, B_LEFT, B_RIGHT };

static bool g_chordSeen = false;
static bool g_longFired = false;
static char g_lastAction[48] = "";

static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
}

static bool Held(uint8_t i) { return g_btn[i].stable == LOW; }

static void ReportAction() {
  // What kayak_working.ino would do with the current combination
  char a[48];
  const bool b = Held(B_BRIGHT), l = Held(B_LEFT), r = Held(B_RIGHT);
  if (b && l && !r)       snprintf(a, sizeof(a), "LIFT DOWN (chord)");
  else if (b && r && !l)  snprintf(a, sizeof(a), "LIFT UP (chord)");
  else if (b && l && r)   snprintf(a, sizeof(a), "nothing (BRIGHTNESS+LEFT+RIGHT)");
  else if (!b && l && r)  snprintf(a, sizeof(a), "HAZARD flash (no pan)");
  else if (!b && l)       snprintf(a, sizeof(a), "PAN LEFT");
  else if (!b && r)       snprintf(a, sizeof(a), "PAN RIGHT");
  else                    snprintf(a, sizeof(a), "idle");
  if (strcmp(a, g_lastAction) != 0) {
    strncpy(g_lastAction, a, sizeof(g_lastAction) - 1);
    Serial.printf("        -> main firmware: %s\n", a);
  }
}

static void PrintRaw() {
  Serial.print("[raw] ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("%s=%s  ", g_btn[i].name, digitalRead(g_btn[i].pin) ? "HIGH" : "LOW(pressed)");
  }
  Serial.println();
}

static void PrintStats() {
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("[stats] %s presses %lu, filtered bounces %lu\n", g_btn[i].name,
                  (unsigned long)g_btn[i].presses, (unsigned long)g_btn[i].bounces);
  }
}

void setup() {
  SafeOutputs();
  for (uint8_t i = 0; i < 4; i++) pinMode(g_btn[i].pin, INPUT_PULLUP);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }
  Serial.println("\n=== test_buttons ===");
  Serial.println("Press each button: expect one 'pressed' and one 'released' line per press.");
  Serial.println("Commands: r = raw pin levels, s = press/bounce counts, ? = this help");
  PrintRaw();
  for (uint8_t i = 0; i < 4; i++) {
    if (digitalRead(g_btn[i].pin) == LOW) {
      Serial.printf("WARNING: %s reads LOW with nothing pressed -> stuck button or short to GND\n", g_btn[i].name);
    }
  }
}

void loop() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    Btn &b = g_btn[i];
    const bool raw = digitalRead(b.pin);
    if (raw != b.raw) {
      if ((now - b.changeMs) < BUTTON_DEBOUNCE_MS && raw == b.stable) b.bounces++;
      b.raw = raw;
      b.changeMs = now;
    }
    if ((now - b.changeMs) >= BUTTON_DEBOUNCE_MS && b.stable != raw) {
      b.stable = raw;
      if (raw == LOW) {
        b.pressedMs = now;
        b.presses++;
        Serial.printf("[%7lu ms] %s pressed\n", (unsigned long)now, b.name);
        if (i == B_BRIGHT) g_chordSeen = false;
        if (i == B_MODE)   g_longFired = false;
      } else {
        const uint32_t held = now - b.pressedMs;
        Serial.printf("[%7lu ms] %s released (held %lu ms)\n", (unsigned long)now, b.name, (unsigned long)held);
        if (i == B_BRIGHT) Serial.printf("        -> main firmware: %s\n",
                                         g_chordSeen ? "no brightness step (was a lift chord)" : "BRIGHTNESS click: next level");
        if (i == B_MODE && !g_longFired) Serial.println("        -> main firmware: MODE click: next lighting mode");
      }
      ReportAction();
    }
  }
  if (Held(B_BRIGHT) && (Held(B_LEFT) || Held(B_RIGHT))) g_chordSeen = true;
  if (Held(B_MODE) && !g_longFired && (now - g_btn[B_MODE].pressedMs) >= MODE_LONG_PRESS_MS) {
    g_longFired = true;
    Serial.println("        -> main firmware: MODE LONG PRESS: re-zero heading");
  }

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'r') PrintRaw();
    else if (c == 's') PrintStats();
    else if (c == '?') Serial.println("r = raw pin levels, s = press/bounce counts");
  }
}
