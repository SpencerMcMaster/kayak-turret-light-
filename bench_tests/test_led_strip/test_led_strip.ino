/*
===============================================================================
  BENCH TEST: ONE SK6812 RGBW LED strand
  Board: Arduino Nano ESP32, core 2.0.17, Pin Numbering = "By GPIO number"

  Pick the strand below (TEST_STRIP). The other three data lines are held LOW.

  Power budget (HARD SAFETY CONSTRAINT, same rule as kayak_working.ino):
    (lit pixels / pixels in THIS strand) x (applied brightness / 255) <= 0.20
  computed from the pixel buffer before EVERY show(). It is stricter than
  the main firmware (which divides by all 600 pixels). Over budget, the
  brightness is scaled down, never the pixels turned off. 'k' prints it.

  What it checks: pixel count (the last pixel lights, nothing beyond),
  data direction, colour order (GRBW vs RGBW), every channel, 245 + 330R path.

  Power the strand AFTER the Nano has booted (data lines float during boot).
  Serial Monitor: 115200 baud, line ending "Newline". Type ? for help.
===============================================================================
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber)."
#endif

// ======================= CHOOSE THE STRAND =======================
// 0 = Bottom Left   (D12 -> U1 A0/B0 -> R1 -> J2.1), 256 px
// 1 = Bottom Right  (D11 -> U1 A1/B1 -> R2 -> J2.2), 256 px
// 2 = Cockpit Front (D10 -> U1 A2/B2 -> R3 -> J2.3),  44 px
// 3 = Cockpit Back  (D9  -> U1 A3/B3 -> R4 -> J2.4),  44 px
#ifndef TEST_STRIP
#define TEST_STRIP 0
#endif
// =================================================================

// ---------------- PINS (copied from kayak_working.ino; netlist rev 2.0) ----------------
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12; // GPIO47  net Net-(A1-D12_MISO) -> U1.A0 -> B0 -> R1 -> J2.1
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11; // GPIO38  net Net-(A1-D11_MOSI) -> U1.A1 -> B1 -> R2 -> J2.2
static const uint8_t PIN_LED_COCKPIT_FRONT = D10; // GPIO21  net Net-(U1-A2)       -> U1.A2 -> B2 -> R3 -> J2.3
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;  // GPIO18  net Net-(U1-A3)       -> U1.A3 -> B3 -> R4 -> J2.4
static const uint8_t PIN_PWM_DATA          = D8;  // GPIO17  net Net-(J2-Pin_5)
static const uint8_t PIN_UPDOWN_EN         = D5;  // GPIO8   net Net-(A2-~{EN})
static const uint8_t PIN_LEFTRIGHT_EN      = D2;  // GPIO5   net Net-(A3-~{EN})

static const uint16_t LED_COUNT_BOTTOM_LEFT   = 256;
static const uint16_t LED_COUNT_BOTTOM_RIGHT  = 256;
static const uint16_t LED_COUNT_COCKPIT_FRONT = 44;
static const uint16_t LED_COUNT_COCKPIT_BACK  = 44;

#if TEST_STRIP == 0
static const char *STRIP_NAME = "Bottom Left (D12, J2.1)";
static const uint8_t STRIP_PIN = PIN_LED_BOTTOM_LEFT;   static const uint16_t STRIP_COUNT = LED_COUNT_BOTTOM_LEFT;
#elif TEST_STRIP == 1
static const char *STRIP_NAME = "Bottom Right (D11, J2.2)";
static const uint8_t STRIP_PIN = PIN_LED_BOTTOM_RIGHT;  static const uint16_t STRIP_COUNT = LED_COUNT_BOTTOM_RIGHT;
#elif TEST_STRIP == 2
static const char *STRIP_NAME = "Cockpit Front (D10, J2.3)";
static const uint8_t STRIP_PIN = PIN_LED_COCKPIT_FRONT; static const uint16_t STRIP_COUNT = LED_COUNT_COCKPIT_FRONT;
#elif TEST_STRIP == 3
static const char *STRIP_NAME = "Cockpit Back (D9, J2.4)";
static const uint8_t STRIP_PIN = PIN_LED_COCKPIT_BACK;  static const uint16_t STRIP_COUNT = LED_COUNT_COCKPIT_BACK;
#else
#error "TEST_STRIP must be 0..3"
#endif

// Colour order: most SK6812 RGBW are GRBW. If 'm' shows the wrong colours,
// try NEO_RGBW here (and then in kayak_working.ino).
static const neoPixelType LED_PIXEL_TYPE = (neoPixelType)(NEO_GRBW + NEO_KHZ800);
static const uint8_t BPP = 4;

// Extra pixels sent past the configured count: if any of them light, the
// strand is LONGER than STRIP_COUNT.
static const uint16_t OVERRUN_PIXELS = 8;

static const float LED_POWER_BUDGET_MAX = 0.20f;   // DO NOT raise

Adafruit_NeoPixel strip(STRIP_COUNT + OVERRUN_PIXELS, STRIP_PIN, LED_PIXEL_TYPE);

enum Pattern : uint8_t { PAT_OFF, PAT_CHASE, PAT_STEP, PAT_ENDS, PAT_FILL, PAT_ORDER, PAT_OVERRUN };
static Pattern  g_pat = PAT_OFF;
static uint8_t  g_fillR = 0, g_fillG = 0, g_fillB = 0, g_fillW = 0;
static uint16_t g_pos = 0;
static uint8_t  g_requested = 64;      // requested brightness (0..255)
static uint8_t  g_applied = 0;
static uint16_t g_lit = 0;
static uint16_t g_chaseMs = 20;
static uint32_t g_lastStepMs = 0;
static bool     g_dirty = true;
static uint8_t  g_scratch[(STRIP_COUNT + OVERRUN_PIXELS) * BPP];

// ---------------- SAFE OUTPUTS ----------------
static void SafeOutputs() {
  const uint8_t en[] = { PIN_UPDOWN_EN, PIN_LEFTRIGHT_EN };
  for (uint8_t p : en) { digitalWrite(p, HIGH); pinMode(p, OUTPUT); }      // drivers disabled
  const uint8_t low[] = { PIN_LED_BOTTOM_LEFT, PIN_LED_BOTTOM_RIGHT, PIN_LED_COCKPIT_FRONT,
                          PIN_LED_COCKPIT_BACK, PIN_PWM_DATA };
  for (uint8_t p : low) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }      // other strands: no data
}

// ---------------- BUDGET + SHOW ----------------
// Budget over the CONFIGURED strand length (overrun pixels are only lit by
// the 'v' test, one dim pixel at a time).
static void EnforcePowerBudget() {
  const uint8_t *px = strip.getPixels();
  uint16_t lit = 0;
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    const uint8_t *p = px + (uint32_t)i * BPP;
    if (p[0] | p[1] | p[2] | p[3]) lit++;
  }
  const float frac = (float)lit / (float)STRIP_COUNT;
  uint8_t applied = g_requested;
  if (frac > 0.0f) {
    const float maxB = LED_POWER_BUDGET_MAX * 255.0f / frac;
    if ((float)applied > maxB) applied = (uint8_t)maxB;
    while (applied > 0 && frac * ((float)applied / 255.0f) > LED_POWER_BUDGET_MAX) applied--;
  }
  g_lit = lit;
  g_applied = applied;
}

static void Show() {
  EnforcePowerBudget();                      // every frame, from the real buffer
  uint8_t *px = strip.getPixels();
  const uint32_t n = (uint32_t)strip.numPixels() * BPP;
  memcpy(g_scratch, px, n);
  for (uint32_t i = 0; i < n; i++) px[i] = (uint8_t)(((uint16_t)px[i] * g_applied) / 255u);
  strip.show();
  memcpy(px, g_scratch, n);                  // keep full-scale colours (lossless)
}

static void PrintBudget() {
  const float frac = (float)g_lit / (float)STRIP_COUNT;
  Serial.printf("[k] lit %u/%u (%.3f) x applied %u/255 = %.4f (limit %.2f)  requested %u%s\n",
                g_lit, STRIP_COUNT, (double)frac, g_applied, (double)(frac * g_applied / 255.0f),
                (double)LED_POWER_BUDGET_MAX, g_requested, g_applied < g_requested ? "  ** LIMITED **" : "");
}

// ---------------- PATTERNS ----------------
static void Render() {
  strip.clear();
  switch (g_pat) {
    case PAT_OFF: break;
    case PAT_CHASE:
    case PAT_STEP:
      strip.setPixelColor(g_pos, strip.Color(0, 0, 0, 255));
      break;
    case PAT_ENDS:
      strip.setPixelColor(0, strip.Color(255, 0, 0, 0));                 // first = RED
      strip.setPixelColor(STRIP_COUNT - 1, strip.Color(0, 0, 255, 0));   // last  = BLUE
      break;
    case PAT_FILL:
      for (uint16_t i = 0; i < STRIP_COUNT; i++) strip.setPixelColor(i, strip.Color(g_fillR, g_fillG, g_fillB, g_fillW));
      break;
    case PAT_ORDER:
      strip.setPixelColor(0, strip.Color(255, 0, 0, 0));   // R
      strip.setPixelColor(1, strip.Color(0, 255, 0, 0));   // G
      strip.setPixelColor(2, strip.Color(0, 0, 255, 0));   // B
      strip.setPixelColor(3, strip.Color(0, 0, 0, 255));   // W
      break;
    case PAT_OVERRUN:
      for (uint16_t i = STRIP_COUNT; i < STRIP_COUNT + OVERRUN_PIXELS; i++)
        strip.setPixelColor(i, strip.Color(0, 64, 0, 0));  // dim green past the end
      break;
  }
  g_dirty = false;
  Show();
}

static void SetPattern(Pattern p) {
  g_pat = p;
  g_pos = 0;
  g_dirty = true;
}

static void Help() {
  Serial.printf("--- test_led_strip: %s, %u px ---\n", STRIP_NAME, STRIP_COUNT);
  Serial.println("0          all off");
  Serial.println("c          chase one white pixel end to end (auto)");
  Serial.println("n / p      step one white pixel next / previous (prints index) -- count the strand");
  Serial.println("e          ends: first pixel RED, last pixel BLUE");
  Serial.println("v          overrun: light 8 pixels PAST the configured count (should stay dark)");
  Serial.println("m          colour order: px0 R, px1 G, px2 B, px3 W");
  Serial.println("r g b w    fill the strand with one channel (budget-limited)");
  Serial.println("+ / -      requested brightness +/- 16");
  Serial.println("s <ms>     chase speed (ms per pixel)");
  Serial.println("k          print power-budget numbers");
  Serial.println("?          this help");
}

static void Execute(char c, long val, bool hasVal) {
  switch (c) {
    case '0': SetPattern(PAT_OFF); break;
    case 'c': SetPattern(PAT_CHASE); break;
    case 'n': if (g_pat != PAT_STEP) { SetPattern(PAT_STEP); } else { g_pos = (g_pos + 1) % STRIP_COUNT; g_dirty = true; }
              Serial.printf("pixel %u of %u (last index %u)\n", g_pos, STRIP_COUNT, STRIP_COUNT - 1); break;
    case 'p': if (g_pat != PAT_STEP) { SetPattern(PAT_STEP); } else { g_pos = (g_pos + STRIP_COUNT - 1) % STRIP_COUNT; g_dirty = true; }
              Serial.printf("pixel %u of %u (last index %u)\n", g_pos, STRIP_COUNT, STRIP_COUNT - 1); break;
    case 'e': SetPattern(PAT_ENDS);
              Serial.printf("RED = pixel 0 (data-in end), BLUE = pixel %u (must be the very last LED)\n", STRIP_COUNT - 1); break;
    case 'v': SetPattern(PAT_OVERRUN);
              Serial.printf("lighting pixels %u..%u: if ANY light, the strand is longer than %u\n",
                            STRIP_COUNT, STRIP_COUNT + OVERRUN_PIXELS - 1, STRIP_COUNT); break;
    case 'm': SetPattern(PAT_ORDER); Serial.println("expect: RED, GREEN, BLUE, WHITE from the data-in end"); break;
    case 'r': g_fillR = 255; g_fillG = g_fillB = g_fillW = 0; SetPattern(PAT_FILL); break;
    case 'g': g_fillG = 255; g_fillR = g_fillB = g_fillW = 0; SetPattern(PAT_FILL); break;
    case 'b': g_fillB = 255; g_fillR = g_fillG = g_fillW = 0; SetPattern(PAT_FILL); break;
    case 'w': g_fillW = 255; g_fillR = g_fillG = g_fillB = 0; SetPattern(PAT_FILL); break;
    case '+': g_requested = (uint8_t)min(255, g_requested + 16); g_dirty = true; break;
    case '-': g_requested = (uint8_t)max(0, g_requested - 16); g_dirty = true; break;
    case 's': if (hasVal && val >= 1 && val <= 1000) { g_chaseMs = (uint16_t)val; Serial.printf("chase %u ms/pixel\n", g_chaseMs); }
              else Serial.println("s: 1..1000"); break;
    case 'k': PrintBudget(); break;
    case '?': Help(); break;
    default: Serial.printf("unknown '%c' -- ? for help\n", c); break;
  }
  if (c == 'r' || c == 'g' || c == 'b' || c == 'w' || c == '+' || c == '-') {
    Render();
    PrintBudget();
  }
}

static char    g_line[16];
static uint8_t g_len = 0;

void setup() {
  SafeOutputs();
  strip.begin();
  strip.clear();
  Show();                                    // dark, through the budget
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); }
  Serial.printf("\n=== test_led_strip: %s ===\n", STRIP_NAME);
  Help();
}

void loop() {
  const uint32_t now = millis();
  if (g_pat == PAT_CHASE && (now - g_lastStepMs) >= g_chaseMs) {
    g_lastStepMs = now;
    g_pos = (g_pos + 1) % STRIP_COUNT;
    if (g_pos == 0) Serial.println("chase wrapped (pixel 0)");
    g_dirty = true;
  }
  if (g_dirty) Render();

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      g_line[g_len] = '\0';
      if (g_len) {
        char *end = nullptr;
        const long v = strtol(g_line + 1, &end, 10);
        Execute(g_line[0], v, end != g_line + 1);
      }
      g_len = 0;
    } else if (g_len == 0 && c != 's' && c != ' ') {
      Execute((char)c, 0, false);            // single-key commands act immediately
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = (char)c;
    }
  }
}
