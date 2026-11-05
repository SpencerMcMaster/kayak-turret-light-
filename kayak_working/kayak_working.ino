/*
===============================================================================
  Project: Kayak Light Bar Controller
  Board:   Arduino Nano ESP32-S3

  Description:
    Multi-channel RGBW lighting and motion controller for a kayak-mounted
    light bar system.

    - 4x SK6812 RGBW (12V) addressable LED strips
      (driven via SN74HCT245 level shifter + 330Ω series resistors)
    - 2x TMC2209 stepper motor drivers (STEP/DIR mode)
      - Up/Down lift axis
      - Left/Right rotation axis
    - 4x momentary pushbuttons
      - Brightness
      - Mode select
      - Turn left
      - Turn right
    - GY-521 MPU6050 IMU for motion / orientation sensing
    - Optional PWM-controlled auxiliary output

  Electrical Notes:
    - LED strips powered from 12V supply (NOT from Arduino)
    - Logic powered from Arduino 3.3V
    - Common ground required between Arduino, LED power, and drivers
    - RGBW channel order assumed GRBW (change to RGBW if required)

  Design Goals:
    - Deterministic behavior (no blocking delays)
    - Safe power-on state
    - Readable, modular structure
    - Easy future expansion (IMU leveling, auto modes, etc.)

  Author: [SPENCER MCMASTER]
  Date:   [DEC 14, 2025]

  PIN SOURCE OF TRUTH (Nov 2026):
    PIN DEFINITIONS match the KiCad netlist "kayak working.net" (rev 2.0,
    Eeschema 9.0.6) pin-for-pin. Firmware follows KiCad, never the reverse.
    Re-check after any schematic change with:
        python3 tools/netlist_audit.py "kayak working.net"

  BUILD:
    FQBN arduino:esp32:nano_nora:PinNumbers=byGPIONumber, core 2.0.17.
    (Arduino IDE: Tools > Pin Numbering > "By GPIO number (legacy)".)
    The D0..D13 / A0..A7 names below still work; in this mode they resolve
    to raw GPIO numbers, which is what every library here expects.
===============================================================================
*/

// ----------------------------------------------------------------------------
// PIN NUMBERING GUARD
// With the default "By Arduino pin" numbering, Adafruit_NeoPixel on core
// 2.0.17 (ESP-IDF 4.4 RMT path, esp.c) passes the pin number straight to the
// IDF RMT driver WITHOUT remapping, while its pinMode()/digitalWrite() ARE
// remapped. Result: e.g. a strip on D9 drives GPIO18 low and emits data on
// GPIO9 (= D6, a STEP line). Build "By GPIO number" so every API agrees.
// ----------------------------------------------------------------------------
#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
#error "Build with Tools > Pin Numbering > By GPIO number (FQBN option PinNumbers=byGPIONumber). See PIN NUMBERING GUARD."
#endif

#include <Arduino.h>                          // Core Arduino definitions
#include <Wire.h>                             // I2C communication (MPU6050)
#include <Adafruit_NeoPixel.h>                // RGBW LED control (SK6812)
#include <Adafruit_MPU6050.h>                 // GY-521 / MPU6050 IMU
#include <Adafruit_Sensor.h>                  // Required by Adafruit_MPU6050
#include <AccelStepper.h>                     // Step/Dir stepper motor control



// ============================================================================
// COMPILE-TIME CONFIGURATION CONSTANTS
// ============================================================================

// -------------------- LED STRIP CONFIGURATION --------------------
// Set the exact number of LEDs on each physical run.
// These MUST match the real strip lengths.

static const uint16_t LED_COUNT_BOTTOM_LEFT   = 256;
static const uint16_t LED_COUNT_BOTTOM_RIGHT  = 256;
static const uint16_t LED_COUNT_COCKPIT_FRONT = 44;
static const uint16_t LED_COUNT_COCKPIT_BACK  = 44;

// SK6812 RGBW strips (12V) timing + color order
// Most SK6812 RGBW strips are GRBW.
// If colors are wrong, change NEO_GRBW -> NEO_RGBW.
static const neoPixelType LED_PIXEL_TYPE =
  (neoPixelType)(NEO_GRBW + NEO_KHZ800);

// Global brightness limits (important for 12V current control)
static const uint8_t BRIGHTNESS_MIN     = 16;   // very dim but visible
static const uint8_t BRIGHTNESS_MAX     = 220;  // DO NOT run full 255 on 12V bench
static const uint8_t BRIGHTNESS_DEFAULT = 96;

// Number of discrete brightness steps for PB1
static const uint8_t BRIGHTNESS_LEVELS = 4;

// -------------------- ANIMATION TIMING --------------------
// All values in milliseconds unless otherwise noted

static const uint16_t KNIGHT_RIDER_INTERVAL_MS = 30;
static const uint16_t TURN_SIGNAL_INTERVAL_MS  = 70;
static const uint16_t AMBIENT_PULSE_INTERVAL_MS = 20;

// Ambient pulse parameters
static const float AMBIENT_PULSE_SPEED = 0.03f; // radians per frame
static const uint8_t AMBIENT_W_MIN = 20;
static const uint8_t AMBIENT_W_MAX = 100;

// -------------------- STEPPER MOTION LIMITS --------------------
// These are SOFT limits in steps — tune after mechanical testing.

// Left / Right rotation axis
static const long LR_STEP_INCREMENT = 800;   // steps per button activation
static const long LR_STEP_LIMIT     = 6400;  // +/- hard limit

// Up / Down lift axis (reserved for future logic)
static const long UD_STEP_INCREMENT = 800;
static const long UD_STEP_LIMIT     = 4000;

// Stepper speed/acceleration (AccelStepper units = steps/sec)
static const float STEPPER_MAX_SPEED = 2000.0f;
static const float STEPPER_ACCEL     = 4000.0f;

// -------------------- INPUT HANDLING --------------------
// Button debounce time
static const uint16_t BUTTON_DEBOUNCE_MS = 25;

// -------------------- SAFETY / STARTUP --------------------
// Startup delay to allow power rails and level shifter to stabilize
static const uint16_t STARTUP_DELAY_MS = 150;

// ============================================================================
// END OF CONFIGURATION CONSTANTS
// ============================================================================



// ============================================================================
// PIN DEFINITIONS (FROM KiCad NETLIST "kayak working.net" rev 2.0)
// ============================================================================
// Every line cites its netlist net name. Refdes: A1 = Nano ESP32 socket,
// A2 = TMC2209 UP/DOWN (motor on J3), A3 = TMC2209 LEFT/RIGHT (motor on J4),
// U1 = 74HCT245 (DIR "A->B" tied +5V, CE tied GND), U2 = GY-521.
// GPIO numbers in comments: arduino:esp32 2.0.17 variants/arduino_nano_nora.

// -------------------- LED DATA OUTPUTS --------------------
// MCU -> U1 A-side (3.3 V in, HCT VIH = 2.0 V) -> B-side (5 V) -> 330R -> J2
static const uint8_t PIN_LED_BOTTOM_LEFT   = D12; // GPIO47  net Net-(A1-D12_MISO) -> U1.A0 -> B0 -> R1 -> J2.1 "TO BOTTOM LEFT LED DATA PIN"
static const uint8_t PIN_LED_BOTTOM_RIGHT  = D11; // GPIO38  net Net-(A1-D11_MOSI) -> U1.A1 -> B1 -> R2 -> J2.2 "TO BOTTOM RIGHT LED DATA PIN"
static const uint8_t PIN_LED_COCKPIT_FRONT = D10; // GPIO21  net Net-(U1-A2)       -> U1.A2 -> B2 -> R3 -> J2.3 "TO COCKPIT FRONT LED DATA PIN"
static const uint8_t PIN_LED_COCKPIT_BACK  = D9;  // GPIO18  net Net-(U1-A3)       -> U1.A3 -> B3 -> R4 -> J2.4 "TO COCKPIT BACK LED DATA PIN"

// -------------------- LIGHT-BAR PWM OUTPUT --------------------
// NOT level-shifted: 3.3 V PWM straight to the MOSFET module input.
static const uint8_t PIN_PWM_DATA          = D8;  // GPIO17  net Net-(J2-Pin_5)    -> J2.5 "PWM CONTROL"

// -------------------- STEPPER DRIVER PINS (TMC2209 STEP/DIR/EN) --------------------
// EN is active-LOW (~{EN}); module has a 20k pull-up to VIO, so EN floats
// HIGH (driver disabled) while the ESP32 is in reset / booting.

// Up / Down lift axis -- TMC2209 A2, motor terminal J3 "UP/DOWN"
static const uint8_t PIN_UPDOWN_EN         = D5;  // GPIO8   net Net-(A2-~{EN})    -> A2.8  ~{EN}
static const uint8_t PIN_UPDOWN_STEP       = D6;  // GPIO9   net Net-(A2-STEP)     -> A2.2  STEP
static const uint8_t PIN_UPDOWN_DIR        = D7;  // GPIO10  net Net-(A2-DIR)      -> A2.1  DIR
static const uint8_t PIN_UPDOWN_DIAG       = A6;  // GPIO13  net Net-(A1-PadA6)    <- A2    DIAG_A2 (StallGuard/error out, VIO = 3.3 V)

// Left / Right pan axis -- TMC2209 A3, motor terminal J4 "LEFT/RIGHT"
static const uint8_t PIN_LEFTRIGHT_EN      = D2;  // GPIO5   net Net-(A3-~{EN})    -> A3.8  ~{EN}
static const uint8_t PIN_LEFTRIGHT_STEP    = D3;  // GPIO6   net Net-(A3-STEP)     -> A3.2  STEP
static const uint8_t PIN_LEFTRIGHT_DIR     = D4;  // GPIO7   net Net-(A3-DIR)      -> A3.1  DIR
static const uint8_t PIN_LEFTRIGHT_DIAG    = A7;  // GPIO14  net Net-(A1-PadA7)    <- A3    DIAG_A3 (StallGuard/error out, VIO = 3.3 V)

// -------------------- TMC2209 UART (shared bus, both drivers) --------------------
// Used by Serial1 (Serial = native USB CDC, so no clash with debug output).
// BTT TMC2209 V1.3 (module schematic + manual): header pin 5 "RX"/UART_RX
// is wired DIRECTLY to PDN_UART; header pin 4 "TX"/UART_TX reaches PDN_UART
// only through R10, which is NOT FITTED at the factory. There is no series
// resistor on the module. See BRINGUP open item "TMC UART wiring" before
// Phase 2 relies on PIN_TMC_UART_TX.
static const uint8_t PIN_TMC_UART_RX       = D0;  // GPIO44  net Net-(A1-D0{slash}RX) -> A2.5, A3.5 UART_RX (= PDN_UART)
static const uint8_t PIN_TMC_UART_TX       = D1;  // GPIO43  net Net-(A1-D1{slash}TX) -> A2.4, A3.4 UART_TX (-> R10, NC by default)

// UART slave addresses from MS1/MS2 tie-offs (MS1 = bit 0, MS2 = bit 1)
static const uint8_t TMC_ADDR_UPDOWN       = 0;   // A2: MS1 = GND, MS2 = GND
static const uint8_t TMC_ADDR_LEFTRIGHT    = 1;   // A3: MS1 = +3V3, MS2 = GND

// -------------------- BUTTON INPUTS (J6) --------------------
// Active-low momentary to J6.5 COMMON GND, INPUT_PULLUP, no external resistors.
// NOTE: J6 terminal order is reversed relative to A0..A3.
static const uint8_t PIN_PB_BRIGHTNESS     = A3;  // GPIO4   net Net-(J6-Pin_1)    -> J6.1 "PB1 BRIGHTNESS"
static const uint8_t PIN_PB_MODE           = A2;  // GPIO3   net Net-(J6-Pin_2)    -> J6.2 "PB1 MODE"
static const uint8_t PIN_PB_TURN_LEFT      = A1;  // GPIO2   net Net-(J6-Pin_3)    -> J6.3 "PB1 LEFT"
static const uint8_t PIN_PB_TURN_RIGHT     = A0;  // GPIO1   net Net-(J6-Pin_4)    -> J6.4 "PB1 RIGHT"

// -------------------- IMU (GY-521 MPU6050) --------------------
// U2 VCC = +5V (module LDO), AD0 unconnected (module pull-down -> 0x68), INT unconnected.
static const uint8_t PIN_I2C_SDA           = A4;  // GPIO11  net Net-(A1-A4{slash}SDA) -> U2.4 SDA
static const uint8_t PIN_I2C_SCL           = A5;  // GPIO12  net Net-(A1-A5{slash}SCL) -> U2.3 SCL
static const uint8_t IMU_I2C_ADDR          = 0x68;

// -------------------- UNUSED / RESERVED MCU PINS --------------------
// D13 (GPIO48, SCK/LED_BUILTIN): unconnected-(A1-D13_SCK-PadD13) -- off-limits.
// B0, B1, RST, 5V: unconnected. VIN = +12V (J5), 3V3 -> TMC VIO + A3 MS1.

// ============================================================================
// END OF PIN DEFINITIONS
// ============================================================================




// ============================================================================
// GLOBAL HARDWARE OBJECTS
// ============================================================================

// -------------------- LED STRIP OBJECTS --------------------
// SK6812 RGBW (12V) addressable LED strips

Adafruit_NeoPixel stripBottomLeft(
  LED_COUNT_BOTTOM_LEFT,
  PIN_LED_BOTTOM_LEFT,
  LED_PIXEL_TYPE
);

Adafruit_NeoPixel stripBottomRight(
  LED_COUNT_BOTTOM_RIGHT,
  PIN_LED_BOTTOM_RIGHT,
  LED_PIXEL_TYPE
);

Adafruit_NeoPixel stripCockpitFront(
  LED_COUNT_COCKPIT_FRONT,
  PIN_LED_COCKPIT_FRONT,
  LED_PIXEL_TYPE
);

Adafruit_NeoPixel stripCockpitBack(
  LED_COUNT_COCKPIT_BACK,
  PIN_LED_COCKPIT_BACK,
  LED_PIXEL_TYPE
);

// Convenience array for iterating all LED strips uniformly
Adafruit_NeoPixel* LED_STRIPS[] = {
  &stripBottomLeft,
  &stripBottomRight,
  &stripCockpitFront,
  &stripCockpitBack
};

static const uint8_t LED_STRIP_COUNT =
  sizeof(LED_STRIPS) / sizeof(LED_STRIPS[0]);

// -------------------- IMU OBJECT --------------------
// GY-521 MPU6050
Adafruit_MPU6050 imu;

// -------------------- STEPPER MOTOR OBJECTS --------------------
// AccelStepper DRIVER mode: (STEP pin, DIR pin)

// Up / Down lift axis
AccelStepper stepperUpDown(
  AccelStepper::DRIVER,
  PIN_UPDOWN_STEP,
  PIN_UPDOWN_DIR
);

// Left / Right rotation axis
AccelStepper stepperLeftRight(
  AccelStepper::DRIVER,
  PIN_LEFTRIGHT_STEP,
  PIN_LEFTRIGHT_DIR
);

// ============================================================================
// END OF GLOBAL HARDWARE OBJECTS
// ============================================================================



// ============================================================================
// GLOBAL STATE (RUNTIME VARIABLES)
// ============================================================================

// -------------------- LIGHTING MODES --------------------
enum LightingMode : uint8_t {
  MODE_OFF = 0,
  MODE_NAV_LIGHTS,
  MODE_WHITE_FLOOD,
  MODE_KNIGHT_RIDER,
  MODE_AMBIENT_PULSE,
  MODE_COUNT
};

static LightingMode g_mode = MODE_NAV_LIGHTS;

// -------------------- BRIGHTNESS STATE --------------------
// We use discrete brightness levels mapped into [BRIGHTNESS_MIN..BRIGHTNESS_MAX]
static uint8_t g_brightnessIndex = 0;      // 0..(BRIGHTNESS_LEVELS-1)
static uint8_t g_brightnessValue = BRIGHTNESS_DEFAULT;

// -------------------- TURN / HAZARD STATE --------------------
// Hold-to-jog behavior will use PB3/PB4 states directly,
// but we still keep flags for overlay rendering logic.
static bool g_turnLeftHeld  = false;
static bool g_turnRightHeld = false;

// -------------------- ANIMATION STATE --------------------
static uint32_t g_lastKnightMs = 0;
static uint32_t g_lastPulseMs  = 0;
static uint32_t g_lastTurnMs   = 0;

static uint16_t g_knightPos = 0;   // virtual position across cockpit front+back span
static int8_t   g_knightDir = 1;

static float    g_pulsePhase = 0.0f;
static uint16_t g_turnFrame  = 0;

// -------------------- STEPPER JOG STATE --------------------
// The steppers will move continuously while the button is held.
// We enforce soft limits in "steps" from an assumed 0..MAX travel range.
//
// IMPORTANT: These limits should be tuned after mechanical testing.
// For rotation: 0 = full left, MAX = full right (180° total travel assumption).
// For lift:     0 = full down, MAX = full up.

static long g_lrPosSteps = 0;     // current logical position (rotation)
static long g_udPosSteps = 0;     // current logical position (lift)

// Jog speeds (steps/sec). Positive = right/up, negative = left/down.
// We will clamp motion at the soft limits.
static float g_lrJogSpeed = 0.0f;
static float g_udJogSpeed = 0.0f;

// If you later add lift jog buttons, these will be used.
// For now we keep them defined to support future expansion.
static bool g_liftUpHeld   = false;
static bool g_liftDownHeld = false;

// -------------------- BUTTON EDGE STATE (DEBOUNCE) --------------------
// We will implement a small debounce/edge detector struct in the next block.
// These placeholders exist so the "state section" documents what we track.
static bool g_brightnessPressedEdge = false;
static bool g_modePressedEdge       = false;

// ============================================================================
// END OF GLOBAL STATE
// ============================================================================



// ============================================================================
// HELPERS + INPUT (BUTTON DEBOUNCE / EDGE DETECTION)
// ============================================================================

struct DebouncedButton {
  uint8_t pin;
  bool lastStable;       // debounced stable state
  bool lastRaw;          // last raw sample
  uint32_t lastChangeMs; // last time raw changed
  bool pressedEdge;      // one-shot on press (HIGH->LOW)
};

// -------------------- LED HELPERS --------------------
static inline uint32_t ColorRGBW(Adafruit_NeoPixel &s,
                                uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  return s.Color(r, g, b, w);
}

static void ClearAllStrips() {
  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *s = LED_STRIPS[si];
    for (uint16_t i = 0; i < s->numPixels(); i++) {
      s->setPixelColor(i, 0);
    }
  }
}

static void ShowAllStrips() {
  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    LED_STRIPS[si]->show();
  }
}

static void SetBrightnessAll(uint8_t b) {
  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    LED_STRIPS[si]->setBrightness(b);
  }
}

static uint8_t BrightnessFromIndex(uint8_t idx) {
  if (BRIGHTNESS_LEVELS <= 1) return BRIGHTNESS_DEFAULT;
  if (idx >= (BRIGHTNESS_LEVELS - 1)) return BRIGHTNESS_MAX;

  const float t = (float)idx / (float)(BRIGHTNESS_LEVELS - 1);
  const float v = (float)BRIGHTNESS_MIN + t * (float)(BRIGHTNESS_MAX - BRIGHTNESS_MIN);
  return (uint8_t)(v + 0.5f);
}

static void ApplyBrightnessFromIndex() {
  g_brightnessValue = BrightnessFromIndex(g_brightnessIndex);
  SetBrightnessAll(g_brightnessValue);
}

// -------------------- PWM HELPER (AUX OUTPUT) --------------------
static void SetAuxPwmDuty(uint8_t duty /*0..255*/) {
  analogWrite(PIN_PWM_DATA, duty);
}

// -------------------- BUTTON INSTANCES --------------------
static DebouncedButton btnBrightness = { PIN_PB_BRIGHTNESS, HIGH, HIGH, 0, false };
static DebouncedButton btnMode       = { PIN_PB_MODE,       HIGH, HIGH, 0, false };
static DebouncedButton btnTurnLeft   = { PIN_PB_TURN_LEFT,  HIGH, HIGH, 0, false };
static DebouncedButton btnTurnRight  = { PIN_PB_TURN_RIGHT, HIGH, HIGH, 0, false };

// -------------------- BUTTON FUNCTIONS --------------------
static void UpdateButton(DebouncedButton &b, uint32_t nowMs) {
  const bool raw = digitalRead(b.pin);

  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChangeMs = nowMs;
  }

  if ((nowMs - b.lastChangeMs) >= BUTTON_DEBOUNCE_MS) {
    if (b.lastStable != raw) {
      b.lastStable = raw;
      if (raw == LOW) b.pressedEdge = true; // active-low press edge
    }
  }
}

static bool ConsumePressedEdge(DebouncedButton &b) {
  if (b.pressedEdge) {
    b.pressedEdge = false;
    return true;
  }
  return false;
}

static bool IsHeld(const DebouncedButton &b) {
  return (b.lastStable == LOW);
}

static void UpdateInputs(uint32_t nowMs) {
  UpdateButton(btnBrightness, nowMs);
  UpdateButton(btnMode,       nowMs);
  UpdateButton(btnTurnLeft,   nowMs);
  UpdateButton(btnTurnRight,  nowMs);

  g_brightnessPressedEdge = ConsumePressedEdge(btnBrightness);
  g_modePressedEdge       = ConsumePressedEdge(btnMode);

  g_turnLeftHeld  = IsHeld(btnTurnLeft);
  g_turnRightHeld = IsHeld(btnTurnRight);
}

// ============================================================================
// END OF HELPERS + INPUT
// ============================================================================




// ============================================================================
// RENDER FUNCTIONS (BASE MODES + TURN/HAZARD OVERLAY)
// ============================================================================
//
// Conventions used here:
// - Bottom Left  strip = Red (port-style)
// - Bottom Right strip = Green (starboard-style)
// - Cockpit Front/Back = White channel (natural white)
//
// Overlay behavior:
// - If BOTH turn buttons held => Hazard flash amber (all strips)
// - If ONLY left held        => Amber sweep across cockpit strips (front+back)
// - If ONLY right held       => Amber sweep opposite direction across cockpit
//
// NOTE: These functions only set pixel buffers. Call ShowAllStrips() after.
// ============================================================================

// -------------------- BASE MODES --------------------
static void RenderNavLights() {
  // Bottom Left = Red
  for (uint16_t i = 0; i < stripBottomLeft.numPixels(); i++) {
    stripBottomLeft.setPixelColor(i, ColorRGBW(stripBottomLeft, 255, 0, 0, 0));
  }

  // Bottom Right = Green
  for (uint16_t i = 0; i < stripBottomRight.numPixels(); i++) {
    stripBottomRight.setPixelColor(i, ColorRGBW(stripBottomRight, 0, 255, 0, 0));
  }

  // Cockpit Front = White channel
  for (uint16_t i = 0; i < stripCockpitFront.numPixels(); i++) {
    stripCockpitFront.setPixelColor(i, ColorRGBW(stripCockpitFront, 0, 0, 0, 255));
  }

  // Cockpit Back = White channel
  for (uint16_t i = 0; i < stripCockpitBack.numPixels(); i++) {
    stripCockpitBack.setPixelColor(i, ColorRGBW(stripCockpitBack, 0, 0, 0, 255));
  }
}

static void RenderWhiteFlood() {
  // Pure white using W channel (natural white)
  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *s = LED_STRIPS[si];
    for (uint16_t i = 0; i < s->numPixels(); i++) {
      s->setPixelColor(i, ColorRGBW(*s, 0, 0, 0, 255));
    }
  }
}

static void RenderAmbientPulse(uint32_t nowMs) {
  if ((nowMs - g_lastPulseMs) < AMBIENT_PULSE_INTERVAL_MS) return;
  g_lastPulseMs = nowMs;

  g_pulsePhase += AMBIENT_PULSE_SPEED;
  if (g_pulsePhase > 6.2831853f) g_pulsePhase -= 6.2831853f;

  // sin wave 0..1
  const float s = (sinf(g_pulsePhase) + 1.0f) * 0.5f;
  const float wFloat = (float)AMBIENT_W_MIN + s * (float)(AMBIENT_W_MAX - AMBIENT_W_MIN);
  const uint8_t w = (uint8_t)(wFloat + 0.5f);

  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *st = LED_STRIPS[si];
    for (uint16_t i = 0; i < st->numPixels(); i++) {
      st->setPixelColor(i, ColorRGBW(*st, 0, 0, 0, w));
    }
  }
}

// Knight rider runs across cockpit strips (front + back) only.
// Bottom strips are dim nav references.
static void RenderKnightRider(uint32_t nowMs) {
  if ((nowMs - g_lastKnightMs) < KNIGHT_RIDER_INTERVAL_MS) return;
  g_lastKnightMs = nowMs;

  ClearAllStrips();

  // Dim ends so the bar still "reads" directionally
  for (uint16_t i = 0; i < stripBottomLeft.numPixels(); i++) {
    stripBottomLeft.setPixelColor(i, ColorRGBW(stripBottomLeft, 80, 0, 0, 0));
  }
  for (uint16_t i = 0; i < stripBottomRight.numPixels(); i++) {
    stripBottomRight.setPixelColor(i, ColorRGBW(stripBottomRight, 0, 80, 0, 0));
  }

  const uint16_t spanCF = stripCockpitFront.numPixels();
  const uint16_t spanCB = stripCockpitBack.numPixels();
  const uint32_t span   = (uint32_t)spanCF + (uint32_t)spanCB;
  if (span < 2) return;

  // Bounce within [0..span-1]
  if (g_knightPos == 0) g_knightDir = 1;
  if (g_knightPos >= (span - 1)) g_knightDir = -1;

  auto SetCockpitSpanPixel = [&](int32_t p, uint32_t col) {
    if (p < 0 || p >= (int32_t)span) return;
    if ((uint32_t)p < spanCF) {
      stripCockpitFront.setPixelColor((uint16_t)p, col);
    } else {
      stripCockpitBack.setPixelColor((uint16_t)((uint32_t)p - spanCF), col);
    }
  };

  // White comet with a small blue tint (looks crisp on RGBW)
  for (int8_t o = -3; o <= 3; o++) {
    const uint8_t amp = (uint8_t)(255 - (uint8_t)(abs(o) * 40));
    SetCockpitSpanPixel((int32_t)g_knightPos + o, ColorRGBW(stripCockpitFront, 0, 0, amp / 10, amp));
  }

  g_knightPos = (uint16_t)((int32_t)g_knightPos + (int32_t)g_knightDir);
}

// Render base mode based on g_mode.
// Note: Overlays are applied separately afterward.
static void RenderBaseMode(uint32_t nowMs) {
  switch (g_mode) {
    case MODE_OFF:
      ClearAllStrips();
      break;

    case MODE_NAV_LIGHTS:
      RenderNavLights();
      break;

    case MODE_WHITE_FLOOD:
      RenderWhiteFlood();
      break;

    case MODE_KNIGHT_RIDER:
      RenderKnightRider(nowMs);
      break;

    case MODE_AMBIENT_PULSE:
      RenderAmbientPulse(nowMs);
      break;

    default:
      ClearAllStrips();
      break;
  }
}

// -------------------- TURN / HAZARD OVERLAYS --------------------
// Hazard = both held: flash amber on all strips.
// Single direction held: animate amber sweep across cockpit span.

static void RenderHazardFlash(uint32_t nowMs) {
  // Simple 400ms toggle
  static bool on = false;
  static uint32_t lastToggleMs = 0;

  if ((nowMs - lastToggleMs) > 400) {
    lastToggleMs = nowMs;
    on = !on;
  }

  if (!on) {
    // Off phase: show nothing (could also show base mode faintly if desired)
    ClearAllStrips();
    return;
  }

  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *s = LED_STRIPS[si];
    for (uint16_t i = 0; i < s->numPixels(); i++) {
      // Amber in RGB channels (no W) to preserve "signal" look
      s->setPixelColor(i, ColorRGBW(*s, 255, 120, 0, 0));
    }
  }
}

static void RenderTurnSweep(uint32_t nowMs, bool leftDirection) {
  if ((nowMs - g_lastTurnMs) < TURN_SIGNAL_INTERVAL_MS) return;
  g_lastTurnMs = nowMs;
  g_turnFrame++;

  // Keep base nav under sweep (so it still reads like nav lights)
  // If you prefer the sweep to override everything, replace this with ClearAllStrips().
  RenderNavLights();

  const uint16_t spanCF = stripCockpitFront.numPixels();
  const uint16_t spanCB = stripCockpitBack.numPixels();
  const uint32_t span   = (uint32_t)spanCF + (uint32_t)spanCB;
  if (span < 1) return;

  // Head moves across cockpit span
  const uint32_t head = leftDirection ? (g_turnFrame % span)
                                      : ((span - 1) - (g_turnFrame % span));

  const uint8_t tail = 10;

  auto SetCockpitSpanPixel = [&](int32_t p, uint32_t col) {
    if (p < 0 || p >= (int32_t)span) return;
    if ((uint32_t)p < spanCF) {
      stripCockpitFront.setPixelColor((uint16_t)p, col);
    } else {
      stripCockpitBack.setPixelColor((uint16_t)((uint32_t)p - spanCF), col);
    }
  };

  for (uint8_t i = 0; i < tail; i++) {
    const int32_t p = (int32_t)head - (int32_t)i;
    const uint8_t amp = (uint8_t)(255 - (uint8_t)(i * (220 / tail)));
    SetCockpitSpanPixel(p, ColorRGBW(stripCockpitFront, amp, amp / 2, 0, 0)); // amber
  }
}

static void ApplyTurnHazardOverlay(uint32_t nowMs) {
  const bool hazard = (g_turnLeftHeld && g_turnRightHeld);

  if (hazard) {
    RenderHazardFlash(nowMs);
    return;
  }

  if (g_turnLeftHeld) {
    RenderTurnSweep(nowMs, true);
  } else if (g_turnRightHeld) {
    RenderTurnSweep(nowMs, false);
  }
}

// ============================================================================
// END OF RENDER FUNCTIONS
// ============================================================================






// ============================================================================
// STEPPER "JOG" UPDATE (HOLD-TO-MOVE WITH SOFT LIMITS)
// ============================================================================
//
// Goal:
//  - While PB3 (TURN LEFT) is held: rotate continuously left
//  - While PB4 (TURN RIGHT) is held: rotate continuously right
//  - Release: stop smoothly (decelerate) or stop immediately (configurable)
//  - Prevent motion beyond soft limits (0..MAX)
//  - Uses AccelStepper with acceleration for smooth starts/stops
//
// Notes:
//  - We treat stepperLeftRight position as "steps from full-left".
//  - g_lrPosSteps is kept in sync with stepperLeftRight.currentPosition().
//  - Soft limits are enforced using LR_STEP_LIMIT (max travel in steps).
//    For your 0°..180° idea, LR_STEP_LIMIT should correspond to 180° travel,
//    but can be tuned to your actual mechanics.
// ============================================================================

// Jog parameters (tune these):
static const float LR_JOG_SPEED_STEPS_PER_SEC = 600.0f;  // continuous speed while held
static const bool  LR_STOP_SMOOTH = true;                // true = decel to stop, false = hard stop

// Optional lift jog parameters (reserved; no buttons assigned yet)
static const float UD_JOG_SPEED_STEPS_PER_SEC = 600.0f;
static const bool  UD_STOP_SMOOTH = true;

// Helper: clamp long
static inline long ClampLong(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Call once in setup() after configuring max speed/accel
static void InitStepperState() {
  // Establish known starting positions (assume we start at "full-left" and "full-down")
  // If you start physically elsewhere, you can set these to match reality.
  stepperLeftRight.setCurrentPosition(0);
  stepperUpDown.setCurrentPosition(0);

  g_lrPosSteps = 0;
  g_udPosSteps = 0;

  // Ensure stopped initially
  stepperLeftRight.setSpeed(0.0f);
  stepperUpDown.setSpeed(0.0f);
}

// Decide commanded jog speeds based on held buttons + limits
static void UpdateJogCommands() {
  // Sync logical positions from AccelStepper
  g_lrPosSteps = stepperLeftRight.currentPosition();
  g_udPosSteps = stepperUpDown.currentPosition();

  // Enforce bounds in case something drifted
  g_lrPosSteps = ClampLong(g_lrPosSteps, 0, LR_STEP_LIMIT);
  g_udPosSteps = ClampLong(g_udPosSteps, 0, UD_STEP_LIMIT);

  // If clamped, force stepper positions to match clamp
  stepperLeftRight.setCurrentPosition(g_lrPosSteps);
  stepperUpDown.setCurrentPosition(g_udPosSteps);

  // ---------------- Left/Right Jog ----------------
  // PB3 held => left (negative direction) but don't go below 0
  // PB4 held => right (positive direction) but don't exceed LR_STEP_LIMIT

  if (g_turnLeftHeld && !g_turnRightHeld) {
    if (g_lrPosSteps > 0) {
      g_lrJogSpeed = -LR_JOG_SPEED_STEPS_PER_SEC;
    } else {
      g_lrJogSpeed = 0.0f; // at left limit
    }
  } else if (g_turnRightHeld && !g_turnLeftHeld) {
    if (g_lrPosSteps < LR_STEP_LIMIT) {
      g_lrJogSpeed = +LR_JOG_SPEED_STEPS_PER_SEC;
    } else {
      g_lrJogSpeed = 0.0f; // at right limit
    }
  } else {
    // neither held OR both held (hazard) => stop rotation
    g_lrJogSpeed = 0.0f;
  }

  // ---------------- Up/Down Jog (reserved) ----------------
  // No buttons assigned yet, keep stopped unless you later set g_liftUpHeld/g_liftDownHeld
  if (g_liftUpHeld && !g_liftDownHeld) {
    if (g_udPosSteps < UD_STEP_LIMIT) {
      g_udJogSpeed = +UD_JOG_SPEED_STEPS_PER_SEC;
    } else {
      g_udJogSpeed = 0.0f;
    }
  } else if (g_liftDownHeld && !g_liftUpHeld) {
    if (g_udPosSteps > 0) {
      g_udJogSpeed = -UD_JOG_SPEED_STEPS_PER_SEC;
    } else {
      g_udJogSpeed = 0.0f;
    }
  } else {
    g_udJogSpeed = 0.0f;
  }
}

// Apply the jog commands to AccelStepper
static void ApplyJogToSteppers() {
  // LEFT/RIGHT
  if (g_lrJogSpeed != 0.0f) {
    // Run indefinitely in the commanded direction using runSpeed().
    // We use setSpeed + runSpeed to get constant speed.
    stepperLeftRight.setSpeed(g_lrJogSpeed);
  } else {
    if (LR_STOP_SMOOTH) {
      // Smooth stop: command it to stop and let acceleration ramp down.
      stepperLeftRight.stop();
    } else {
      // Hard stop
      stepperLeftRight.setSpeed(0.0f);
    }
  }

  // UP/DOWN
  if (g_udJogSpeed != 0.0f) {
    stepperUpDown.setSpeed(g_udJogSpeed);
  } else {
    if (UD_STOP_SMOOTH) {
      stepperUpDown.stop();
    } else {
      stepperUpDown.setSpeed(0.0f);
    }
  }
}

// Call every loop()
static void UpdateSteppersJog() {
  // Determine desired speeds based on inputs + limits
  UpdateJogCommands();

  // Apply desired speeds/stops
  ApplyJogToSteppers();

  // Execute motion (non-blocking)
  // If we're using constant speed jogging, use runSpeed().
  // For smooth stop, AccelStepper uses its internal target; run() is safer.
  // We'll call BOTH: run() handles decel targets, runSpeed handles steady jog.
  //
  // Pattern:
  //  - When moving: runSpeed() advances at setSpeed
  //  - When stopping smoothly: stop() sets a target; run() decelerates to it
  //
  // This combination works reliably for jog + smooth stop.

  // Left/Right axis
  if (g_lrJogSpeed != 0.0f) {
    stepperLeftRight.runSpeed();
  } else {
    stepperLeftRight.run(); // allow smooth stop decel if enabled
  }

  // Up/Down axis
  if (g_udJogSpeed != 0.0f) {
    stepperUpDown.runSpeed();
  } else {
    stepperUpDown.run();
  }

  // Re-sync logical positions (so overlays/limits reflect actual motion)
  g_lrPosSteps = stepperLeftRight.currentPosition();
  g_udPosSteps = stepperUpDown.currentPosition();
}

// ============================================================================
// END OF STEPPER JOG UPDATE
// ============================================================================





// ============================================================================
// SETUP()
// ============================================================================
//
// One-time initialization only.
// NO runtime logic here.
//
// Responsibilities:
//  - Configure pin modes
//  - Initialize LED strips (safe/off state)
//  - Initialize buttons
//  - Initialize steppers (speed, accel, enable)
//  - Initialize I2C + IMU
//  - Establish known stepper reference state
// ============================================================================

void setup() {
  // -------------------- CORE --------------------
  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);   // allow power rails + level shifter to settle

  // -------------------- BUTTON INPUTS --------------------
  pinMode(PIN_PB_BRIGHTNESS, INPUT_PULLUP);
  pinMode(PIN_PB_MODE,       INPUT_PULLUP);
  pinMode(PIN_PB_TURN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_PB_TURN_RIGHT, INPUT_PULLUP);

  // -------------------- PWM OUTPUT --------------------
  pinMode(PIN_PWM_DATA, OUTPUT);
  SetAuxPwmDuty(0);          // ensure auxiliary output is off

  // -------------------- STEPPER DRIVER ENABLES --------------------
  // ~{EN} is active-LOW. SAFE POWER-ON: drivers stay DISABLED (HIGH) until
  // UART configuration has succeeded (Phase 2). Without UART config the
  // current is set by the module's VREF trimpot, not by firmware.
  // Write the level BEFORE switching to OUTPUT so there is no LOW glitch.
  digitalWrite(PIN_UPDOWN_EN, HIGH);
  digitalWrite(PIN_LEFTRIGHT_EN, HIGH);
  pinMode(PIN_UPDOWN_EN, OUTPUT);
  pinMode(PIN_LEFTRIGHT_EN, OUTPUT);

  // Configure stepper motion limits
  stepperUpDown.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperUpDown.setAcceleration(STEPPER_ACCEL);

  stepperLeftRight.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperLeftRight.setAcceleration(STEPPER_ACCEL);

  // Establish known starting positions (logical 0)
  InitStepperState();

  // -------------------- LED STRIPS --------------------
  stripBottomLeft.begin();
  stripBottomRight.begin();
  stripCockpitFront.begin();
  stripCockpitBack.begin();

  // Start dark (important for 12V systems)
  ClearAllStrips();
  SetBrightnessAll(BRIGHTNESS_DEFAULT);
  ShowAllStrips();

  // -------------------- IMU (MPU6050) --------------------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Optional sanity check
  if (!imu.begin(IMU_I2C_ADDR, &Wire)) {
    Serial.println("MPU6050 NOT DETECTED");
  } else {
    Serial.println("MPU6050 OK");
  }


  // -------------------- INITIAL STATE --------------------
  g_mode = MODE_NAV_LIGHTS;
  g_brightnessIndex = 0;
  ApplyBrightnessFromIndex();

  g_turnLeftHeld  = false;
  g_turnRightHeld = false;

  // Render initial mode
  RenderNavLights();
  ShowAllStrips();

  Serial.println("Setup complete");
}

// ============================================================================
// END OF SETUP()
// ============================================================================





// ============================================================================
// LOOP()
// ============================================================================
//
// Scheduler order (every cycle):
//  1) Read inputs (debounced + edges + held states)
//  2) Update "one-shot" state changes (brightness/mode changes)
//  3) Render LEDs: base mode then apply turn/hazard overlay
//  4) Update stepper jogging (hold-to-move with limits)
// ============================================================================

void loop() {
  const uint32_t nowMs = millis();

  // -------------------- 1) INPUTS --------------------
  UpdateInputs(nowMs);

  // -------------------- 2) ONE-SHOT STATE UPDATES --------------------
  // Brightness button (PB1) cycles discrete brightness levels
  if (g_brightnessPressedEdge) {
    g_brightnessIndex++;
    if (g_brightnessIndex >= BRIGHTNESS_LEVELS) {
      g_brightnessIndex = 0;
    }
    ApplyBrightnessFromIndex();
  }

  // Mode button (PB2) cycles lighting modes
  if (g_modePressedEdge) {
    g_mode = (LightingMode)((uint8_t)(g_mode + 1) % MODE_COUNT);

    // Reset animation timers/state for a clean transition
    g_lastKnightMs = 0;
    g_lastPulseMs  = 0;
    g_lastTurnMs   = 0;

    g_knightPos = 0;
    g_knightDir = 1;

    g_pulsePhase = 0.0f;
    g_turnFrame  = 0;
  }

  // -------------------- 3) LED RENDERING --------------------
  // Base mode first
  RenderBaseMode(nowMs);

  // Overlay turn/hazard effects when turn buttons are held
  // (This can override base visuals during hazard/turn indication)
  if (g_turnLeftHeld || g_turnRightHeld) {
    ApplyTurnHazardOverlay(nowMs);
  }

  // Push pixels out
  ShowAllStrips();

  // -------------------- 4) STEPPER UPDATE --------------------
  // Jog rotation while buttons are held + enforce soft limits
  UpdateSteppersJog();

  // Short cooperative delay to reduce CPU churn
  delay(5);
}

// ============================================================================
// END OF LOOP()
// ============================================================================
