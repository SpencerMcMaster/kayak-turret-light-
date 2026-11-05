/*
===============================================================================
  Project: Kayak Light Bar Controller
  Board:   Arduino Nano ESP32-S3

  Description:
    Multi-channel RGBW lighting and motion controller for a kayak-mounted
    light bar system.

    - 4x SK6812 RGBW (12V) addressable LED strips
      (driven via SN74HCT245 level shifter + 330Ω series resistors)
    - 2x TMC2209 stepper motor drivers (STEP/DIR motion, UART configuration)
      - Up/Down lift axis
      - Left/Right rotation axis
    - 4x momentary pushbuttons
      - Brightness            (click = next level, on release)
      - Mode select
      - Turn left             (hold = pan left)
      - Turn right            (hold = pan right)
      - Brightness + Left     (hold = lift DOWN)
      - Brightness + Right    (hold = lift UP)
    - GY-521 MPU6050 IMU for motion / orientation sensing
    - Light-bar PWM: 3x 18 W bars dimmed together via a MOSFET module
      (5 kHz LEDC, on only when the lift is at its UP soft limit)

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
#include <TMCStepper.h>                       // TMC2209 UART configuration

// Serial debug console ('?' for help). Set to 0 to compile it out.
#ifndef DEBUG_CONSOLE
#define DEBUG_CONSOLE 1
#endif



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

// Bytes per pixel, decoded from the NeoPixel type (W offset == R offset -> RGB)
static const uint8_t LED_BYTES_PER_PIXEL =
  (((LED_PIXEL_TYPE >> 6) & 0x03) == ((LED_PIXEL_TYPE >> 4) & 0x03)) ? 3 : 4;

// Global brightness limits (important for 12V current control)
static const uint8_t BRIGHTNESS_MIN     = 16;   // very dim but visible
static const uint8_t BRIGHTNESS_MAX     = 220;  // DO NOT run full 255 on 12V bench
static const uint8_t BRIGHTNESS_DEFAULT = 96;

// -------------------- LED POWER BUDGET (HARD SAFETY CONSTRAINT) --------------------
// (fraction of LEDs lit) x (applied brightness / 255) must stay <= 0.20,
// computed from the real pixel buffers every frame by EnforcePowerBudget().
// Over budget -> brightness is scaled down (pixels are never switched off).
// DO NOT raise, remove or bypass.
static const float LED_POWER_BUDGET_MAX = 0.20f;

// Number of discrete brightness steps for PB1
static const uint8_t BRIGHTNESS_LEVELS = 4;

// -------------------- LIGHT BARS (PWM CONTROL on J2.5) --------------------
// Single-channel MOSFET module, accepts 0-20 kHz PWM. LEDC hardware PWM
// (the LED strips use RMT, so no peripheral conflict).
static const uint8_t  LIGHTBAR_LEDC_CHANNEL = 0;
static const uint32_t LIGHTBAR_PWM_FREQ_HZ  = 5000;
static const uint8_t  LIGHTBAR_PWM_RES_BITS = 8;     // duty 0..255
// Light-bar duty per brightness index (same button as the strips)
static const uint8_t  LIGHTBAR_DUTY_BY_INDEX[] = { 64, 128, 191, 255 };
static_assert(sizeof(LIGHTBAR_DUTY_BY_INDEX) == BRIGHTNESS_LEVELS,
              "LIGHTBAR_DUTY_BY_INDEX needs exactly one entry per brightness level");

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
static const float STEPPER_MAX_SPEED = 2000.0f;   // absolute ceiling for any move
static const float STEPPER_ACCEL     = 4000.0f;

// Console 's' command: out-and-back test move per axis (steps, within soft limits)
static const long TEST_JOG_STEPS = 200;

// -------------------- TMC2209 (UART) --------------------
// HARD SAFETY CONSTRAINT: motors are 1 A rated. Run current <= 800 mA RMS,
// automatic standstill reduction to ~30 %. Never configure above this.
static constexpr uint16_t TMC_RUN_CURRENT_MA  = 800;
static constexpr float    TMC_HOLD_MULTIPLIER = 0.30f;   // IHOLD = 30 % of IRUN
static_assert(TMC_RUN_CURRENT_MA <= 800, "TMC run current must stay <= 800 mA (1 A motors)");
static_assert(TMC_HOLD_MULTIPLIER > 0.0f && TMC_HOLD_MULTIPLIER <= 0.5f, "IHOLD must be a reduction");

static const float    TMC_R_SENSE         = 0.11f;   // BTT V1.3 sense resistors R3/R5 = 0R11 (module schematic)
static const uint32_t TMC_UART_BAUD       = 115200;
static const uint16_t TMC_MICROSTEPS      = 16;
static const uint8_t  TMC_TOFF            = 3;       // chopper off time; 0 would disable the output stage
static const uint8_t  TMC_IHOLDDELAY      = 6;       // ramp IRUN -> IHOLD over 6 x 2^18 clocks
static const uint8_t  TMC_TPOWERDOWN      = 20;      // standstill -> IHOLD after ~0.44 s (20 x 2^18 / 12 MHz)

// StallGuard4 homing PLACEHOLDERS. 0 = disabled (DIAG only reports driver
// errors). Tune on the bench before enabling sensorless homing:
// DIAG goes high when SG_RESULT <= 2 x SGTHRS while TCOOLTHRS >= TSTEP.
static const uint32_t TMC_TCOOLTHRS       = 0;
static const uint8_t  TMC_SGTHRS          = 0;

// -------------------- IMU (MPU6050 via Adafruit_MPU6050) --------------------
// Library choice: Adafruit_MPU6050, not the jrowberg DMP. With no
// magnetometer the DMP's yaw is also pure gyro integration (same drift), so
// it adds nothing for pan tracking but costs a firmware blob upload at every
// boot and FIFO handling, and INT is not wired on this PCB.
// MOUNTING ASSUMPTION: GY-521 flat, +Z up, +X toward the bow. Change the
// axis mapping in ImuTask() if it is mounted otherwise.
static const uint16_t IMU_PERIOD_MS          = 10;     // 100 Hz
static const uint16_t IMU_CAL_SAMPLES        = 100;    // 1 s gyro-bias average at boot (keep still)
static const float    IMU_COMP_ALPHA         = 0.98f;  // complementary filter: gyro weight
static const uint32_t IMU_I2C_CLOCK_HZ       = 400000;
static const uint32_t IMU_TASK_STACK         = 4096;
static const UBaseType_t IMU_TASK_PRIO       = 2;      // above loopTask (1) on core 1

// -------------------- PAN TRACKING (SCAFFOLD, OFF BY DEFAULT) --------------------
// A 6-DOF IMU has NO true heading (no magnetometer). Tracking holds the
// light pointed at a fixed WORLD direction by integrating the yaw rate, which
// drifts (gyro bias). Re-zero with a MODE long-press. Bench-test before use.
static constexpr float PAN_TRAVEL_DEG             = 180.0f;  // LR_STEP_LIMIT spans this (<= 180)
static const float    PAN_STEPS_PER_DEG          = (float)LR_STEP_LIMIT / PAN_TRAVEL_DEG;
static const float    PAN_TRACK_SIGN             = 1.0f;    // flip to -1 if tracking turns the wrong way
static const long     PAN_TRACK_DEADBAND_STEPS   = 18;      // ~0.5 deg: no hunting on noise
static const uint16_t MODE_LONG_PRESS_MS         = 1000;    // MODE held this long = re-zero heading
static_assert(PAN_TRAVEL_DEG <= 180.0f, "pan travel must stay <= 180 degrees");

// -------------------- TASKS / SCHEDULING --------------------
// LED show() blocks ~24 ms for 600 RGBW pixels, so steppers run in their own
// task on core 0; loop() (core 1) only exchanges commands with it.
static const uint16_t FRAME_INTERVAL_MS   = 20;      // LED render + show cadence
static const uint32_t STEPPER_TASK_STACK  = 4096;
static const UBaseType_t STEPPER_TASK_PRIO = 1;
static const uint16_t STEPPER_TASK_YIELD_MS = 100;   // let IDLE0 run (task WDT) while moving

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

// -------------------- ONBOARD STATUS RGB LED (on the Nano module) --------------------
// Not on the PCB netlist. Active-LOW. GPIO46 (R), GPIO0 (G), GPIO45 (B) are
// strapping pins, sampled only at reset; driving them afterwards is fine.
static const uint8_t PIN_STATUS_LED_R      = LED_RED;    // GPIO46
static const uint8_t PIN_STATUS_LED_G      = LED_GREEN;  // GPIO0
static const uint8_t PIN_STATUS_LED_B      = LED_BLUE;   // GPIO45

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

static const uint16_t LED_TOTAL_PIXELS =
  LED_COUNT_BOTTOM_LEFT + LED_COUNT_BOTTOM_RIGHT +
  LED_COUNT_COCKPIT_FRONT + LED_COUNT_COCKPIT_BACK;

// Largest single strip, in bytes (scratch buffer for lossless brightness scaling)
static const uint16_t LED_MAX_STRIP_PIXELS =
  (LED_COUNT_BOTTOM_LEFT > LED_COUNT_BOTTOM_RIGHT ? LED_COUNT_BOTTOM_LEFT : LED_COUNT_BOTTOM_RIGHT);
static_assert(LED_MAX_STRIP_PIXELS >= LED_COUNT_COCKPIT_FRONT &&
              LED_MAX_STRIP_PIXELS >= LED_COUNT_COCKPIT_BACK,
              "LED_MAX_STRIP_PIXELS must cover the longest strip");

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
// NOTE: the AccelStepper objects are owned by the stepper task (core 0).
// Nothing else may call their methods once that task is running.

// -------------------- TMC2209 UART DRIVERS --------------------
// One shared single-wire UART bus (Serial1), addressed by MS1/MS2 tie-offs.
TMC2209Stepper driverUpDown   (&Serial1, TMC_R_SENSE, TMC_ADDR_UPDOWN);
TMC2209Stepper driverLeftRight(&Serial1, TMC_R_SENSE, TMC_ADDR_LEFTRIGHT);

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
// g_brightnessValue is the REQUESTED brightness. The library's own
// setBrightness() is never used (it is lossy: it rescales the pixel buffer in
// place). Pixel buffers hold full-scale colours; the applied brightness is
// decided by EnforcePowerBudget() and applied only on the way out in
// ShowAllStrips().
static uint8_t g_brightnessIndex = 0;      // 0..(BRIGHTNESS_LEVELS-1)
static uint8_t g_brightnessValue = BRIGHTNESS_DEFAULT;

// -------------------- POWER BUDGET STATE --------------------
struct PowerBudgetStats {
  uint16_t litPixels;    // pixels with any channel non-zero (all strips)
  uint16_t totalPixels;  // all pixels (all strips)
  float    litFraction;  // litPixels / totalPixels
  uint8_t  requested;    // g_brightnessValue at evaluation time
  uint8_t  applied;      // brightness actually sent to the strips
  float    product;      // litFraction * applied / 255  (must be <= 0.20)
  bool     limited;      // true if applied < requested
};
static PowerBudgetStats g_budget = { 0, LED_TOTAL_PIXELS, 0.0f, 0, 0, 0.0f, false };
static bool g_budgetFresh = false;  // set by EnforcePowerBudget, consumed by ShowAllStrips

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

// -------------------- TMC2209 STATUS --------------------
struct DriverStatus {
  bool    configOk;      // UART config written AND verified
  bool    enabled;       // ~{EN} currently driven LOW
  uint8_t connResult;    // test_connection(): 0 ok, 1 = read 0xFFFFFFFF, 2 = no reply (read timed out -> 0)
  uint8_t ifcntDelta;    // IFCNT increments seen during config
  uint8_t ifcntExpected; // writes issued during config
};
static DriverStatus g_drvUD = { false, false, 0xFF, 0, 0 };
static DriverStatus g_drvLR = { false, false, 0xFF, 0, 0 };

// -------------------- MOTION COMMANDS (loop -> stepper task) --------------------
// loop() (core 1) writes commands; the stepper task (core 0) owns the
// AccelStepper objects and publishes positions back. Guarded by g_motionMux.
enum AxisMode : uint8_t {
  AXIS_HOLD = 0,   // decelerate (or hard stop) and hold
  AXIS_JOG_POS,    // move toward the + soft limit (right / up)
  AXIS_JOG_NEG,    // move toward the - soft limit (left / down)
  AXIS_GOTO        // move to target (clamped to soft limits)
};
struct AxisCommand {
  AxisMode mode;
  long     target;   // AXIS_GOTO only
  bool     enabled;  // driver enabled; the task never steps a disabled axis
};
struct AxisFeedback {
  long position;
  long target;
  bool running;
};
static portMUX_TYPE  g_motionMux = portMUX_INITIALIZER_UNLOCKED;
static AxisCommand   g_cmdLR = { AXIS_HOLD, 0, false };
static AxisCommand   g_cmdUD = { AXIS_HOLD, 0, false };
static AxisFeedback  g_fbLR  = { 0, 0, false };
static AxisFeedback  g_fbUD  = { 0, 0, false };
static TaskHandle_t  g_stepperTask = nullptr;

// Per-axis engine memory (stepper task only). Declared here, ahead of the
// Arduino builder's auto-generated prototypes, because ServiceAxis() takes it.
struct AxisEngineState {
  AxisMode lastMode;
};

// Console 's' test move: 0 = idle, 1 = going out, 2 = coming back
static uint8_t g_testJogPhase = 0;
static long    g_testJogOriginLR = 0;
static long    g_testJogOriginUD = 0;

// -------------------- LIGHT-BAR PWM --------------------
static uint8_t g_lightBarDuty = 0;   // last duty written to PIN_PWM_DATA

// -------------------- IMU ESTIMATE (IMU task -> loop) --------------------
struct ImuEstimate {
  bool     present;      // imu.begin() succeeded
  bool     calibrated;   // gyro bias measured
  float    rollDeg;      // complementary filter
  float    pitchDeg;     // complementary filter
  float    yawDeg;       // integrated yaw since boot (drifts; NOT a compass heading)
  float    yawRateDps;   // tilt-compensated yaw rate
  float    ax, ay, az;   // m/s^2
  float    gx, gy, gz;   // rad/s, bias removed
  float    tempC;
  uint32_t samples;
  uint32_t readErrors;
};
static portMUX_TYPE g_imuMux = portMUX_INITIALIZER_UNLOCKED;
static ImuEstimate  g_imu = { false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float        g_yawZeroDeg = 0.0f;  // loop-side offset; heading = yawDeg - g_yawZeroDeg

// -------------------- PAN TRACKING STATE --------------------
enum PanMode : uint8_t { PAN_MODE_MANUAL = 0, PAN_MODE_TRACK };
static bool    g_panTrackEnabled = false;   // OFF until bench-tested ('t' toggles)
static PanMode g_panMode         = PAN_MODE_MANUAL;
static bool    g_trackNeedAnchor = true;    // re-anchor after enable / manual move / re-zero
static long    g_trackAnchorPos  = 0;
static float   g_trackAnchorYaw  = 0.0f;
static long    g_trackTarget     = 0;

// -------------------- MODE BUTTON --------------------
static bool g_modeLongPressEdge = false;    // one-shot: MODE held >= MODE_LONG_PRESS_MS
static bool g_modeLongFired     = false;    // long press already fired during this hold

// -------------------- FRAME SCHEDULER --------------------
static uint32_t g_lastFrameMs = 0;

// -------------------- BUTTON EDGE STATE (DEBOUNCE) --------------------
// We will implement a small debounce/edge detector struct in the next block.
// These placeholders exist so the "state section" documents what we track.
static bool g_brightnessPressedEdge = false;   // brightness CLICK (see UpdateInputs)
static bool g_modePressedEdge       = false;

// -------------------- LIFT CHORD STATE --------------------
static bool g_liftChordActive     = false;  // BRIGHTNESS held + LEFT/RIGHT held
static bool g_brightnessChordUsed = false;  // a chord happened during this BRIGHTNESS hold
static bool g_panLockout          = false;  // LEFT/RIGHT must be released after a chord

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
  bool releasedEdge;     // one-shot on release (LOW->HIGH)
  uint32_t pressedAtMs;  // time of the last debounced press
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

// -------------------- LED POWER BUDGET --------------------
// HARD SAFETY CONSTRAINT. Walks every pixel of every strip, counts pixels
// with any non-zero channel, and limits the applied brightness so that
//   litFraction * (applied / 255) <= LED_POWER_BUDGET_MAX.
// Brightness is scaled down; pixels are never turned off. Must be called
// immediately before ShowAllStrips() every frame (ShowAllStrips enforces it
// anyway if a caller forgets).
static void EnforcePowerBudget() {
  uint32_t lit = 0;
  uint32_t total = 0;

  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *s = LED_STRIPS[si];
    const uint8_t *px = s->getPixels();
    const uint16_t n = s->numPixels();
    total += n;
    if (px == nullptr) continue;  // begin()/alloc failed: nothing can be shown
    for (uint16_t i = 0; i < n; i++) {
      const uint8_t *p = px + (uint32_t)i * LED_BYTES_PER_PIXEL;
      uint8_t any = 0;
      for (uint8_t c = 0; c < LED_BYTES_PER_PIXEL; c++) any |= p[c];
      if (any) lit++;
    }
  }

  const float litFraction = (total > 0) ? ((float)lit / (float)total) : 0.0f;
  const uint8_t requested = g_brightnessValue;
  uint8_t applied = requested;

  if (litFraction > 0.0f) {
    const float maxB = (LED_POWER_BUDGET_MAX * 255.0f) / litFraction;
    if ((float)applied > maxB) applied = (uint8_t)maxB;   // floor
    // Guard against float rounding: step down until it provably passes.
    while (applied > 0 &&
           (litFraction * ((float)applied / 255.0f)) > LED_POWER_BUDGET_MAX) {
      applied--;
    }
  }

  g_budget.litPixels   = (uint16_t)lit;
  g_budget.totalPixels = (uint16_t)total;
  g_budget.litFraction = litFraction;
  g_budget.requested   = requested;
  g_budget.applied     = applied;
  g_budget.product     = litFraction * ((float)applied / 255.0f);
  g_budget.limited     = (applied < requested);
  g_budgetFresh = true;
}

// Scratch copy so brightness scaling is lossless: the render buffers always
// keep full-scale colours, the scaled copy only exists while show() runs.
static uint8_t g_showScratch[(uint32_t)LED_MAX_STRIP_PIXELS * 4];

static void ShowAllStrips() {
  // Defence in depth: nothing reaches the strips without a budget check
  // computed from the current buffers.
  if (!g_budgetFresh) EnforcePowerBudget();

  const uint16_t scale = (uint16_t)g_budget.applied + 1;  // 1..256, same math as the library

  for (uint8_t si = 0; si < LED_STRIP_COUNT; si++) {
    Adafruit_NeoPixel *s = LED_STRIPS[si];
    uint8_t *px = s->getPixels();
    if (px == nullptr) continue;
    const uint32_t nBytes = (uint32_t)s->numPixels() * LED_BYTES_PER_PIXEL;
    if (nBytes > sizeof(g_showScratch)) continue;  // cannot happen (static_assert above)

    memcpy(g_showScratch, px, nBytes);
    for (uint32_t i = 0; i < nBytes; i++) {
      px[i] = (uint8_t)(((uint16_t)px[i] * scale) >> 8);
    }
    s->show();                       // blocking: RMT transmit completes before return
    memcpy(px, g_showScratch, nBytes);
  }

  g_budgetFresh = false;
}

// Sets the REQUESTED brightness for all strips. The applied value is decided
// per frame by EnforcePowerBudget().
static void SetBrightnessAll(uint8_t b) {
  g_brightnessValue = b;
}

static uint8_t BrightnessFromIndex(uint8_t idx) {
  if (BRIGHTNESS_LEVELS <= 1) return BRIGHTNESS_DEFAULT;
  if (idx >= (BRIGHTNESS_LEVELS - 1)) return BRIGHTNESS_MAX;

  const float t = (float)idx / (float)(BRIGHTNESS_LEVELS - 1);
  const float v = (float)BRIGHTNESS_MIN + t * (float)(BRIGHTNESS_MAX - BRIGHTNESS_MIN);
  return (uint8_t)(v + 0.5f);
}

static void ApplyBrightnessFromIndex() {
  SetBrightnessAll(BrightnessFromIndex(g_brightnessIndex));
}

// -------------------- PWM HELPER (LIGHT BARS) --------------------
// LEDC hardware PWM, 5 kHz, 8-bit. Call LightBarPwmInit() once first.
static void LightBarPwmInit() {
  digitalWrite(PIN_PWM_DATA, LOW);              // hold the MOSFET input low...
  pinMode(PIN_PWM_DATA, OUTPUT);
  ledcSetup(LIGHTBAR_LEDC_CHANNEL, LIGHTBAR_PWM_FREQ_HZ, LIGHTBAR_PWM_RES_BITS);
  ledcWrite(LIGHTBAR_LEDC_CHANNEL, 0);          // ...and set duty 0 before attaching
  ledcAttachPin(PIN_PWM_DATA, LIGHTBAR_LEDC_CHANNEL);
  g_lightBarDuty = 0;
}

static void SetAuxPwmDuty(uint8_t duty /*0..255*/) {
  g_lightBarDuty = duty;
  ledcWrite(LIGHTBAR_LEDC_CHANNEL, duty);
}

// Light bars follow the brightness index, but are ONLY on when the lift is
// at its UP soft limit (stopped, driver enabled so the position is trusted).
// ASSUMPTION: MODE_OFF also turns the light bars off.
// A console 'p <n>' override (bench use) bypasses the lift gate until 'p auto'.
static int16_t g_lightBarOverride = -1;   // -1 = automatic

static bool LiftAtUpLimit() {
  AxisFeedback lr, ud;
  ReadMotionFeedback(lr, ud);
  return g_drvUD.enabled && !ud.running && (ud.position >= UD_STEP_LIMIT);
}

static void UpdateLightBars() {
  uint8_t duty = 0;
  if (g_lightBarOverride >= 0) {
    duty = (uint8_t)g_lightBarOverride;
  } else if (g_mode != MODE_OFF && LiftAtUpLimit()) {
    duty = LIGHTBAR_DUTY_BY_INDEX[g_brightnessIndex];
  }
  if (duty != g_lightBarDuty) SetAuxPwmDuty(duty);
}

// -------------------- STATUS LED (onboard RGB, active-LOW) --------------------
static void StatusLedInit() {
  digitalWrite(PIN_STATUS_LED_R, HIGH);
  digitalWrite(PIN_STATUS_LED_G, HIGH);
  digitalWrite(PIN_STATUS_LED_B, HIGH);
  pinMode(PIN_STATUS_LED_R, OUTPUT);
  pinMode(PIN_STATUS_LED_G, OUTPUT);
  pinMode(PIN_STATUS_LED_B, OUTPUT);
}

static void StatusLed(bool r, bool g, bool b) {
  digitalWrite(PIN_STATUS_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_STATUS_LED_B, b ? LOW : HIGH);
}

// -------------------- DRIVER ENABLE --------------------
// Only a driver whose UART config was verified may be enabled.
static bool SetDriverEnabled(DriverStatus &st, uint8_t enPin, bool enable) {
  if (enable && !st.configOk) return false;
  digitalWrite(enPin, enable ? LOW : HIGH);   // ~{EN}: LOW = enabled
  st.enabled = enable;
  portENTER_CRITICAL(&g_motionMux);
  if (&st == &g_drvLR) g_cmdLR.enabled = enable;
  else                 g_cmdUD.enabled = enable;
  portEXIT_CRITICAL(&g_motionMux);
  return true;
}

// -------------------- MOTION FEEDBACK --------------------
static void ReadMotionFeedback(AxisFeedback &lr, AxisFeedback &ud) {
  portENTER_CRITICAL(&g_motionMux);
  lr = g_fbLR;
  ud = g_fbUD;
  portEXIT_CRITICAL(&g_motionMux);
}

// -------------------- BUTTON INSTANCES --------------------
static DebouncedButton btnBrightness = { PIN_PB_BRIGHTNESS, HIGH, HIGH, 0, false, false, 0 };
static DebouncedButton btnMode       = { PIN_PB_MODE,       HIGH, HIGH, 0, false, false, 0 };
static DebouncedButton btnTurnLeft   = { PIN_PB_TURN_LEFT,  HIGH, HIGH, 0, false, false, 0 };
static DebouncedButton btnTurnRight  = { PIN_PB_TURN_RIGHT, HIGH, HIGH, 0, false, false, 0 };

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
      if (raw == LOW) {
        b.pressedEdge = true;               // active-low press edge
        b.pressedAtMs = nowMs;
      } else {
        b.releasedEdge = true;              // release edge
      }
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

static bool ConsumeReleasedEdge(DebouncedButton &b) {
  if (b.releasedEdge) {
    b.releasedEdge = false;
    return true;
  }
  return false;
}

static bool IsHeld(const DebouncedButton &b) {
  return (b.lastStable == LOW);
}

// Button map (J6, no extra hardware):
//   BRIGHTNESS click (press+release, no chord)  -> next brightness level
//   MODE click (released before 1 s)            -> next lighting mode
//   MODE held >= MODE_LONG_PRESS_MS             -> re-zero tracking heading
//   LEFT / RIGHT held                           -> pan jog (+ turn overlay)
//   BRIGHTNESS held + LEFT held                 -> lift DOWN
//   BRIGHTNESS held + RIGHT held                -> lift UP
// While BRIGHTNESS is held, LEFT/RIGHT belong to the lift chord: pan jog and
// the turn overlay are suppressed, and the brightness click is cancelled.
// A LEFT/RIGHT that was part of a chord stays locked out of pan jog until
// it is released, so letting go of BRIGHTNESS first cannot start a pan move.
static void UpdateInputs(uint32_t nowMs) {
  UpdateButton(btnBrightness, nowMs);
  UpdateButton(btnMode,       nowMs);
  UpdateButton(btnTurnLeft,   nowMs);
  UpdateButton(btnTurnRight,  nowMs);

  const bool brightPressed  = ConsumePressedEdge(btnBrightness);
  const bool brightReleased = ConsumeReleasedEdge(btnBrightness);

  // ---- MODE: click on release (< long press), long press fires once while held ----
  const bool modePressed  = ConsumePressedEdge(btnMode);
  const bool modeReleased = ConsumeReleasedEdge(btnMode);
  if (modePressed) g_modeLongFired = false;
  g_modeLongPressEdge = false;
  if (IsHeld(btnMode) && !g_modeLongFired &&
      (nowMs - btnMode.pressedAtMs) >= MODE_LONG_PRESS_MS) {
    g_modeLongFired = true;
    g_modeLongPressEdge = true;
  }
  g_modePressedEdge = modeReleased && !g_modeLongFired;

  const bool brightHeld = IsHeld(btnBrightness);
  const bool leftHeld   = IsHeld(btnTurnLeft);
  const bool rightHeld  = IsHeld(btnTurnRight);

  // ---- Lift chord ----
  g_liftChordActive = brightHeld && (leftHeld || rightHeld);
  g_liftDownHeld    = brightHeld && leftHeld && !rightHeld;
  g_liftUpHeld      = brightHeld && rightHeld && !leftHeld;

  // ---- Brightness click: fires on RELEASE, only if no chord happened ----
  if (brightPressed) g_brightnessChordUsed = false;
  if (g_liftChordActive) g_brightnessChordUsed = true;
  g_brightnessPressedEdge = brightReleased && !g_brightnessChordUsed;
  if (brightReleased) g_brightnessChordUsed = false;

  // ---- Pan jog / turn overlay (suppressed by the chord) ----
  if (g_liftChordActive) g_panLockout = true;
  if (!leftHeld && !rightHeld) g_panLockout = false;
  const bool panAllowed = !brightHeld && !g_panLockout;
  g_turnLeftHeld  = panAllowed && leftHeld;
  g_turnRightHeld = panAllowed && rightHeld;
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
  // Advance the animation on its own interval, but draw on EVERY frame: the
  // base mode is redrawn underneath each frame and would otherwise erase the
  // sweep between animation steps (visible flicker).
  if ((nowMs - g_lastTurnMs) >= TURN_SIGNAL_INTERVAL_MS) {
    g_lastTurnMs = nowMs;
    g_turnFrame++;
  }

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
// IMU (MPU6050) READ + COMPLEMENTARY FILTER
// ============================================================================
//
// ImuTask() runs at 100 Hz on core 1 (priority above loop) and is the ONLY
// user of Wire/imu after setup(). It publishes an ImuEstimate under
// g_imuMux; everything else reads it with ReadImu().
//
//  - Gyro bias: mean of the first IMU_CAL_SAMPLES samples (keep still ~1 s).
//  - Roll/pitch: complementary filter (gyro integration + accel gravity).
//  - Yaw: integral of the tilt-compensated yaw rate
//        yawRate = (gy*sin(roll) + gz*cos(roll)) / cos(pitch)
//    This is a RELATIVE heading that drifts with residual gyro bias. A 6-DOF
//    IMU cannot give true heading. Re-zero with a MODE long-press.
// ============================================================================

static const float RAD2DEG = 57.2957795f;

static void ReadImu(ImuEstimate &out) {
  portENTER_CRITICAL(&g_imuMux);
  out = g_imu;
  portEXIT_CRITICAL(&g_imuMux);
}

static float ImuHeadingDeg(const ImuEstimate &e) {
  return e.yawDeg - g_yawZeroDeg;
}

static void ImuTask(void * /*arg*/) {
  float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
  uint16_t calCount = 0;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  bool seeded = false;
  uint32_t lastUs = micros();
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));

    sensors_event_t a, g, t;
    const bool ok = imu.getEvent(&a, &g, &t);
    const uint32_t nowUs = micros();
    float dt = (float)(nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    if (!ok) {
      portENTER_CRITICAL(&g_imuMux);
      g_imu.readErrors++;
      portEXIT_CRITICAL(&g_imuMux);
      continue;
    }
    if (dt <= 0.0f || dt > 0.1f) dt = (float)IMU_PERIOD_MS * 1e-3f;

    // ---- Gyro bias calibration (first second) ----
    if (calCount < IMU_CAL_SAMPLES) {
      biasX += g.gyro.x;  biasY += g.gyro.y;  biasZ += g.gyro.z;
      calCount++;
      if (calCount == IMU_CAL_SAMPLES) {
        biasX /= (float)IMU_CAL_SAMPLES;
        biasY /= (float)IMU_CAL_SAMPLES;
        biasZ /= (float)IMU_CAL_SAMPLES;
      }
      continue;
    }

    const float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
    const float gx = g.gyro.x - biasX, gy = g.gyro.y - biasY, gz = g.gyro.z - biasZ;

    // ---- Roll / pitch: complementary filter ----
    const float rollAcc  = atan2f(ay, az) * RAD2DEG;
    const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    if (!seeded) {
      roll = rollAcc;
      pitch = pitchAcc;
      seeded = true;
    }
    roll  = IMU_COMP_ALPHA * (roll  + gx * dt * RAD2DEG) + (1.0f - IMU_COMP_ALPHA) * rollAcc;
    pitch = IMU_COMP_ALPHA * (pitch + gy * dt * RAD2DEG) + (1.0f - IMU_COMP_ALPHA) * pitchAcc;

    // ---- Yaw: tilt-compensated rate, integrated (relative, drifts) ----
    const float rollRad = roll / RAD2DEG;
    float cosPitch = cosf(pitch / RAD2DEG);
    if (cosPitch < 0.2f) cosPitch = 0.2f;   // avoid blow-up near +/-90 deg pitch
    const float yawRate = (gy * sinf(rollRad) + gz * cosf(rollRad)) / cosPitch;  // rad/s
    yaw += yawRate * dt * RAD2DEG;

    portENTER_CRITICAL(&g_imuMux);
    g_imu.calibrated = true;
    g_imu.rollDeg    = roll;
    g_imu.pitchDeg   = pitch;
    g_imu.yawDeg     = yaw;
    g_imu.yawRateDps = yawRate * RAD2DEG;
    g_imu.ax = ax;  g_imu.ay = ay;  g_imu.az = az;
    g_imu.gx = gx;  g_imu.gy = gy;  g_imu.gz = gz;
    g_imu.tempC = t.temperature;
    g_imu.samples++;
    portEXIT_CRITICAL(&g_imuMux);
  }
}

static void StartImuTask() {
  xTaskCreatePinnedToCore(ImuTask, "imu", IMU_TASK_STACK, nullptr,
                          IMU_TASK_PRIO, nullptr, 1 /* core 1 */);
}

// ============================================================================
// END OF IMU
// ============================================================================




// ============================================================================
// TMC2209 UART CONFIGURATION
// ============================================================================
//
// Both drivers share Serial1 (single-wire bus, addresses from MS1/MS2).
// A driver is only ever enabled (~{EN} LOW) after its configuration has been
// written AND verified:
//   - IFCNT (the chip's successful-write counter) advanced by exactly the
//     number of writes issued, and
//   - read-back of the RW registers GCONF / CHOPCONF / PWMCONF matches.
//
// TMCStepper keeps a zeroed shadow copy of every register and writes the
// whole register from it, so a bare microsteps(16) would also write
// TOFF = 0 (output stage OFF) and a bare pwm_autoscale(true) would zero
// PWM_OFS/PWM_GRAD. The chip's RW registers are therefore read first and
// written back so the shadows start from the chip's real values.
// ============================================================================

static const char *ConnResultText(uint8_t r) {
  switch (r) {
    case 0:  return "OK";
    case 1:  return "read 0xFFFFFFFF (line stuck HIGH?)";
    case 2:  return "NO REPLY (timeout/CRC error -> read 0)";
    default: return "NOT TESTED";
  }
}

static bool ConfigureDriver(TMC2209Stepper &d, DriverStatus &st, const char *name) {
  st.configOk = false;
  st.ifcntDelta = 0;
  st.ifcntExpected = 0;

  st.connResult = d.test_connection();
  if (st.connResult != 0) {
    Serial.printf("[TMC] %s: test_connection = %u (%s)\n", name, st.connResult, ConnResultText(st.connResult));
    return false;
  }

  // Seed TMCStepper's shadow registers with the chip's real values.
  const uint32_t gconf = d.GCONF();
  const uint32_t chop  = d.CHOPCONF();
  const uint32_t pwm   = d.PWMCONF();
  if (chop == 0 || pwm == 0) {   // reset values are non-zero; 0 means a failed read
    Serial.printf("[TMC] %s: register read failed (CHOPCONF=0x%08lX PWMCONF=0x%08lX)\n",
                  name, (unsigned long)chop, (unsigned long)pwm);
    return false;
  }

  const uint8_t ifcntBefore = d.IFCNT();
  uint8_t writes = 0;

  d.GCONF(gconf);                          writes++;
  d.CHOPCONF(chop);                        writes++;
  d.PWMCONF(pwm);                          writes++;

  d.pdn_disable(true);                     writes++;  // PDN_UART pin is the UART, not power-down
  d.mstep_reg_select(true);                writes++;  // microsteps from MRES (MS1/MS2 are the address)
  d.I_scale_analog(false);                 writes++;  // internal reference: VREF trimpot does NOT scale current
  d.internal_Rsense(false);                writes++;  // external 0R11 sense resistors
  d.multistep_filt(true);                  writes++;
  d.en_spreadCycle(false);                 writes++;  // StealthChop

  d.toff(TMC_TOFF);                        writes++;
  d.intpol(true);                          writes++;  // interpolate to 256 microsteps
  d.microsteps(TMC_MICROSTEPS);            writes++;

  d.pwm_autoscale(true);                   writes++;
  d.pwm_autograd(true);                    writes++;

  d.rms_current(TMC_RUN_CURRENT_MA, TMC_HOLD_MULTIPLIER); writes += 3;  // vsense + IRUN + IHOLD
  d.iholddelay(TMC_IHOLDDELAY);            writes++;
  d.TPOWERDOWN(TMC_TPOWERDOWN);            writes++;  // automatic standstill reduction

  d.TCOOLTHRS(TMC_TCOOLTHRS);              writes++;  // StallGuard placeholders (disabled)
  d.SGTHRS(TMC_SGTHRS);                    writes++;

  const uint8_t ifcntAfter = d.IFCNT();
  st.ifcntDelta = (uint8_t)(ifcntAfter - ifcntBefore);
  st.ifcntExpected = writes;

  const bool gconfOk = d.pdn_disable() && d.mstep_reg_select() &&
                       !d.I_scale_analog() && !d.en_spreadCycle();
  const bool chopOk  = (d.microsteps() == TMC_MICROSTEPS) && (d.toff() == TMC_TOFF);
  const bool pwmOk   = d.pwm_autoscale();
  const bool ifcntOk = (st.ifcntDelta == st.ifcntExpected);

  st.configOk = gconfOk && chopOk && pwmOk && ifcntOk;

  Serial.printf("[TMC] %s: writes %u/%u acked, GCONF %s, CHOPCONF %s, PWMCONF %s -> %s "
                "(IRUN set %u mA RMS, IHOLD %.0f%%)\n",
                name, st.ifcntDelta, st.ifcntExpected,
                gconfOk ? "ok" : "BAD", chopOk ? "ok" : "BAD", pwmOk ? "ok" : "BAD",
                st.configOk ? "CONFIG OK" : "CONFIG FAILED",
                d.rms_current(), (double)(TMC_HOLD_MULTIPLIER * 100.0f));
  return st.configOk;
}

// ============================================================================
// END OF TMC2209 UART CONFIGURATION
// ============================================================================




// ============================================================================
// STEPPER "JOG" UPDATE (HOLD-TO-MOVE WITH SOFT LIMITS)
// ============================================================================
//
// Goal:
//  - While PB3 (TURN LEFT) is held: rotate continuously left
//  - While PB4 (TURN RIGHT) is held: rotate continuously right
//  - Release: stop smoothly (decelerate) or stop immediately (configurable)
//  - Prevent motion beyond soft limits (0..MAX) -- ENFORCED HERE, in the engine
//  - Uses AccelStepper with acceleration for smooth starts/stops
//
// Architecture (why there is a task):
//  - ShowAllStrips() blocks ~24 ms per frame (600 RGBW pixels over RMT), so
//    AccelStepper polled from loop() would be capped at ~35 steps/s.
//  - StepperTask() runs on core 0 and is the ONLY code that touches the
//    AccelStepper objects. loop() (core 1) sends AxisCommands and reads
//    AxisFeedback through g_motionMux.
//
// Soft limits:
//  - Positions are steps from full-left (pan) / full-down (lift); the turret
//    must be at full-left / full-down at power-on (no homing yet -- see the
//    StallGuard placeholders in COMPILE-TIME CONFIGURATION).
//  - Every target the engine ever uses is clamped to [0, LIMIT]: jog = move
//    toward the limit, release = decelerate (stop target clamped too).
//  - LR_STEP_LIMIT must correspond to <= 180 degrees of pan travel.
//  - The engine never steps an axis whose driver is disabled, so the step
//    count cannot drift away from the real motor position.
// ============================================================================

// Jog parameters (tune these):
static const float LR_JOG_SPEED_STEPS_PER_SEC = 600.0f;  // continuous speed while held
static const bool  LR_STOP_SMOOTH = true;                // true = decel to stop, false = hard stop

// Lift jog parameters
static const float UD_JOG_SPEED_STEPS_PER_SEC = 600.0f;
static const bool  UD_STOP_SMOOTH = true;

// Helper: clamp long
static inline long ClampLong(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Call once in setup() BEFORE the stepper task starts
static void InitStepperState() {
  // Establish known starting positions (assume we start at "full-left" and "full-down")
  // If you start physically elsewhere, you can set these to match reality.
  stepperLeftRight.setCurrentPosition(0);
  stepperUpDown.setCurrentPosition(0);

  g_lrPosSteps = 0;
  g_udPosSteps = 0;

  g_fbLR = { 0, 0, false };
  g_fbUD = { 0, 0, false };
}

// -------------------- ENGINE (stepper task, core 0) --------------------

static void ServiceAxis(AccelStepper &s, AxisEngineState &es, const AxisCommand &c,
                        long limitHi, bool stopSmooth) {
  const long lo = 0;
  const long hi = limitHi;

  if (!c.enabled) {
    // Driver disabled: never issue steps. Hard-stop the profile in place.
    if (s.distanceToGo() != 0 || s.speed() != 0.0f) {
      s.setCurrentPosition(s.currentPosition());   // also zeroes speed
    }
    es.lastMode = AXIS_HOLD;
    return;
  }

  const long pos = s.currentPosition();

  switch (c.mode) {
    case AXIS_JOG_POS:
      if (s.targetPosition() != hi) s.moveTo(hi);
      break;

    case AXIS_JOG_NEG:
      if (s.targetPosition() != lo) s.moveTo(lo);
      break;

    case AXIS_GOTO: {
      const long t = ClampLong(c.target, lo, hi);
      if (s.targetPosition() != t) s.moveTo(t);
      break;
    }

    case AXIS_HOLD:
    default:
      if (es.lastMode != AXIS_HOLD) {
        if (stopSmooth) {
          s.stop();   // decelerate: sets a target just far enough to stop
        } else {
          s.setCurrentPosition(pos);   // hard stop
        }
      }
      break;
  }

  // Clamp whatever target is now set (stop() may add one step past a limit).
  const long t = s.targetPosition();
  const long tc = ClampLong(t, lo, hi);
  if (t != tc) s.moveTo(tc);

  es.lastMode = c.mode;
  s.run();
}

static void StepperTask(void * /*arg*/) {
  AxisEngineState esLR = { AXIS_HOLD };
  AxisEngineState esUD = { AXIS_HOLD };
  uint32_t lastYieldMs = millis();

  for (;;) {
    AxisCommand lr, ud;
    portENTER_CRITICAL(&g_motionMux);
    lr = g_cmdLR;
    ud = g_cmdUD;
    portEXIT_CRITICAL(&g_motionMux);

    ServiceAxis(stepperLeftRight, esLR, lr, LR_STEP_LIMIT, LR_STOP_SMOOTH);
    ServiceAxis(stepperUpDown,    esUD, ud, UD_STEP_LIMIT, UD_STOP_SMOOTH);

    const AxisFeedback fbLR = { stepperLeftRight.currentPosition(),
                                stepperLeftRight.targetPosition(),
                                stepperLeftRight.isRunning() };
    const AxisFeedback fbUD = { stepperUpDown.currentPosition(),
                                stepperUpDown.targetPosition(),
                                stepperUpDown.isRunning() };
    portENTER_CRITICAL(&g_motionMux);
    g_fbLR = fbLR;
    g_fbUD = fbUD;
    portEXIT_CRITICAL(&g_motionMux);

    const uint32_t nowMs = millis();
    if (!fbLR.running && !fbUD.running) {
      vTaskDelay(1);                       // idle: poll at 1 kHz
      lastYieldMs = nowMs;
    } else if ((nowMs - lastYieldMs) >= STEPPER_TASK_YIELD_MS) {
      vTaskDelay(1);                       // let IDLE0 feed the task watchdog
      lastYieldMs = millis();
    }
  }
}

static void StartStepperTask() {
  stepperLeftRight.setMaxSpeed(min(LR_JOG_SPEED_STEPS_PER_SEC, STEPPER_MAX_SPEED));
  stepperLeftRight.setAcceleration(STEPPER_ACCEL);
  stepperUpDown.setMaxSpeed(min(UD_JOG_SPEED_STEPS_PER_SEC, STEPPER_MAX_SPEED));
  stepperUpDown.setAcceleration(STEPPER_ACCEL);

  xTaskCreatePinnedToCore(StepperTask, "steppers", STEPPER_TASK_STACK, nullptr,
                          STEPPER_TASK_PRIO, &g_stepperTask, 0 /* core 0 */);
}

// -------------------- COMMANDS (loop, core 1) --------------------
static void SetAxisCommands(AxisMode lrMode, long lrTarget, AxisMode udMode, long udTarget) {
  portENTER_CRITICAL(&g_motionMux);
  g_cmdLR.mode = lrMode;  g_cmdLR.target = lrTarget;
  g_cmdUD.mode = udMode;  g_cmdUD.target = udTarget;
  portEXIT_CRITICAL(&g_motionMux);
}

// Console 's': out-and-back move of TEST_JOG_STEPS on each axis, inside the
// soft limits. Any jog button cancels it.
#if DEBUG_CONSOLE
static void StartTestJog() {
  AxisFeedback lr, ud;
  ReadMotionFeedback(lr, ud);
  g_testJogOriginLR = lr.position;
  g_testJogOriginUD = ud.position;
  g_testJogPhase = 1;
}
#endif

static void TestJogTargets(const AxisFeedback &lr, const AxisFeedback &ud, long &tLR, long &tUD) {
  if (g_testJogPhase == 1) {
    tLR = (g_testJogOriginLR + TEST_JOG_STEPS <= LR_STEP_LIMIT) ? g_testJogOriginLR + TEST_JOG_STEPS
                                                                 : g_testJogOriginLR - TEST_JOG_STEPS;
    tUD = (g_testJogOriginUD + TEST_JOG_STEPS <= UD_STEP_LIMIT) ? g_testJogOriginUD + TEST_JOG_STEPS
                                                                 : g_testJogOriginUD - TEST_JOG_STEPS;
    tLR = ClampLong(tLR, 0, LR_STEP_LIMIT);
    tUD = ClampLong(tUD, 0, UD_STEP_LIMIT);
    const bool lrDone = !g_drvLR.enabled || (!lr.running && lr.position == tLR);
    const bool udDone = !g_drvUD.enabled || (!ud.running && ud.position == tUD);
    if (lrDone && udDone) g_testJogPhase = 2;
  } else {
    tLR = g_testJogOriginLR;
    tUD = g_testJogOriginUD;
    const bool lrDone = !g_drvLR.enabled || (!lr.running && lr.position == tLR);
    const bool udDone = !g_drvUD.enabled || (!ud.running && ud.position == tUD);
    if (lrDone && udDone) {
      g_testJogPhase = 0;
      Serial.println("[s] test jog complete");
    }
  }
}

// -------------------- PAN_MODE_TRACK (scaffold, off by default) --------------------
// Holds the pan pointed at a fixed WORLD direction using the IMU's integrated
// yaw (relative, drift-limited -- not a compass). Manual LEFT/RIGHT always
// overrides while held; on release tracking waits for the axis to stop, then
// re-anchors at the new position. Targets are clamped to the pan soft limits
// here and again in the engine.
static bool UpdatePanTracking(const AxisFeedback &lr, bool manualActive, long &target) {
  g_panMode = PAN_MODE_MANUAL;
  if (!g_panTrackEnabled || !g_drvLR.enabled) {
    g_trackNeedAnchor = true;
    return false;
  }
  ImuEstimate est;
  ReadImu(est);
  if (!est.calibrated || manualActive) {
    g_trackNeedAnchor = true;          // manual wins; re-anchor after release
    return false;
  }
  if (g_trackNeedAnchor) {
    if (lr.running) return false;      // let the manual move finish (engine HOLD = decel)
    g_trackAnchorPos  = lr.position;
    g_trackAnchorYaw  = ImuHeadingDeg(est);
    g_trackTarget     = lr.position;
    g_trackNeedAnchor = false;
  }

  // Boat yaws +d degrees -> pan by -d degrees to hold the world direction.
  const float dYaw = ImuHeadingDeg(est) - g_trackAnchorYaw;
  long t = g_trackAnchorPos - lroundf(PAN_TRACK_SIGN * dYaw * PAN_STEPS_PER_DEG);
  t = ClampLong(t, 0, LR_STEP_LIMIT);
  if (labs(t - g_trackTarget) >= PAN_TRACK_DEADBAND_STEPS) g_trackTarget = t;

  g_panMode = PAN_MODE_TRACK;
  target = g_trackTarget;
  return true;
}

// Call every loop(): decide commands from inputs; the engine enforces limits.
static void UpdateSteppersJog() {
  AxisFeedback lr, ud;
  ReadMotionFeedback(lr, ud);
  g_lrPosSteps = lr.position;
  g_udPosSteps = ud.position;

  AxisMode lrMode = AXIS_HOLD;
  AxisMode udMode = AXIS_HOLD;
  long lrTarget = 0;
  long udTarget = 0;

  // ---------------- Left/Right Jog ----------------
  // PB3 held => left (negative direction), PB4 held => right (positive).
  // Both held (hazard) => no rotation.
  const bool panJog = (g_turnLeftHeld != g_turnRightHeld);
  if (panJog) {
    lrMode = g_turnLeftHeld ? AXIS_JOG_NEG : AXIS_JOG_POS;
  }

  // ---------------- Up/Down Jog ----------------
  const bool liftJog = (g_liftUpHeld != g_liftDownHeld);
  if (liftJog) {
    udMode = g_liftUpHeld ? AXIS_JOG_POS : AXIS_JOG_NEG;
  }

  // ---------------- Pan tracking (only when no manual pan) ----------------
  if (g_testJogPhase == 0) {
    long trackTarget = 0;
    if (UpdatePanTracking(lr, panJog || g_turnLeftHeld || g_turnRightHeld, trackTarget)) {
      lrMode = AXIS_GOTO;
      lrTarget = trackTarget;
    }
  }

  // ---------------- Console test move ----------------
  if (g_testJogPhase != 0) {
    if (panJog || liftJog || g_turnLeftHeld || g_turnRightHeld) {
      g_testJogPhase = 0;                      // buttons always win
      Serial.println("[s] test jog cancelled by button");
    } else {
      TestJogTargets(lr, ud, lrTarget, udTarget);
      if (g_testJogPhase != 0) {
        lrMode = AXIS_GOTO;
        udMode = AXIS_GOTO;
      }
    }
  }

  g_lrJogSpeed = (lrMode == AXIS_JOG_POS) ? LR_JOG_SPEED_STEPS_PER_SEC :
                 (lrMode == AXIS_JOG_NEG) ? -LR_JOG_SPEED_STEPS_PER_SEC : 0.0f;
  g_udJogSpeed = (udMode == AXIS_JOG_POS) ? UD_JOG_SPEED_STEPS_PER_SEC :
                 (udMode == AXIS_JOG_NEG) ? -UD_JOG_SPEED_STEPS_PER_SEC : 0.0f;

  SetAxisCommands(lrMode, lrTarget, udMode, udTarget);
}

// ============================================================================
// END OF STEPPER JOG UPDATE
// ============================================================================





// ============================================================================
// SERIAL DEBUG CONSOLE (USB CDC, non-blocking)
// ============================================================================
#if DEBUG_CONSOLE

static char    g_conLine[32];
static uint8_t g_conLen = 0;

static void ConsoleHelp() {
  Serial.println(F("--- kayak turret console ---"));
  Serial.println(F("?          this menu"));
  Serial.println(F("e          toggle driver EN (only drivers whose UART config verified)"));
  Serial.println(F("d          dump test_connection + DRV_STATUS for both drivers"));
  Serial.println(F("s          short out-and-back jog on each axis (within soft limits)"));
  Serial.println(F("i          print IMU (filtered roll/pitch, relative heading, raw)"));
  Serial.println(F("t          toggle pan tracking (PAN_MODE_TRACK, default OFF)"));
  Serial.println(F("b          print LED power-budget numbers"));
  Serial.println(F("p <0-255>  light-bar PWM override (bench: ignores lift gate)"));
  Serial.println(F("p auto     light-bar back to automatic (lift-gated)"));
}

static void DumpDriver(TMC2209Stepper &d, const DriverStatus &st, const char *name,
                       uint8_t enPin, uint8_t diagPin, const AxisFeedback &fb, long limit) {
  const uint8_t conn = d.test_connection();
  Serial.printf("== %s (addr %u) ==\n", name, (&d == &driverUpDown) ? TMC_ADDR_UPDOWN : TMC_ADDR_LEFTRIGHT);
  Serial.printf("  test_connection : %u (%s)\n", conn, ConnResultText(conn));
  Serial.printf("  config          : %s (IFCNT %u/%u at boot)\n",
                st.configOk ? "VERIFIED" : "NOT VERIFIED", st.ifcntDelta, st.ifcntExpected);
  Serial.printf("  ~EN pin         : %s (%s)\n", digitalRead(enPin) ? "HIGH" : "LOW",
                st.enabled ? "enabled" : "disabled");
  Serial.printf("  DIAG pin        : %s\n", digitalRead(diagPin) ? "HIGH" : "LOW");
  Serial.printf("  position        : %ld / %ld steps (target %ld, %s)\n",
                fb.position, limit, fb.target, fb.running ? "moving" : "stopped");
  if (conn != 0) return;

  const uint32_t ds = d.DRV_STATUS();
  Serial.printf("  DRV_STATUS      : 0x%08lX\n", (unsigned long)ds);
  Serial.printf("    stst=%u stealth=%u cs_actual=%u otpw=%u ot=%u s2ga=%u s2gb=%u ola=%u olb=%u\n",
                d.stst(), d.stealth(), d.cs_actual(), d.otpw(), d.ot(),
                d.s2ga(), d.s2gb(), d.ola(), d.olb());
  Serial.printf("  GCONF 0x%08lX  CHOPCONF 0x%08lX  PWMCONF 0x%08lX\n",
                (unsigned long)d.GCONF(), (unsigned long)d.CHOPCONF(), (unsigned long)d.PWMCONF());
  Serial.printf("  microsteps=%u  rms_current(set)=%u mA  TSTEP=%lu  SG_RESULT=%u  IFCNT=%u\n",
                d.microsteps(), d.rms_current(), (unsigned long)d.TSTEP(), d.SG_RESULT(), d.IFCNT());
}

static void ConsoleExecute(const char *line) {
  while (*line == ' ') line++;
  const char cmd = *line;
  const char *arg = line + 1;
  while (*arg == ' ') arg++;

  switch (cmd) {
    case '?':
      ConsoleHelp();
      break;

    case 'e': {
      const bool want = !(g_drvUD.enabled || g_drvLR.enabled);
      const bool okUD = SetDriverEnabled(g_drvUD, PIN_UPDOWN_EN, want);
      const bool okLR = SetDriverEnabled(g_drvLR, PIN_LEFTRIGHT_EN, want);
      Serial.printf("[e] UP/DOWN %s%s, LEFT/RIGHT %s%s\n",
                    g_drvUD.enabled ? "ENABLED" : "disabled", okUD ? "" : " (refused: config not verified)",
                    g_drvLR.enabled ? "ENABLED" : "disabled", okLR ? "" : " (refused: config not verified)");
      break;
    }

    case 'd': {
      AxisFeedback lr, ud;
      ReadMotionFeedback(lr, ud);
      DumpDriver(driverUpDown,    g_drvUD, "UP/DOWN (A2)",    PIN_UPDOWN_EN,    PIN_UPDOWN_DIAG,    ud, UD_STEP_LIMIT);
      DumpDriver(driverLeftRight, g_drvLR, "LEFT/RIGHT (A3)", PIN_LEFTRIGHT_EN, PIN_LEFTRIGHT_DIAG, lr, LR_STEP_LIMIT);
      break;
    }

    case 's':
      if (!g_drvUD.enabled && !g_drvLR.enabled) {
        Serial.println("[s] no driver enabled -- nothing would move ('e' to enable)");
        break;
      }
      StartTestJog();
      Serial.printf("[s] test jog +/-%ld steps on each enabled axis\n", TEST_JOG_STEPS);
      break;

    case 'i': {
      ImuEstimate e;
      ReadImu(e);
      if (!e.present) {
        Serial.println("[i] MPU6050 not detected at boot");
        break;
      }
      if (!e.calibrated) {
        Serial.printf("[i] calibrating gyro bias (keep still)... errors %lu\n", (unsigned long)e.readErrors);
        break;
      }
      Serial.printf("[i] roll %.1f  pitch %.1f  heading %.1f deg (relative, drifts)  yaw rate %.2f dps\n",
                    (double)e.rollDeg, (double)e.pitchDeg, (double)ImuHeadingDeg(e), (double)e.yawRateDps);
      Serial.printf("    accel %.2f %.2f %.2f m/s2  gyro %.4f %.4f %.4f rad/s  temp %.1f C  samples %lu  errors %lu\n",
                    (double)e.ax, (double)e.ay, (double)e.az, (double)e.gx, (double)e.gy, (double)e.gz,
                    (double)e.tempC, (unsigned long)e.samples, (unsigned long)e.readErrors);
      Serial.printf("    pan mode %s (tracking %s)\n", g_panMode == PAN_MODE_TRACK ? "TRACK" : "MANUAL",
                    g_panTrackEnabled ? "enabled" : "disabled");
      break;
    }

    case 't':
      g_panTrackEnabled = !g_panTrackEnabled;
      g_trackNeedAnchor = true;
      Serial.printf("[t] pan tracking %s%s\n", g_panTrackEnabled ? "ENABLED" : "disabled",
                    g_panTrackEnabled ? " (anchors at the current pan position + heading)" : "");
      break;

    case 'b':
      Serial.printf("[b] lit %u/%u px (%.3f) x applied %u/255 = %.4f (limit %.2f)  requested %u%s\n",
                    g_budget.litPixels, g_budget.totalPixels, (double)g_budget.litFraction,
                    g_budget.applied, (double)g_budget.product, (double)LED_POWER_BUDGET_MAX,
                    g_budget.requested, g_budget.limited ? "  ** LIMITED **" : "");
      break;

    case 'p': {
      if (strncmp(arg, "auto", 4) == 0) {
        g_lightBarOverride = -1;
        Serial.println("[p] light bars automatic (on only with lift at UP limit)");
        break;
      }
      char *end = nullptr;
      const long v = strtol(arg, &end, 10);
      if (end == arg || v < 0 || v > 255) {
        Serial.printf("[p] usage: p <0-255> | p auto   (now: %s, duty %u)\n",
                      g_lightBarOverride >= 0 ? "override" : "auto", g_lightBarDuty);
        break;
      }
      g_lightBarOverride = (int16_t)v;
      Serial.printf("[p] light-bar OVERRIDE duty %ld/255 at %lu Hz (lift gate bypassed; 'p auto' to clear)\n",
                    v, (unsigned long)LIGHTBAR_PWM_FREQ_HZ);
      break;
    }

    case '\0':
      break;

    default:
      Serial.printf("unknown command '%c' -- '?' for help\n", cmd);
      break;
  }
}

// Non-blocking: consumes whatever bytes are available, executes on CR/LF.
static void PollConsole() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r' || c == '\n') {
      g_conLine[g_conLen] = '\0';
      if (g_conLen > 0) ConsoleExecute(g_conLine);
      g_conLen = 0;
    } else if (g_conLen < sizeof(g_conLine) - 1) {
      g_conLine[g_conLen++] = (char)c;
    }
  }
}

#endif  // DEBUG_CONSOLE

// ============================================================================
// END OF SERIAL DEBUG CONSOLE
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
  // -------------------- SAFE OUTPUT STATE FIRST --------------------
  // Order matters: drivers disabled, strips dark, light bars at 0 -- before
  // anything that can take time or fail (USB, I2C, UART).

  // Stepper drivers: ~{EN} HIGH = disabled (see STEPPER DRIVER ENABLES below)
  digitalWrite(PIN_UPDOWN_EN, HIGH);
  digitalWrite(PIN_LEFTRIGHT_EN, HIGH);
  pinMode(PIN_UPDOWN_EN, OUTPUT);
  pinMode(PIN_LEFTRIGHT_EN, OUTPUT);

  // LED strips: all dark (render buffers zeroed, budget-checked, shown)
  stripBottomLeft.begin();
  stripBottomRight.begin();
  stripCockpitFront.begin();
  stripCockpitBack.begin();
  ClearAllStrips();
  SetBrightnessAll(BRIGHTNESS_DEFAULT);
  EnforcePowerBudget();
  ShowAllStrips();

  // Light-bar PWM at 0 (LEDC 5 kHz, attached with duty already 0)
  LightBarPwmInit();

  // Status LED off
  StatusLedInit();

  // -------------------- CORE --------------------
  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);   // allow power rails + level shifter to settle

  // -------------------- BUTTON INPUTS --------------------
  pinMode(PIN_PB_BRIGHTNESS, INPUT_PULLUP);
  pinMode(PIN_PB_MODE,       INPUT_PULLUP);
  pinMode(PIN_PB_TURN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_PB_TURN_RIGHT, INPUT_PULLUP);

  // -------------------- TMC2209 DIAG INPUTS --------------------
  // Driven by the TMC2209 at VIO (3.3 V). Pull-down keeps a missing module LOW.
  pinMode(PIN_UPDOWN_DIAG,    INPUT_PULLDOWN);
  pinMode(PIN_LEFTRIGHT_DIAG, INPUT_PULLDOWN);

  // Establish known starting positions (logical 0)
  InitStepperState();

  // -------------------- TMC2209 UART BRING-UP --------------------
  // Status LED: BLUE while probing, GREEN = both drivers verified,
  // RED = one or both failed (a failed driver stays disabled).
  StatusLed(false, false, true);
  Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_UART_RX, PIN_TMC_UART_TX);
  driverUpDown.begin();      // no pin/baud side effects on ESP32 (SW_CAPABLE_PLATFORM false)
  driverLeftRight.begin();

  const bool okUD = ConfigureDriver(driverUpDown,    g_drvUD, "UP/DOWN (A2, addr 0)");
  const bool okLR = ConfigureDriver(driverLeftRight, g_drvLR, "LEFT/RIGHT (A3, addr 1)");

  // -------------------- STEPPER DRIVER ENABLES --------------------
  // ~{EN} is active-LOW and was driven HIGH (disabled) at the top of setup().
  // SAFE POWER-ON: a driver is enabled ONLY after its UART config verified.
  // A driver that failed stays disabled: its current would otherwise be set
  // by the module's VREF trimpot, not by firmware.
  SetDriverEnabled(g_drvUD, PIN_UPDOWN_EN,    okUD);
  SetDriverEnabled(g_drvLR, PIN_LEFTRIGHT_EN, okLR);
  if (okUD && okLR) {
    StatusLed(false, true, false);
    Serial.println("[TMC] both drivers verified and ENABLED");
  } else {
    StatusLed(true, false, false);
    Serial.printf("[TMC] driver config FAILED (UP/DOWN %s, LEFT/RIGHT %s) -- failed driver(s) left DISABLED\n",
                  okUD ? "ok" : "FAIL", okLR ? "ok" : "FAIL");
  }

  // Motion engine on core 0 (owns the AccelStepper objects from here on)
  StartStepperTask();

  // -------------------- IMU (MPU6050) --------------------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Optional sanity check
  if (!imu.begin(IMU_I2C_ADDR, &Wire)) {
    Serial.println("MPU6050 NOT DETECTED");
  } else {
    Serial.println("MPU6050 OK -- calibrating gyro bias, keep still ~1 s");
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    g_imu.present = true;
    StartImuTask();   // the IMU task owns Wire/imu from here on
  }


  // -------------------- INITIAL STATE --------------------
  g_mode = MODE_NAV_LIGHTS;
  g_brightnessIndex = 0;
  ApplyBrightnessFromIndex();

  g_turnLeftHeld  = false;
  g_turnRightHeld = false;

  // Render initial mode
  RenderNavLights();
  EnforcePowerBudget();
  ShowAllStrips();

  Serial.println("Setup complete");
#if DEBUG_CONSOLE
  Serial.println("Type ? for the debug console menu");
#endif
}

// ============================================================================
// END OF SETUP()
// ============================================================================





// ============================================================================
// LOOP()
// ============================================================================
//
// Scheduler order (every cycle):
//  0) Poll the debug console (if compiled in)
//  1) Read inputs (debounced + edges + held states)
//  2) Update "one-shot" state changes (brightness/mode changes)
//  3) Update stepper jog commands (engine runs in its own task on core 0)
//  4) Every FRAME_INTERVAL_MS: render base mode + overlay, enforce the power
//     budget, show
// ============================================================================

void loop() {
  const uint32_t nowMs = millis();

  // -------------------- 0) DEBUG CONSOLE --------------------
#if DEBUG_CONSOLE
  PollConsole();
#endif

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

  // Mode button (PB2) click cycles lighting modes
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

  // Mode button long press re-zeros the tracking heading
  if (g_modeLongPressEdge) {
    ImuEstimate e;
    ReadImu(e);
    g_yawZeroDeg = e.yawDeg;
    g_trackNeedAnchor = true;
    Serial.println("[MODE long] heading re-zeroed");
  }

  // -------------------- 3) STEPPER UPDATE --------------------
  // Jog commands from held buttons; the engine on core 0 executes them and
  // enforces the soft limits. Done before rendering so button-to-motion
  // latency does not include a ~24 ms show().
  UpdateSteppersJog();

  // Light bars: brightness-index duty, gated on lift at UP soft limit
  UpdateLightBars();

  // -------------------- 4) LED RENDERING (frame scheduler) --------------------
  // Replaces the old delay(5): render + show at FRAME_INTERVAL_MS. Steppers
  // no longer depend on loop() rate.
  if ((nowMs - g_lastFrameMs) >= FRAME_INTERVAL_MS) {
    g_lastFrameMs = nowMs;

    // Base mode first
    RenderBaseMode(nowMs);

    // Overlay turn/hazard effects when turn buttons are held
    // (This can override base visuals during hazard/turn indication)
    if (g_turnLeftHeld || g_turnRightHeld) {
      ApplyTurnHazardOverlay(nowMs);
    }

    // Power budget from the actual pixel buffers, then push pixels out.
    // HARD SAFETY CONSTRAINT: nothing may sit between these two calls.
    EnforcePowerBudget();
    ShowAllStrips();
  }
}

// ============================================================================
// END OF LOOP()
// ============================================================================
