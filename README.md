# kayak-turret-light-

You are my firmware engineer for a kayak-mounted LED light turret. Goal of this session: get kayak_working.ino to a state that compiles for arduino:esp32:nano_nora, matches the KiCad schematic pin-for-pin, and drives every subsystem in a staged bring-up — without touching the hardware safety constraints below. Work in small, verifiable steps; after every change, compile. Do not ask me to confirm things you can verify from the files.
kayak working.net — KiCad 9 netlist. This is ground truth for every pin assignment. Parse it and produce a table: Arduino pin → net → destination (TMC2209 A2/A3 pins, 74HCT245 A/B pins, GY-521, J6 button terminals, J2 LED data terminals). Firmware follows KiCad, never the reverse.
kayak_working.ino — current firmware. It is known to be stale vs. the schematic. Known gaps: buttons declared on A2–A5 (schematic uses A0–A3), I2C declared on A6/A7 (schematic uses A4/SDA, A5/SCL), LED counts 250/250/50/50 (real: 256/256/44/44), MPU6050 initialized but never read, lift stepper has no button mapping, TMC2209s used in bare STEP/DIR with no UART config, DIAG pins unused.
BIGTREETECH TMC2209 V1.3 User Manual.pdf + TMC2209 V1.3SCH.pdf — UART wiring (single-wire with 1k series resistor on the module), MS1/MS2 = UART address bits, DIAG = StallGuard output, VIO = 3.3V logic, VS = 12V motor supply.
ABX00083fullpinout.pdf / ABX00083datasheet.pdf — Nano ESP32-S3 pinout. Note: Arduino D/A names map to ESP32 GPIO numbers; the onboard RGB LED is active-LOW (R=GPIO46, G=GPIO0, B=GPIO45). D13 is off-limits.
sn74hct245.pdf — level shifter. DIR must be HIGH for A→B, OE/CE tied LOW.
mpu6050.pdf, GY521SCH.jpg — IMU. GY-521 VCC gets 5V (onboard LDO), logic is 3.3V-safe.
BIGTREETECH_Octopus_EN_updated_0719.pdf + BIGTREETECH Octopus SCH.pdf — reference only, for how BTT wires TMC2209 UART on a known-good board.
Hardware facts (do not re-derive)
MCU: Arduino Nano ESP32-S3, FQBN arduino:esp32:nano_nora. Arduino IDE / arduino-cli, ESP32 core pinned to 2.0.17 (newer cores broke USB-CDC auto-reconnect after upload). D0/D1 need native USB CDC so UART0 doesn't collide with Serial debug.
Steppers: 2× BESJMYT NEMA17, 1A rated, 3.5Ω, 3.8mH, 145mNm, 1.8°. Drivers: 2× BTT TMC2209 V1.3. Pan axis = "LEFT/RIGHT" (≤180° travel). Lift axis = "UP/DOWN".
LEDs: 4× SK6812 RGBW 12V strips through SN74HCT245N + 330Ω series. Counts: BottomLeft 256, BottomRight 256, CockpitFront 44, CockpitBack 44. Strip power is off-board; only data goes through the PCB.
Light bars: 3× 18W bars dimmed as one group via a single-channel MOSFET module (0–20 kHz PWM in) on the "PWM CONTROL" terminal of J2 — the pin currently called PIN_PWM_DATA.
IMU: GY-521 MPU6050 on I2C (A4/A5), address 0x68 (AD0 low).
Buttons: 4× momentary to GND on J6, INPUT_PULLUP, no external resistors. Order on J6: BRIGHTNESS, MODE, LEFT, RIGHT, COMMON GND.
Power: Milwaukee 18V → 12V (20A) → 5V (3A WG-12S0503). Arduino VUSB/5V pin is USB-only; the board is fed via VIN/5V from the converter — confirm from the netlist which.
Libraries on hand: TMCStepper (teemuatlut), AccelStepper, Adafruit_NeoPixel, Bounce2, Adafruit_MPU6050 + Adafruit_Sensor, jrowberg I2Cdev/MPU6050 (DMP). Pick Adafruit_MPU6050 unless you find a concrete reason the DMP path is needed for pan stabilization; say which you picked and why.
HARD SAFETY CONSTRAINTS — never remove, bypass, or weaken
LED power budget: (fraction of LEDs lit) × (applied brightness fraction) ≤ 0.20, computed from the actual pixel buffer every frame, not inferred from mode logic. Implement/keep as EnforcePowerBudget() called in loop() immediately before ShowAllStrips(). If the current file lacks it, write it: walk every pixel in every strip, count non-zero pixels, compute litFraction × (brightness/255), and if > 0.20 scale brightness down (not pixels off) until it passes. Every new lighting mode goes through it, never around it.
Stepper current: TMC2209 rms_current() ≤ 800 mA run, with automatic standstill reduction (ihold ≈ 30% of irun). Motors are 1A rated; never configure above that.
Soft limits on both axes stay enforced in the jog engine; pan ≤ 180° of travel.
Safe power-on: all strips dark, light-bar PWM at 0, drivers disabled (EN HIGH) until UART config succeeds, then enabled.
Work plan — do these in order, compile after each, commit after each
Phase 0 — Pin audit
Parse the netlist. Rewrite the PIN DEFINITIONS block of kayak_working.ino to match it exactly, including DIAG pins for both drivers and the TMC2209 UART pin(s). Add a comment on each line citing the net name.
Print the before/after pin table in your summary. Flag any net in the schematic that has no firmware consumer, and any firmware pin with no net.
Fix LED counts to 256/256/44/44.
Phase 1 — Compile baseline
arduino-cli compile --fqbn arduino:esp32:nano_nora kayak_working.ino (install core 2.0.17 and the libraries above if missing). Fix errors only; no feature work yet. Report RAM/flash usage — four NeoPixel buffers (600 RGBW pixels = 2400 B) plus DMA is fine on S3, but say so.
Phase 2 — TMC2209 UART bring-up
Add TMCStepper: one HardwareSerial (Serial1 on the pins from the netlist) shared by both drivers, addresses from MS1/MS2 as wired (read the netlist — don't assume 0 and 1).
In setup(): begin(), test_connection() for each driver, rms_current(800), microsteps(16), en_spreadCycle(false) (StealthChop), pwm_autoscale(true), TCOOLTHRS/SGTHRS placeholders for StallGuard homing, ihold reduction. Only then drive EN LOW.
Onboard RGB LED = status: blue = UART probing, green = both drivers answered, red = one or both failed. Active-LOW.
Add a Serial debug console (non-blocking, Serial.available()): ? menu, e toggle EN, d dump DRV_STATUS + test_connection for both, s short jog each axis, i print IMU, b print current power-budget numbers, p <0-255> set light-bar PWM. Keep this behind #define DEBUG_CONSOLE 1 so it can be compiled out.
Phase 3 — Lift axis + light bars
Map lift control: BRIGHTNESS held + LEFT = lift down, BRIGHTNESS held + RIGHT = lift up (chord on existing four buttons; don't invent new hardware). While the chord is active, suppress the normal brightness-cycle and pan-jog actions. Set g_liftUpHeld/g_liftDownHeld from this.
Replace analogWrite on the light-bar pin with ledcSetup/ledcAttachPin at 5 kHz, 8-bit (the MOSFET module takes 0–20 kHz). Tie light-bar duty to the brightness index, and gate it: light bars only on when the can is at the "up" soft limit.
Phase 4 — IMU read + pan tracking scaffold
Read the MPU6050 in loop() at ~100 Hz into a complementary-filtered heading/tilt estimate. Print it from the i command.
Add a PAN_MODE_TRACK scaffold: target pan position derived from filtered yaw rate integration (true heading is not available from a 6-DOF IMU — say this in a comment; it's drift-limited and gets re-zeroed by a long-press of MODE). Manual LEFT/RIGHT held always overrides tracking while held, and tracking resumes from the new position on release. Keep it disabled by default (static bool g_panTrackEnabled = false) and toggled by a t console command until bench-tested.
Phase 5 — Verify
Full compile, zero warnings at -Wall if practical.
Produce BRINGUP_CHECKLIST.md: an ordered bench procedure — what to probe with the DMM at each step (EN pin level, VIO = 3.3V, VS = 12V, UART idle HIGH), what the RGB LED should show, what each console command should print, and what "pass" looks like for each phase. Include the open physical question: there is an unidentified pot-like component on the TMC2209 V1.3 module not on the BTT schematic — tell me what to photograph/measure to identify it.
Summarize: what is verified by compile, what is verified by netlist, and what still needs a physical continuity check or bench test. Don't blur these three categories.
Style rules
Rewrite code directly; don't just critique. Keep the file's existing section banners and non-blocking structure; no delay() in loop() other than the existing 5 ms yield (and replace that with a millis() scheduler if it costs stepper step rate).
Be blunt about electrical risk. If the netlist shows something that will damage hardware (e.g., 5V on a 3.3V-only pin, DIAG pulled up to 5V, missing common ground), stop and tell me before writing firmware that assumes it.
One clarifying question max per phase, and only if the netlist and datasheets can't settle it. Otherwise state the assumption in a code comment and proceed.
Keep kayak1.txt and sketch_dec2a.txt out of this — they are superseded prototypes.