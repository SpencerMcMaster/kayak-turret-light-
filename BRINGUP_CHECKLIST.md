# Kayak LED Turret — Bench Bring-Up Checklist

Firmware: `kayak_working/kayak_working.ino` · Netlist: `kayak working.txt` (KiCad 9.0.6 netlist, rev 2.0)

Work top to bottom. Do not start a stage until the previous one passes.

**Standalone bench sketches** (`bench_tests/`, see its README) test one subsystem each and are the quickest way through the early stages:

| Stage | Flash this first | Then |
|---|---|---|
| 3 (first boot on the PCB) | `test_buttons` (USB power is enough), then `test_imu` (needs 12 V, and the Stage 2.4 pull-up check passed), then `test_tmc_uart` (never enables a driver) | `kayak_working` boot checks |
| 4 | `test_motor`, `TEST_AXIS` 0 then 1 | `kayak_working` jog / chord / limit checks |
| 5 | `test_led_strip`, `TEST_STRIP` 0–3 | `kayak_working` budget checks |
| 6 | `test_lightbar` | `kayak_working` lift-gated light bars |
| 7–8 | — | `kayak_working` IMU / tracking |

**Never plug or unplug a stepper motor while 12 V is on** (TMC2209 output stage damage).

---

## Open items: resolve before or during bring-up

| # | Item | Blocks | How to resolve |
|---|---|---|---|
| 1 | **TMC UART wiring: D1/TX reaches nothing on factory modules** | Stage 3 green LED, all motion | See [Open item 1](#open-item-1--tmc2209-uart-tx-path-r10) |
| 2 | **GY-521 I²C pull-up rail** (3.3 V or 5 V?) | Stage 2 (before the Nano is fitted) | Measure in Stage 2, step 2.4 |
| 3 | **Level shifter part**: netlist value says `74HC245`, build uses SN74HCT245N | Stage 5 strips | Read the chip marking. It must say **HCT**. An HC part at 5 V needs VIH ≥ 3.5 V, and 3.3 V drive is out of spec |
| 4 | **Unidentified pot-like part on the TMC2209 module** | Nothing (UART mode ignores it). See note | See [Open item 4](#open-item-4--the-pot-like-part-on-the-tmc2209-v13) |
| 5 | **No homing yet**: pan must be at full LEFT and lift at full DOWN at power-on | Stage 4 soft limits | Position by hand before every power-on until StallGuard homing is tuned |
| 6 | **Outputs float before `setup()`** (ROM bootloader + ~0.9 s core double-tap fade): LED data inputs on U1 and D8 are undriven | Board revision | See [Board revision items](#board-revision-items) |

---

## Stage 0: Build and flash (Nano off the board, USB only)

1. Arduino IDE → Board **Arduino Nano ESP32**, core **esp32 2.0.17**.
2. **Tools → Pin Numbering → "By GPIO number (legacy)"**. The sketch has an `#error` guard and will not compile otherwise.
   CLI: `arduino-cli compile kayak_working` (reads `kayak_working/sketch.yaml`).
3. Libraries: TMCStepper 0.7.3, AccelStepper ≥ 1.64, Adafruit NeoPixel 1.15.5, Adafruit MPU6050 2.2.9, Adafruit Unified Sensor 1.1.15, Adafruit BusIO 1.17.4.
4. **Do not flash the old firmware onto this board.** Its EN/DIR assignments are swapped, and it runs I²C on A6/A7, which are the DIAG nets.
5. Flash the bare Nano over USB and open the Serial Monitor at 115200.

| Check | Pass |
|---|---|
| RGB LED at boot | **Blue** briefly, then **red** (no drivers present) |
| Serial | `[TMC] UP/DOWN (A2, addr 0): test_connection = 2 (NO REPLY ...)`, same for LEFT/RIGHT, `driver config FAILED ... left DISABLED`, `MPU6050 NOT DETECTED`, `Setup complete` |
| `?` | Menu lists `? e d r s i b p t` |
| `e` | `refused: config not verified` for both |
| DMM: D2 and D5 to GND | **3.3 V** (~EN HIGH = disabled) |
| DMM: D8 to GND | **0 V** (light-bar PWM 0) |

---

## Stage 1: Unpowered continuity (no Nano, no modules, no supply)

Every row is a netlist connection. Beep it out on the bare PCB.

| Nano pin | GPIO | Net | Destination |
|---|---|---|---|
| D0/RX | 44 | `Net-(A1-D0/RX)` | A2 pin 5 **and** A3 pin 5 (module "RX" = UART_RX = PDN_UART) |
| D1/TX | 43 | `Net-(A1-D1/TX)` | A2 pin 4 **and** A3 pin 4 (module "TX" = UART_TX) |
| D2 | 5 | `Net-(A3-~{EN})` | A3 pin 8 EN (pan) |
| D3 | 6 | `Net-(A3-STEP)` | A3 pin 2 STEP (pan) |
| D4 | 7 | `Net-(A3-DIR)` | A3 pin 1 DIR (pan) |
| D5 | 8 | `Net-(A2-~{EN})` | A2 pin 8 EN (lift) |
| D6 | 9 | `Net-(A2-STEP)` | A2 pin 2 STEP (lift) |
| D7 | 10 | `Net-(A2-DIR)` | A2 pin 1 DIR (lift) |
| D8 | 17 | `Net-(J2-Pin_5)` | J2.5 PWM CONTROL (direct, not level-shifted) |
| D9 | 18 | `Net-(U1-A3)` | U1 pin 5 (A3) → B3 pin 15 → R4 → J2.4 Cockpit Back |
| D10 | 21 | `Net-(U1-A2)` | U1 pin 4 (A2) → B2 pin 16 → R3 → J2.3 Cockpit Front |
| D11 | 38 | `Net-(A1-D11_MOSI)` | U1 pin 3 (A1) → B1 pin 17 → R2 → J2.2 Bottom Right |
| D12 | 47 | `Net-(A1-D12_MISO)` | U1 pin 2 (A0) → B0 pin 18 → R1 → J2.1 Bottom Left |
| A0 | 1 | `Net-(J6-Pin_4)` | J6.4 RIGHT |
| A1 | 2 | `Net-(J6-Pin_3)` | J6.3 LEFT |
| A2 | 3 | `Net-(J6-Pin_2)` | J6.2 MODE |
| A3 | 4 | `Net-(J6-Pin_1)` | J6.1 BRIGHTNESS |
| A4/SDA | 11 | `Net-(A1-A4/SDA)` | GY-521 SDA |
| A5/SCL | 12 | `Net-(A1-A5/SCL)` | GY-521 SCL |
| A6 | 13 | `Net-(A1-PadA6)` | A2 DIAG (lift) |
| A7 | 14 | `Net-(A1-PadA7)` | A3 DIAG (pan) |
| VIN | — | `+12V` | J5.1, A2/A3 VS (pin 9) |
| 3V3 | — | `+3V3` | A2/A3 VIO (pin 15), A3 MS1 (pin 7) |
| GND | — | `GND` | J1.2, J5.2, J6.5, U1 pins 10 and 19 (CE), A2/A3 GND pins, A2 MS1+MS2, A3 MS2, GY-521 GND |
| — | — | `+5V` | J1.1, U1 pin 20 (VCC), U1 pin 1 (DIR "A->B"), GY-521 VCC |

| Check | Pass |
|---|---|
| Every row above | Beeps |
| +12V↔GND, +5V↔GND, +3V3↔GND | **No** short (expect a cap-charging blip, then open) |
| D13 | Connected to nothing |
| U1 chip marking | **SN74HCT245N** (open item 3) |

---

## Stage 2: Power rails (12 V on, no Nano, no modules, no motors, no strips)

| Step | Probe (DMM, black on GND) | Pass |
|---|---|---|
| 2.1 | J5.1 / Nano VIN socket pin | **11.5–12.6 V** |
| 2.2 | J1.1 / U1 pin 20 / GY-521 VCC | **4.9–5.2 V** |
| 2.3 | U1 pin 1 (DIR) | = 5 V rail (HIGH, A→B) · U1 pin 19 (CE) = **0 V** |
| 2.4 | **GY-521 fitted, Nano still out**: SDA and SCL to GND | **≈ 3.3 V → OK.** **≈ 5 V → STOP**: pull-ups go to VCC, which would put 5 V on A4/A5. Remove the GY-521's pull-ups or add a level shifter before fitting the Nano |
| 2.5 | Module sockets VS (pin 9) | **12 V** · VIO (pin 15) = **0 V** (no Nano yet) |

---

## Stage 3: Nano fitted, modules fitted, 12 V on, **motors and strips disconnected**

| Check | Pass |
|---|---|
| Module VIO (pin 15), both | **3.3 V** |
| Module VS (pin 9), both | **12 V** |
| RGB LED | **Blue** (≈ 0.1 s) → **green** if open item 1 is fixed, **red** if not |
| EN pins (module pin 8) | **0 V** if that driver verified (green), **3.3 V** if it failed. A failed driver must stay at 3.3 V |
| UART idle, D1 | **3.3 V** (TX idles HIGH) |
| UART idle, D0 | **≈ 3.0 V** with 1 kΩ fitted (the modules' 2 × 20 kΩ pull-downs against the 1 kΩ). **≈ 0 V means TX is not reaching PDN_UART** (R10 still open) |
| DIAG, A6 and A7 | **0 V** |
| D8 | **0 V** (light bars off: lift is not at its UP limit) |

Boot log when it passes:
```
[TMC] UP/DOWN (A2, addr 0): writes 21/21 acked, GCONF ok, CHOPCONF ok, PWMCONF ok -> CONFIG OK (IRUN set 795 mA RMS, IHOLD 30%)
[TMC] LEFT/RIGHT (A3, addr 1): writes 21/21 acked, ... -> CONFIG OK ...
[TMC] both drivers verified and ENABLED
MPU6050 OK -- calibrating gyro bias, keep still ~1 s
```
`d` must show, for **each** driver: `test_connection : 0 (OK)`, `config : VERIFIED (IFCNT 21/21 at boot)`, `~EN pin : LOW (enabled)`, `microsteps=16`, `rms_current(set)=795 mA`, `stealth=1`, `stst=1`, `ot=0 otpw=0 s2ga=0 s2gb=0`. `ola=1 olb=1` (open load) is **expected with no motor connected**.

`d` takes about 0.1–0.3 s (many UART reads). The buttons are not sampled during that time, so don't run it while jogging.

**Fail modes:** `test_connection = 2 (NO REPLY)` → open item 1 (TX path), wrong address, or no 12 V on VS (the TMC2209 logic is powered from VS). `= 1 (0xFFFFFFFF)` → bus stuck HIGH. `writes n/21` with n < 21 → UART bit errors (check the series resistor value and wiring).

---

## Stage 4: Motors (12 V OFF while connecting!)

Hand-position pan at **full LEFT** and lift at **full DOWN** (open item 5), connect J3 (lift) and J4 (pan), then power on.

| Check | Pass |
|---|---|
| `d` | `ola=0 olb=0` with the motor connected; `cs_actual` ≈ **7** at standstill (IHOLD ≈ 30 % of IRUN 25) |
| `s` | Each axis moves out 200 steps and back, then `[s] test jog complete`. During the move `cs_actual` ≈ **25**; ≈ 0.5 s after stopping it falls back to ≈ **7** (automatic standstill reduction) |
| Hold LEFT / RIGHT | Pan moves left / right, decelerates smoothly on release |
| Hold at a limit | Motion stops at position 0 / `LR_STEP_LIMIT`, never past it (`d` → `position`) |
| BRIGHTNESS + RIGHT / + LEFT | Lift up / down; **no pan**, no brightness change |
| Release BRIGHTNESS first, keep RIGHT held | Nothing moves until RIGHT is released and pressed again |
| Mark the pan at 0 and at `LR_STEP_LIMIT` | Arc **≤ 180°**. If larger, lower `LR_STEP_LIMIT` |
| Wrong direction | Swap one coil pair at J3/J4, or set the DIR polarity in firmware |
| Motor case after 5 min holding | Warm, not too hot to touch (< ~60 °C) |
| `e` | Both disable (EN 3.3 V, motors go limp; **the lift may drop under load**, and the firmware's position is then wrong: re-position by hand), `e` again re-enables |
| **12 V dropout with USB still connected**: switch the 12 V supply off for 2 s, then back on | Within 0.25 s: `[TMC] ... FAULT: no reply or config lost ... DISABLED`, LED **red**, EN pins **3.3 V**, and they **stay** 3.3 V when 12 V returns (the motors must not energise). Then `r` → green, EN 0 V |

---

## Stage 5: LED strips (strip 12 V supply on, data via J2.1–J2.4)

| Check | Pass |
|---|---|
| Boot | All strips dark until `Setup complete`, then nav lights at the lowest level |
| `b` at boot | `lit 600/600 (1.000) x applied 16/255 = 0.0627` |
| Click BRIGHTNESS to the top level, `b` | `requested 220 ... applied 51 ... = 0.2000 ** LIMITED **` |
| Brightness levels in all-lit modes (nav, flood, pulse) | **Expected:** levels 2, 3 and 4 (requested 84/152/220) all show at applied 51, so only level 1 (16) looks different. That is the 0.20 budget doing its job, not a button fault. If you want four visible steps there, the levels have to sit at or below 51 |
| MODE through every mode, `b` each time | `product` **≤ 0.2000** always |
| Strip supply current (clamp meter), white flood at top level | Record it. It must not exceed ≈ 20 % of the strips' rated full-white current |
| Colour order | Bottom Left red, Bottom Right green, cockpit white. If wrong, change `NEO_GRBW` ↔ `NEO_RGBW` |

---

## Stage 6: Light bars (MOSFET module on J2.5)

| Check | Pass |
|---|---|
| `p 64` | D8: DMM DC ≈ **0.83 V**, frequency (DMM Hz mode) ≈ **5.0 kHz**; bars dim |
| `p 255` / `p 0` | ≈ 3.3 V / 0 V |
| `p auto`, lift not at UP | Bars **off** |
| Lift to its UP soft limit (BRIGHTNESS + RIGHT until it stops) | Bars **on** at the brightness level's duty (64/128/191/255); they go off as soon as the lift moves down |

---

## Stage 7: IMU

| Check | Pass |
|---|---|
| `i` within 1 s of boot | `calibrating gyro bias (keep still)` |
| `i` flat and still | roll and pitch ≈ 0 ± 2°, `errors 0`, `samples` increasing ≈ 100/s |
| Rotate the board 90° by hand, slowly | heading changes by ≈ 90° (sign = mounting) |
| Still for 5 min | heading drift: record it (expect a few °/min; this is why re-zeroing exists) |
| Hold MODE ≥ 1 s | `[MODE long] heading re-zeroed`, lighting mode **unchanged** |
| Tilt 30° | roll/pitch follow within ~1 s, no runaway |

---

## Stage 8: Pan tracking (scaffold, off by default)

1. `t` → `pan tracking ENABLED`.
2. Rotate the base slowly: the pan should counter-rotate to hold its world direction. **If it turns the same way, set `PAN_TRACK_SIGN = -1.0f`.**
3. Hold LEFT: manual pan wins. Release: tracking resumes from the new position.
4. At a soft limit, tracking stops there (it never drives past it).
5. `t` again to disable. Leave `g_panTrackEnabled = false` in the code until this stage passes on the water.

---

## Open item 1: TMC2209 UART TX path (R10)

The BTT TMC2209 V1.3 module schematic (`TMC2209 V1.3SCH.pdf`) and manual show:

* Header pin 5 **"RX" (UART_RX)** connects **directly** to the chip's PDN_UART pin, with a 20 kΩ pull-down (R6).
* Header pin 4 **"TX" (UART_TX)** connects to PDN_UART **only through R10, which is not fitted** at the factory ("UART (default)" = RX pin only). The manual's "DIY" option is to **solder R10's pads together**.
* **There is no series resistor on the module.**

This PCB wires Nano D0/RX → both modules' RX pins and Nano D1/TX → both modules' TX pins. With factory modules, **D1 drives nothing**: the drivers never receive a command, `test_connection` returns `NO REPLY`, the LED goes red and both drivers stay disabled. That is a safe failure, but nothing moves.

| Option | Verdict |
|---|---|
| **A. Fit a 1 kΩ resistor at R10 on ONE module** (match the pad size) | **Recommended.** Gives the standard single-wire topology: D1 —1 kΩ— PDN_UART bus — D0. Works with the firmware as-is. Fitting 1 kΩ on **both** modules is also acceptable (500 Ω effective; ≈ 6.6 mA when a driver pulls the bus low), and keeps the modules interchangeable |
| **B. Solder-bridge R10 (as the BTT manual shows)** | **Do not.** That puts D1's push-pull output directly on PDN_UART. Every time a driver answers, it drives LOW against D1's idle HIGH with no current limit. Firmware cannot prevent this |
| C. Next PCB revision | Put a 1 kΩ in series from D1 to the UART_RX net on the carrier board and leave the module TX pins unconnected. Firmware unchanged |

Also note: the ESP32-S3 ROM prints its boot log on GPIO43 (D1) at every reset. With option A it reaches the PDN_UART bus. The TMC2209 rejects it (no valid sync/CRC), and both drivers are disabled through boot anyway (EN has a 20 kΩ pull-up to VIO).

## Open item 4: the pot-like part on the TMC2209 V1.3

It is almost certainly the **VREF trimmer**: the small square trimpot with a cross slot next to the DIAG and VS pins, visible in the manual's top-view photo. The module schematic draws VREF (chip pin 17) as a fixed 20 kΩ/20 kΩ divider from 5VOUT (R8/R9) plus R11 = 0 Ω, so the trimpot replaces part of that divider on the real board.

**What to capture so it can be confirmed:**
1. Photos: the top side square-on, a close-up of the part's marking (e.g. `103` = 10 kΩ, `203` = 20 kΩ), and the bottom side around R8, R9 and R11.
2. Module **unplugged**: resistance between the two outer terminals (fixed value), then wiper to each end while turning it (should sweep 0 → full).
3. Module plugged in, 12 V on, motors disconnected: DC voltage from the wiper to GND while turning it slowly. Expect about **0 → ~2.5 V** (VREF range).
4. Continuity from the wiper to R11 (the 0 Ω link to the VREF pin).

**Why it does not matter in this firmware:** `I_scale_analog` is written **0** and read back, so the current comes from the internal reference and IRUN/IHOLD only. The trimpot setting has no effect in UART mode. It would only set the current if a module ran standalone, and the firmware never enables a driver whose UART config failed. Still turn it to mid-range so that a module moved into another, standalone machine starts at a sane current.

---

## Board revision items

* **Pull-downs on the LED data inputs and the PWM output.** 10 kΩ to GND on U1 A0–A3 (pins 2–5) and on J2.5 / D8. Until `setup()` runs (ROM bootloader, then the core's ~0.9 s double-tap RGB fade), these pins float. Floating HCT inputs can clock random data into the SK6812s, at any brightness and outside the power budget, and can half-turn-on the light-bar MOSFET. Firmware cannot run earlier than this. Until the board is revised, power the strips **after** the Nano has booted (RGB LED green or red).
* **Tie the unused 74HCT245 inputs A4–A7 (pins 6–9) to GND.** They are floating CMOS inputs.
* **UART:** add a 1 kΩ in series from D1 to the UART_RX net, and leave the module TX pins unconnected (see Open item 1, option C).

## What "done" looks like

* Green LED at every boot, `d` clean on both drivers.
* All Stage 4–7 checks pass, with `b` product ≤ 0.20 in every mode and level.
* `LR_STEP_LIMIT` arc measured at ≤ 180°.
* Open items 1–3 closed, 4 photographed and measured, 5 either accepted (manual positioning) or replaced by StallGuard homing (tune `TMC_TCOOLTHRS`/`TMC_SGTHRS`).
