# Bench test sketches

Standalone Arduino programs, each testing **one** subsystem of the kayak turret
PCB. None depends on the others or on `kayak_working.ino`. Flash one, test,
then flash the next.

Every sketch starts by putting **all** outputs in a safe state, whatever it
tests: both TMC2209 drivers disabled (~EN HIGH), all four LED data lines LOW,
light-bar PWM LOW. A test for one subsystem cannot wake another.

## Build

Same setup as the main firmware: board **Arduino Nano ESP32**, core
**esp32 2.0.17**, **Tools → Pin Numbering → "By GPIO number (legacy)"**. Each
sketch has an `#error` guard against the default numbering and a
`sketch.yaml`, so this works from the CLI:

```
arduino-cli compile bench_tests/test_motor
arduino-cli upload  -p <port> bench_tests/test_motor
```

In the IDE, open the `.ino` inside its folder. Serial Monitor: **115200 baud**.
Line ending **Newline** (commands with a number, such as `d 128`, need Enter;
single-key commands act immediately). `?` prints each sketch's menu.

## Recommended order

| # | Sketch | Tests | Power needed | Pass looks like |
|---|---|---|---|---|
| 1 | `test_buttons` | J6 buttons, debounce, what the main firmware would do with each combination | Nano on the PCB; USB power is enough | One `pressed` and one `released` line per press; correct name per J6 terminal; no stuck-LOW warning at boot |
| 2 | `test_imu` | I²C scan, WHO_AM_I, config read-back, raw and filtered angles | 12 V (GY-521 VCC is on +5 V). **Measure the SDA/SCL pull-up voltage first** (checklist Stage 2.4) | One device at 0x68; WHO_AM_I 0x68; config PASS; flat and still → accel ≈ 0, 0, +1.00 g |
| 3 | `test_tmc_uart` | Whether D1/TX reaches PDN_UART (the R10 issue), driver replies at addresses 0 and 1, chip version, a write acknowledged. **Never enables a driver** | 12 V (TMC logic runs from VS); motors optional | `TX path ok \| UP/DOWN PASS \| LEFT/RIGHT PASS`, LED green |
| 4 | `test_motor` | **One** motor: verified UART config, enable, small moves (±800 steps = ¼ rev max), run/hold current, health check | 12 V. Set `TEST_AXIS` 0 = lift, 1 = pan | `config ... VERIFIED`, moves both ways, `cs_actual` ≈ 25 moving / ≈ 7 held |
| 5 | `test_lightbar` | D8 PWM: duty, the four brightness-level duties, ramp, frequency | First nothing on J2.5 (DMM only), then the MOSFET module | DMM ≈ duty/255 × 3.3 V DC, ≈ 5 kHz |
| 6 | `test_led_strip` | **One** strand: count (step, ends, overrun), direction, colour order, every channel, with the 0.20 power budget | Strand 12 V on **after** the Nano boots. Set `TEST_STRIP` 0–3 | Last LED = configured count, overrun pixels dark, `m` shows R G B W |

Then flash `kayak_working` and continue with `BRINGUP_CHECKLIST.md` Stage 3 onward.

## Picking the axis or strand

`test_motor`: edit `#define TEST_AXIS` (0 = UP/DOWN lift on J3, 1 = LEFT/RIGHT pan on J4).
`test_led_strip`: edit `#define TEST_STRIP` (0 Bottom Left, 1 Bottom Right, 2 Cockpit Front, 3 Cockpit Back).
Both can also be set from the CLI without editing:
`arduino-cli compile --build-property "compiler.cpp.extra_flags=-DTEST_AXIS=1" bench_tests/test_motor`

## Safety rules kept in the bench sketches

* `test_motor`: run current 800 mA RMS, IHOLD 30 %, VREF trimpot ignored. The driver is enabled only after the IFCNT count and register read-back verify. A 250 ms health check disables it on a 12 V dropout or reset. Moves are clamped to ±800 steps around the power-on position. Only the selected driver is ever enabled.
* `test_led_strip`: lit fraction × brightness ≤ 0.20 before every `show()`, computed over the one strand alone, which is stricter than the main firmware.
* `test_tmc_uart`, `test_buttons`, `test_imu`, `test_lightbar`: never enable a driver.

## Keeping them in sync

Each sketch copies only the pin definitions and constants it needs, with the
same net comments. `tools/check_bench_constants.py` (also run by
`tools/test/run_all.sh`) fails if any copied `PIN_*`, `TMC_*`, LED count, I²C
address or PWM frequency differs from `kayak_working.ino`. Re-run it after any
schematic or pin change.
