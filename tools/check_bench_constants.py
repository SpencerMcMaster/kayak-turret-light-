#!/usr/bin/env python3
"""
Check that every hardware constant copied into a bench test sketch
(bench_tests/*/*.ino) is identical to kayak_working/kayak_working.ino.

Compared: PIN_*, TMC_*, IMU_I2C_ADDR, LED_COUNT_*, LIGHTBAR_PWM_FREQ_HZ.
A PIN_* in a bench sketch that does not exist in the main firmware is an
error too. Exit status 1 on any mismatch.
"""
import glob
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MAIN = os.path.join(ROOT, "kayak_working", "kayak_working.ino")
DECL = re.compile(
    r"^\s*static\s+(?:const|constexpr)\s+[\w:]+\s+"
    r"(PIN_\w+|TMC_\w+|IMU_I2C_ADDR|LED_COUNT_\w+|LIGHTBAR_PWM_FREQ_HZ)\s*=\s*([^;]+);",
    re.M)


def consts(path):
    text = open(path, encoding="utf-8").read()
    out = {}
    for name, value in DECL.findall(text):
        out[name] = re.sub(r"\s+", "", value)
    return out


def main():
    ref = consts(MAIN)
    errors = 0
    checked = 0
    for sketch in sorted(glob.glob(os.path.join(ROOT, "bench_tests", "*", "*.ino"))):
        rel = os.path.relpath(sketch, ROOT)
        for name, value in sorted(consts(sketch).items()):
            if name in ref:
                checked += 1
                if ref[name] != value:
                    print(f"MISMATCH {rel}: {name} = {value}   (main: {ref[name]})")
                    errors += 1
            elif name.startswith("PIN_"):
                print(f"UNKNOWN  {rel}: {name} = {value} has no counterpart in the main firmware")
                errors += 1
    print(f"{checked} shared constants checked across bench sketches: "
          + ("ALL MATCH" if not errors else f"{errors} PROBLEM(S)"))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
