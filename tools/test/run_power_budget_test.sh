#!/usr/bin/env bash
# Host test for EnforcePowerBudget()/ShowAllStrips() in kayak_working.ino.
# Usage: tools/test/run_power_budget_test.sh      (needs g++ and python3)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
ino="$here/../../kayak_working/kayak_working.ino"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
python3 -I - "$ino" "$out/extracted.inc" <<'PY'
import sys
s = open(sys.argv[1]).read()
def between(a, b):
    i = s.index(a); j = s.index(b, i); return s[i:j]
parts = [
    between("// Bytes per pixel, decoded", "// Global brightness limits"),
    between("static const float LED_POWER_BUDGET_MAX", "// ====="),
    between("// -------------------- POWER BUDGET STATE", "// -------------------- TURN / HAZARD STATE"),
    between("// -------------------- LED POWER BUDGET --------------------", "static uint8_t BrightnessFromIndex"),
]
open(sys.argv[2], "w").write("\n".join(parts))
PY
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-function -O1 -I"$out" "$here/power_budget_test.cpp" -o "$out/test"
"$out/test"
