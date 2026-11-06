#!/usr/bin/env bash
# Host test for UpdateInputs() in kayak_working.ino.  Needs g++ and python3.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
ino="$here/../../kayak_working/kayak_working.ino"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
python3 -I - "$ino" "$out/ex.inc" <<'PY'
import sys
s = open(sys.argv[1]).read()
def between(a, b):
    i = s.index(a); j = s.index(b, i); return s[i:j]
parts = [
    between("struct DebouncedButton {", "// -------------------- LED HELPERS"),
    between("// -------------------- BUTTON INSTANCES",
            "// ============================================================================\n// END OF HELPERS + INPUT"),
]
open(sys.argv[2], "w").write("\n".join(parts))
PY
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -I"$out" "$here/input_test.cpp" -o "$out/test"
"$out/test"
