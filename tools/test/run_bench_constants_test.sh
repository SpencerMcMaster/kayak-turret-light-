#!/usr/bin/env bash
# Bench sketches must use the same pins/constants as the main firmware.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
python3 -I "$here/../check_bench_constants.py"
