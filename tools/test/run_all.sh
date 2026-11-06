#!/usr/bin/env bash
# Run every host test; exits non-zero if any fails.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
for t in "$here"/run_*_test.sh; do
  echo "=== $(basename "$t")"
  "$t"
done
echo "=== all host tests passed"
