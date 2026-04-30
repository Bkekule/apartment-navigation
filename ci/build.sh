#!/usr/bin/env zsh
set -euo pipefail

ASAN=${ENABLE_ASAN:-OFF}
TSAN=${ENABLE_TSAN:-OFF}
UBSAN=${ENABLE_UBSAN:-ON}


rm -rf build && cmake -S . -B build \
    -DENABLE_ASAN="$ASAN" \
    -DENABLE_TSAN="$TSAN" \
    -DENABLE_UBSAN="$UBSAN"

cmake --build build -- -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"
