#!/usr/bin/env zsh
set -euo pipefail

ASAN=${ENABLE_ASAN:-OFF}
TSAN=${ENABLE_TSAN:-OFF}
UBSAN=${ENABLE_UBSAN:-ON}

if [[ "$(uname)" == "Darwin" ]]; then
    CC="$(xcrun -f clang)"
    CXX="$(xcrun -f clang++)"
    export CMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5
    export PKG_CONFIG_PATH=/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig
else
    CC="$(which clang)"
    CXX="$(which clang++)"
fi

rm -rf build && cmake -S . -B build \
    -DCMAKE_C_COMPILER="$CC" \
    -DCMAKE_CXX_COMPILER="$CXX" \
    -DENABLE_ASAN="$ASAN" \
    -DENABLE_TSAN="$TSAN" \
    -DENABLE_UBSAN="$UBSAN"

cmake --build build -- -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"
