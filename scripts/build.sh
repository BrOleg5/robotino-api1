#!/usr/bin/env bash
# Builds and tests inside the development container (see compose.yaml).
# Usage: scripts/build.sh [gcc|clang|mingw|python|all]   (default: gcc)
#   gcc, clang  native build + ctest
#   mingw       Windows cross-build with MinGW-w64, tests run under Wine
#   python      builds the Python module and runs its tests
# From the host: docker compose run --rm dev scripts/build.sh all
set -euo pipefail
cd "$(dirname "$0")/.."

native() {
    local name=$1 cc=$2 cxx=$3
    cmake -S . -B "build/$name" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER="$cc" -DCMAKE_CXX_COMPILER="$cxx"
    cmake --build "build/$name"
    ctest --test-dir "build/$name" --output-on-failure
}

mingw() {
    cmake -S . -B build/mingw -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64.cmake
    cmake --build build/mingw
    ctest --test-dir build/mingw --output-on-failure
}

python_module() {
    pip install --no-build-isolation --no-deps -q . 2>&1 | tail -n 20
    python -m pytest -q python/tests
}

case "${1:-gcc}" in
    gcc) native gcc gcc g++ ;;
    clang) native clang clang clang++ ;;
    mingw) mingw ;;
    python) python_module ;;
    all) native gcc gcc g++; native clang clang clang++; mingw; python_module ;;
    *) echo "unknown target: $1" >&2; exit 2 ;;
esac
