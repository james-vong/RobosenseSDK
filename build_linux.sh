#!/usr/bin/env bash
# Build rs_driver (patched) + our demos on Linux. Needs: gcc/g++, cmake >= 3.16, make, libudev-dev.
# Verified on Amlogic MB41, Debian 13 arm64 (gcc 14.2, cmake 3.31), 2026-11-18.
set -euo pipefail
cd "$(dirname "$0")/rs_driver"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_USB=ON -DCOMPILE_DEMOS=ON -DDISABLE_PCAP_PARSE=ON
cmake --build build -j"$(nproc)"
ls -la build/demo/demo_*
