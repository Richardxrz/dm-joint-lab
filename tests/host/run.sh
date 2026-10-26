#!/usr/bin/env bash
# 主机侧规格测试：不需要板子、不需要 H7 工具链。退出码 0 = 全绿。
set -euo pipefail
cd "$(dirname "$0")/../.."

cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Debug >/dev/null
cmake --build build/host -j >/dev/null
./build/host/host_tests
