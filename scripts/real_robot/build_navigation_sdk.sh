#!/usr/bin/env bash
# Build only; never execute the hardware transport.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
[[ $(uname -s) == Linux && $(uname -m) == aarch64 ]] || { echo '请在 PC2 ARM64 本机构建'; exit 1; }
sdk="$repo/.deps/src/unitree_sdk2"
out="${G1_DATA:-$HOME/ldy/g1_real_data}/bin"
[[ -f "$sdk/lib/aarch64/libunitree_sdk2.a" ]] || { echo '缺少本机 Unitree SDK2'; exit 1; }
mkdir -p "$out"
env -u LD_LIBRARY_PATH g++ -O2 -std=c++17 -I"$sdk/include" -I"$sdk/thirdparty/include" \
  -I"$sdk/thirdparty/include/ddscxx" "$repo/scripts/real_robot/g1_navigation_sdk.cpp" \
  -L"$sdk/lib/aarch64" -lunitree_sdk2 -L"$sdk/thirdparty/lib/aarch64" \
  -lddscxx -lddsc -lpthread -Wl,-rpath,"$sdk/thirdparty/lib/aarch64" \
  -o "$out/g1_navigation_sdk.new"
mv "$out/g1_navigation_sdk.new" "$out/g1_navigation_sdk"
echo "已构建 $out/g1_navigation_sdk；未执行 SDK、未发送运动命令。"
