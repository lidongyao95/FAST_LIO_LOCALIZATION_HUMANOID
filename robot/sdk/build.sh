#!/usr/bin/env bash
# Build the two Unitree SDK programs on PC2. Building does not talk to the robot.
#   g1_navigation_sdk  forwards navigation velocity to the robot (used by the web page)
#   g1_fsm             reads or switches the locomotion mode (./g1 robot fsm)
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
[[ $(uname -s) == Linux && $(uname -m) == aarch64 ]] || { echo '请在 PC2（ARM64）上构建'; exit 1; }
sdk="$repo/.deps/src/unitree_sdk2"
out="${G1_DATA:-$HOME/ldy/g1_real_data}/bin"
[[ -f "$sdk/lib/aarch64/libunitree_sdk2.a" ]] || { echo '缺少 Unitree SDK2；先运行 ./g1 robot deploy sync'; exit 1; }
mkdir -p "$out"
for program in navigation:g1_navigation_sdk fsm:g1_fsm; do
    source_file="$repo/robot/sdk/${program%%:*}.cpp"
    target="$out/${program##*:}"
    env -u LD_LIBRARY_PATH g++ -O2 -std=c++17 -I"$sdk/include" -I"$sdk/thirdparty/include" \
        -I"$sdk/thirdparty/include/ddscxx" "$source_file" \
        -L"$sdk/lib/aarch64" -lunitree_sdk2 -L"$sdk/thirdparty/lib/aarch64" \
        -lddscxx -lddsc -lpthread -Wl,-rpath,"$sdk/thirdparty/lib/aarch64" \
        -o "$target.new"
    mv "$target.new" "$target"
    echo "已构建 $target"
done
