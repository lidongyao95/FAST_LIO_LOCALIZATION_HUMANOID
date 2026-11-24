#!/usr/bin/env bash
workflow=scan-preview
source "$(dirname -- "${BASH_SOURCE[0]}")/../workflows/lib.sh"
source "$repo/scripts/simulation/session.sh"

usage() {
    cat <<'EOF'
用法：scan_preview.sh SCENE_DIR [导航参数]
使用实机扫描预览工具生成的 scene.xml、map.ply 和 map.yaml。
启动同一套 MuJoCo 导航会话。生成场景见 g1_real_robot_docs/scan_mujoco_preview.md。
EOF
    navigation_options
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
[[ "$1" != --* ]] || usage_error "请传入扫描场景目录"
map_dir=$(absolute "$1")
shift
model_path="$map_dir/scene.xml"
initial_z=0.793
parse_navigation "$@"
run_navigation
