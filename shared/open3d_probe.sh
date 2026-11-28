#!/usr/bin/env bash
# 构建前/运行前的 Open3D 本地库解析预检。
# 在链接之前发现 .deps 里解包的 Open3D 传递依赖（lapacke -> tmglib 等）
# 只存在于本地目录、系统 ldconfig 看不到的问题，避免编译到一半才失败。
#
# 用法：
#   shared/open3d_probe.sh [--check|--env|--fix] [--quiet] [仓库根]
#   --check  人读报告（默认）
#   --env    只输出可 eval 的赋值：OPEN3D_RUNTIME_LIB / OPEN3D_NEEDS_RPATH_LINK
#   --fix    对缺失的库 apt-get download + dpkg-deb -x 解包到 runtime 树（免 root）
# 退出码：0 全部可解析；1 存在缺失；2 未发现本地 Open3D 运行库
set -euo pipefail

mode=check
quiet=false
repo=
for arg in "$@"; do
    case "$arg" in
        --check) mode=check ;;
        --env) mode=env ;;
        --fix) mode=fix ;;
        --quiet) quiet=true ;;
        -h|--help)
            sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*)
            echo "未知参数：$arg" >&2
            exit 64 ;;
        *) repo="$arg" ;;
    esac
done
if [ -z "$repo" ]; then
    repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
fi
arch=$(uname -m)

lib_dirs=()
for candidate in \
    "$repo/.deps/arm-open3d/runtime/usr/lib/${arch}-linux-gnu" \
    "$repo/.deps/root/usr/lib/${arch}-linux-gnu" \
    "$repo/.deps/arm-open3d/open3d141/lib"; do
    [ -d "$candidate" ] && lib_dirs+=("$candidate")
done

say() { $quiet || printf '%s\n' "$*"; }
info() { printf '%s\n' "$*" >&2; }

if [ ${#lib_dirs[@]} -eq 0 ]; then
    if [ "$mode" = env ]; then
        printf 'OPEN3D_RUNTIME_LIB=%s\n' "''"
        printf 'OPEN3D_NEEDS_RPATH_LINK=0\n'
    else
        info "未发现本地 Open3D 运行库目录（检查 .deps/arm-open3d 或 .deps/root）。"
    fi
    exit 2
fi

ldconfig_bin=''
for candidate in "$(command -v ldconfig 2>/dev/null || true)" /sbin/ldconfig /usr/sbin/ldconfig; do
    if [ -n "$candidate" ] && [ -x "$candidate" ]; then
        ldconfig_bin=$candidate
        break
    fi
done

declare -A system_libs=()
if [ -n "$ldconfig_bin" ]; then
    while read -r name; do
        [ -n "$name" ] && system_libs["$name"]=1
    done < <("$ldconfig_bin" -p 2>/dev/null | awk '/=>/{print $1}')
fi
# 精简镜像或受限 PATH 下 ldconfig 可能不可用；此时扫描系统库目录，
# 否则 libc/libm/liblapack 这类系统库会被整体误判为缺失。
if [ ${#system_libs[@]} -eq 0 ]; then
    for dir in /lib/"$arch"-linux-gnu /usr/lib/"$arch"-linux-gnu /lib64 /usr/lib64; do
        [ -d "$dir" ] || continue
        while IFS= read -r path; do
            system_libs["$(basename -- "$path")"]=1
        done < <(find "$dir" -maxdepth 1 \( -type f -o -type l \) -name '*.so*' 2>/dev/null)
    done
fi

libs=()
for dir in "${lib_dirs[@]}"; do
    while IFS= read -r path; do
        libs+=("$path")
    done < <(find "$dir" -maxdepth 1 \( -type f -o -type l \) -name '*.so*' ! -name '*.a' 2>/dev/null | sort)
done

if [ ${#libs[@]} -eq 0 ]; then
    if [ "$mode" = env ]; then
        printf 'OPEN3D_RUNTIME_LIB=%s\n' "''"
        printf 'OPEN3D_NEEDS_RPATH_LINK=0\n'
    else
        info "本地 Open3D 目录存在但没有任何 .so（只有静态库或尚未解包）。"
    fi
    exit 2
fi

declare -A local_name=()
for path in "${libs[@]}"; do
    local_name["$(basename "$path")"]=1
done

missing=()
for path in "${libs[@]}"; do
    base=$(basename "$path")
    while IFS= read -r needed; do
        [ -n "$needed" ] || continue
        if [ -n "${local_name[$needed]:-}" ]; then continue; fi
        if [ -n "${system_libs[$needed]:-}" ]; then continue; fi
        missing+=("$needed (被 $base 依赖)")
    done < <(readelf -d "$path" 2>/dev/null | awk '/NEEDED/{gsub(/[][]/, "", $5); print $5}')
done

runtime_dir=${lib_dirs[0]}

pkg_for_lib() {
    case "$1" in
        libtmglib.so.*) echo libtmglib3 ;;
        liblapacke.so.*) echo liblapacke ;;
        liblapack.so.*) echo liblapack3 ;;
        libblas.so.*) echo libblas3 ;;
        liblzf.so.*) echo liblzf1 ;;
        libstb.so.*) echo libstb0 ;;
        libtinyobjloader.so.*) echo libtinyobjloader1 ;;
        libfilament.so.*) echo libfilament1.9 ;;
        *) echo ;;
    esac
}

if [ ${#missing[@]} -gt 0 ]; then
    if [ "$mode" = env ]; then
        printf 'OPEN3D_RUNTIME_LIB=%q\n' "$runtime_dir"
        printf 'OPEN3D_NEEDS_RPATH_LINK=1\n'
    fi
    info "Open3D 本地库存在未解析的传递依赖："
    for item in "${missing[@]}"; do
        info "  - $item"
    done
    packages=()
    for item in "${missing[@]}"; do
        lib=${item%% *}
        package=$(pkg_for_lib "$lib")
        [ -n "$package" ] && packages+=("$package")
    done
    if [ ${#packages[@]} -gt 0 ]; then
        info "缺少的库对应软件包：${packages[*]}"
        if [ "$mode" = fix ]; then
            tmp=$(mktemp -d)
            trap 'rm -rf -- "$tmp"' EXIT
            info "解包到 $runtime_dir（免 root）："
            (cd "$tmp" && apt-get download "${packages[@]}") || {
                info "apt-get download 失败；检查网络与软件源。"
                exit 1
            }
            mkdir -p "$runtime_dir"
            for deb in "$tmp"/*.deb; do
                dpkg-deb -x "$deb" "$tmp/root"
            done
            cp -a "$tmp/root/usr/lib/${arch}-linux-gnu/." "$runtime_dir"/
            info "已解包；重新执行本预检确认。"
            exit 0
        fi
        info "修复（免 root，下载并解包到本地 runtime 树）："
        info "  bash shared/open3d_probe.sh --fix $repo"
    else
        info "缺失库无法自动映射到软件包；自行 apt-get download 后 dpkg-deb -x 到："
        info "  $runtime_dir"
    fi
    exit 1
fi

if [ "$mode" = env ]; then
    printf 'OPEN3D_RUNTIME_LIB=%q\n' "$runtime_dir"
    printf 'OPEN3D_NEEDS_RPATH_LINK=1\n'
    exit 0
fi

say "Open3D 本地库目录：${lib_dirs[*]}"
say "共享库 ${#libs[@]} 个；传递依赖全部可解析。"
say "构建时固定加：-Wl,-rpath-link,$runtime_dir"
say "运行定位节点前该目录需在 LD_LIBRARY_PATH 中（shared/env.sh 已包含）。"
exit 0
