#!/usr/bin/env bash
# ============================================================================
# record_bag.sh — 录制 MID-360 雷达 rosbag
#
# 用法:
#   ./scripts/record_bag.sh [选项]
#
# 选项:
#   -d, --duration SEC    录制时长，默认 30 秒
#   -o, --output DIR      输出目录，默认 data/record files/mid360_<timestamp>
#   -t, --topics "t1 t2"  录制的话题，默认 "/livox/lidar /livox/imu"
#   -f, --format FMT      xfer_format: 0=PointCloud2, 1=CustomMsg，默认 1
#   --no-driver           不启动驱动（雷达已在运行）
#   -h, --help            显示帮助
#
# 示例:
#   ./scripts/record_bag.sh                         # 默认录制 30 秒
#   ./scripts/record_bag.sh -d 60                   # 录制 60 秒
#   ./scripts/record_bag.sh -d 10 -o my_bag         # 录制 10 秒到指定目录
#   ./scripts/record_bag.sh --no-driver             # 手动启动雷达后录制
# ============================================================================

set -euo pipefail

# --- 默认配置 ---
DURATION=30
OUTPUT_DIR=""
TOPICS="/livox/lidar /livox/imu"
XFER_FORMAT=1
AUTO_DRIVER=true
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# --- 路径 ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/cod_-rm2026_-navigation-master"
BAGS_DIR="$PROJECT_DIR/data/record files"
LOG_DIR="$WS_DIR/log"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# --- 函数 ---
log_info()  { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S') $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S') $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $*"; }
log_step()  { echo -e "${BLUE}[STEP]${NC}  $(date '+%H:%M:%S') $*"; }

usage() {
    sed -n '/^# 用法:/,/^# =/{/^# /s/^# //p}' "$0"
    exit 0
}

cleanup() {
    log_info "正在清理..."
    if [[ -n "${DRIVER_PID:-}" ]] && kill -0 "$DRIVER_PID" 2>/dev/null; then
        log_info "停止雷达驱动 (PID: $DRIVER_PID)..."
        kill "$DRIVER_PID" 2>/dev/null || true
        wait "$DRIVER_PID" 2>/dev/null || true
    fi
    pkill -f livox_ros_driver2_node 2>/dev/null || true
    log_info "清理完成"
}

trap 'cleanup; exit 130' INT TERM
trap 'cleanup' EXIT

# --- 解析参数 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--duration)  DURATION="$2"; shift 2 ;;
        -o|--output)    OUTPUT_DIR="$2"; shift 2 ;;
        -t|--topics)    TOPICS="$2"; shift 2 ;;
        -f|--format)    XFER_FORMAT="$2"; shift 2 ;;
        --no-driver)    AUTO_DRIVER=false; shift ;;
        -h|--help)      usage ;;
        *)              log_error "未知选项: $1"; usage ;;
    esac
done

# --- 设置默认输出目录 ---
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$BAGS_DIR/mid360_${TIMESTAMP}"
fi

# --- 前置检查 ---
log_step "=== Livox MID-360 Rosbag 录制 ==="
log_info "录制时长:   ${DURATION}s"
log_info "输出目录:   ${OUTPUT_DIR}"
log_info "话题:       ${TOPICS}"
log_info "xfer_format: ${XFER_FORMAT}"
log_info "自动启动驱动: ${AUTO_DRIVER}"

# 检查 ROS2
if ! command -v ros2 &>/dev/null; then
    log_info "正在 source ROS2 环境..."
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        set +u  # ROS2 setup.bash references unset vars
        source /opt/ros/humble/setup.bash
        set -u
    else
        log_error "找不到 ROS2 Humble 安装，请手动 source"
        exit 1
    fi
fi

# 检查工作空间
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
    log_error "工作空间未编译: $WS_DIR"
    log_error "请先运行: cd $WS_DIR && colcon build"
    exit 1
fi
set +u; source "$WS_DIR/install/setup.bash"; set -u

# 创建输出目录
mkdir -p "$BAGS_DIR" "$LOG_DIR"

# --- 启动驱动 ---
if $AUTO_DRIVER; then
    log_step "启动雷达驱动..."

    # 检查雷达连通性
    if ping -c 1 -W 2 192.168.1.181 &>/dev/null; then
        log_info "MID-360 雷达可达 (192.168.1.181)"
    else
        log_warn "MID-360 雷达不可达，请检查网络连接"
        log_warn "继续尝试启动驱动..."
    fi

    LAUNCH_FILE="msg_MID360_launch.py"
    ros2 launch livox_ros_driver2 "$LAUNCH_FILE" \
        > "$LOG_DIR/driver_${TIMESTAMP}.log" 2>&1 &
    DRIVER_PID=$!
    log_info "驱动已启动 (PID: $DRIVER_PID)"

    # 等待话题出现
    log_info "等待雷达话题就绪..."
    WAIT_SEC=0
    MAX_WAIT=15
    while [[ $WAIT_SEC -lt $MAX_WAIT ]]; do
        if ros2 topic list 2>/dev/null | grep -q "/livox/lidar"; then
            log_info "雷达话题已就绪 (/livox/lidar)"
            break
        fi
        sleep 1
        WAIT_SEC=$((WAIT_SEC + 1))
        echo -n "."
    done
    echo

    if [[ $WAIT_SEC -ge $MAX_WAIT ]]; then
        log_error "超时: 雷达话题未在 ${MAX_WAIT}s 内出现"
        log_error "检查日志: $LOG_DIR/driver_${TIMESTAMP}.log"
        exit 1
    fi
else
    log_step "跳过驱动启动 (--no-driver)"
    if ! ros2 topic list 2>/dev/null | grep -q "/livox/lidar"; then
        log_error "未检测到 /livox/lidar 话题，请先启动雷达驱动"
        exit 1
    fi
    log_info "已检测到 /livox/lidar 话题"
fi

# --- 录制 ---
log_step "开始录制 ${DURATION}s ..."
log_info "话题: ${TOPICS}"
log_info "按 Ctrl+C 提前停止"

BAG_TEMP="$BAGS_DIR/.tmp_${TIMESTAMP}"

set +e
# Note: ros2 bag record -d controls per-file duration (splitting), NOT total time.
# Use timeout(1) to limit total recording duration.
# --max-bag-size 0 disables size-based file splitting.
timeout "$DURATION" ros2 bag record \
    -o "$BAG_TEMP" \
    --max-bag-size 0 \
    $TOPICS \
    2>&1 | tee "$LOG_DIR/record_${TIMESTAMP}.log"
RECORD_EXIT=$?
# timeout returns 124 if the command timed out (expected — means recording completed)
if [[ $RECORD_EXIT -eq 124 ]]; then
    RECORD_EXIT=0
fi
set -e

# 移动临时目录到目标位置
if [[ -d "$BAG_TEMP" ]]; then
    mv "$BAG_TEMP" "$OUTPUT_DIR"
fi

# --- 结果 ---
echo
if [[ -d "$OUTPUT_DIR" ]] && [[ -f "$OUTPUT_DIR/metadata.yaml" ]]; then
    log_step "=== 录制完成 ==="
    log_info "输出目录: ${OUTPUT_DIR}"

    echo
    log_info "--- Bag 信息 ---"
    set +u; source /opt/ros/humble/setup.bash 2>/dev/null || true; set -u
    set +u; source "$WS_DIR/install/setup.bash" 2>/dev/null || true; set -u
    ros2 bag info "$OUTPUT_DIR" 2>/dev/null || true

    echo
    log_info "--- 文件大小 ---"
    du -sh "$OUTPUT_DIR"

    echo
    log_info "回放命令:"
    echo -e "  ${GREEN}source /opt/ros/humble/setup.bash${NC}"
    echo -e "  ${GREEN}source ${WS_DIR}/install/setup.bash${NC}"
    echo -e "  ${GREEN}ros2 bag play ${OUTPUT_DIR}${NC}"

    echo
    log_info "录制成功!"
else
    log_error "录制失败: 未生成有效的 bag 文件"
    log_error "检查日志: $LOG_DIR/record_${TIMESTAMP}.log"
    exit 1
fi
