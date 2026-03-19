#!/bin/bash
#
# Adaptive-LIO SLAM SDK 打包脚本
#
# 在编译完成后运行此脚本，自动收集二进制、配置、库文件并打包为可分发的 SDK。
# 支持 x86_64 和 aarch64 (ARM/Jetson) 架构，自动检测当前平台。
#
# 前置条件:
#   1. 已完成 colcon build 编译
#   2. 已安装 livox_ros_driver2 (默认 ~/livox_ws)
#
# 用法:
#   ./pack_sdk.sh                          # 默认输出到 ./sdk_<arch>/
#   ./pack_sdk.sh -o /home/user/my_sdk     # 指定输出目录
#   ./pack_sdk.sh --tar                    # 打包后额外生成 .tar.gz 压缩包
#   ./pack_sdk.sh -o ~/sdk --tar           # 指定目录 + 生成压缩包
#

set -e

# ==================== 颜色输出 ====================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "${CYAN}[STEP]${NC} $1"; }

# ==================== 路径检测 ====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 自动检测项目根目录 (pack_sdk.sh 应在项目根目录或 src/adaptive_lio/bash/ 下)
if [[ -d "$SCRIPT_DIR/src/adaptive_lio" ]]; then
    WS_DIR="$SCRIPT_DIR"
elif [[ -d "$SCRIPT_DIR/../../../src/adaptive_lio" ]]; then
    WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
elif [[ -d "$SCRIPT_DIR/../../src/adaptive_lio" ]]; then
    WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
else
    log_error "无法定位项目根目录，请在项目根目录下运行此脚本"
    exit 1
fi

PKG_DIR="$WS_DIR/src/adaptive_lio"
BUILD_DIR="$WS_DIR/build/adaptive_lio"
INSTALL_DIR="$WS_DIR/install/adaptive_lio"

# 架构检测
ARCH="$(uname -m)"
case "$ARCH" in
    x86_64)  ARCH_LABEL="x86_64" ;;
    aarch64) ARCH_LABEL="arm64" ;;
    *)       ARCH_LABEL="$ARCH" ;;
esac

# ==================== 参数解析 ====================
OUTPUT_DIR=""
CREATE_TAR=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --tar)
            CREATE_TAR=true
            shift
            ;;
        -h|--help)
            echo "Adaptive-LIO SDK 打包脚本"
            echo ""
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  -o, --output DIR   指定 SDK 输出目录 (默认: ./sdk_<arch>)"
            echo "  --tar              打包后生成 .tar.gz 压缩包"
            echo "  -h, --help         显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                         # 输出到 ./sdk_arm64/"
            echo "  $0 -o ~/my_sdk             # 输出到 ~/my_sdk/"
            echo "  $0 --tar                   # 输出并生成 tar.gz"
            echo "  $0 -o /tmp/release --tar   # 指定目录 + 压缩包"
            exit 0
            ;;
        *)
            log_error "未知选项: $1"
            exit 1
            ;;
    esac
done

# 默认输出目录
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$WS_DIR/sdk_${ARCH_LABEL}"
fi

# ==================== 环境检查 ====================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Adaptive-LIO SDK 打包工具${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "  项目目录:  $WS_DIR"
echo -e "  系统架构:  $ARCH ($ARCH_LABEL)"
echo -e "  输出目录:  $OUTPUT_DIR"
echo ""

# 检查编译产物
log_step "检查编译产物..."

BINARY_PATH=""
if [[ -x "$BUILD_DIR/adaptive_lio_node" ]]; then
    BINARY_PATH="$BUILD_DIR/adaptive_lio_node"
elif [[ -x "$INSTALL_DIR/lib/adaptive_lio/adaptive_lio_node" ]]; then
    # install/ 下可能是符号链接，解析到真实文件
    BINARY_PATH="$(readlink -f "$INSTALL_DIR/lib/adaptive_lio/adaptive_lio_node")"
else
    log_error "未找到编译好的 adaptive_lio_node"
    log_error "请先编译: colcon build --symlink-install"
    exit 1
fi

BINARY_ARCH="$(file "$BINARY_PATH" | grep -oP '(x86-64|aarch64|ARM aarch64)')"
log_info "找到二进制: $BINARY_PATH"
log_info "二进制架构: $BINARY_ARCH"

if [[ "$ARCH" == "x86_64" && "$BINARY_ARCH" != *"x86-64"* ]]; then
    log_warn "二进制架构与当前系统不匹配!"
elif [[ "$ARCH" == "aarch64" && "$BINARY_ARCH" != *"aarch64"* && "$BINARY_ARCH" != *"ARM"* ]]; then
    log_warn "二进制架构与当前系统不匹配!"
fi

# 检查配置文件
CONFIG_DIR="$PKG_DIR/config"
if [[ ! -d "$CONFIG_DIR" ]]; then
    log_error "配置目录不存在: $CONFIG_DIR"
    exit 1
fi

REQUIRED_CONFIGS=("mapping_m.yaml" "mapping_orin_nx.yaml" "mapping_orin_nano.yaml" "adaptive_lio.rviz")
for cfg in "${REQUIRED_CONFIGS[@]}"; do
    if [[ ! -f "$CONFIG_DIR/$cfg" ]]; then
        log_error "缺少配置文件: $CONFIG_DIR/$cfg"
        exit 1
    fi
done
log_info "配置文件检查通过 (${#REQUIRED_CONFIGS[@]} 个)"

# 检查 livox_ros_driver2 库
LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
LIVOX_LIB_DIR="$LIVOX_WS/install/livox_ros_driver2/lib"
if [[ ! -d "$LIVOX_LIB_DIR" ]]; then
    log_error "未找到 livox_ros_driver2 库目录: $LIVOX_LIB_DIR"
    log_error "请设置 LIVOX_WS_PATH 环境变量指向 livox 工作空间"
    exit 1
fi

LIVOX_SO_COUNT=$(ls "$LIVOX_LIB_DIR"/liblivox_ros_driver2*.so 2>/dev/null | wc -l)
if [[ $LIVOX_SO_COUNT -eq 0 ]]; then
    log_error "livox_ros_driver2 库文件不存在"
    exit 1
fi
log_info "livox_ros_driver2 库检查通过 ($LIVOX_SO_COUNT 个 .so)"

echo ""
log_step "开始打包..."

# ==================== 创建目录结构 ====================
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"/{bin,lib,config,launch,scripts,share/adaptive_lio/{config,launch,environment},share/livox_ros_driver2,share/ament_index/resource_index/packages,map}

# ==================== 复制文件 ====================

# 1. 二进制文件 (复制实际文件，不复制符号链接)
log_info "复制二进制文件..."
cp "$(readlink -f "$BINARY_PATH")" "$OUTPUT_DIR/bin/adaptive_lio_node"
chmod +x "$OUTPUT_DIR/bin/adaptive_lio_node"

# 2. 配置文件
log_info "复制配置文件..."
for cfg in "${REQUIRED_CONFIGS[@]}"; do
    cp "$CONFIG_DIR/$cfg" "$OUTPUT_DIR/config/"
done

# 3. Launch 文件
log_info "复制 Launch 文件..."
if [[ -f "$PKG_DIR/launch/run.launch.py" ]]; then
    cp "$PKG_DIR/launch/run.launch.py" "$OUTPUT_DIR/launch/"
fi

# 4. 脚本
log_info "复制脚本文件..."
[[ -f "$PKG_DIR/bash/run_slam.sh" ]]           && cp "$PKG_DIR/bash/run_slam.sh" "$OUTPUT_DIR/scripts/"
[[ -f "$PKG_DIR/scripts/gui_launcher.py" ]]     && cp "$PKG_DIR/scripts/gui_launcher.py" "$OUTPUT_DIR/scripts/"
[[ -f "$PKG_DIR/scripts/start_gui.sh" ]]        && cp "$PKG_DIR/scripts/start_gui.sh" "$OUTPUT_DIR/scripts/"
[[ -f "$PKG_DIR/scripts/run_adaptive_lio.sh" ]] && cp "$PKG_DIR/scripts/run_adaptive_lio.sh" "$OUTPUT_DIR/scripts/"
chmod +x "$OUTPUT_DIR/scripts/"*.sh 2>/dev/null || true

# 5. Livox 库文件
log_info "复制 livox_ros_driver2 库..."
cp "$LIVOX_LIB_DIR"/liblivox_ros_driver2*.so "$OUTPUT_DIR/lib/"

# 6. ROS2 share 元数据 (让 ament_index 能发现包)
log_info "复制 ROS2 包元数据..."
echo "ament_cmake" > "$OUTPUT_DIR/share/ament_index/resource_index/packages/adaptive_lio"
cp "$OUTPUT_DIR/config/"* "$OUTPUT_DIR/share/adaptive_lio/config/"
[[ -f "$OUTPUT_DIR/launch/run.launch.py" ]] && cp "$OUTPUT_DIR/launch/run.launch.py" "$OUTPUT_DIR/share/adaptive_lio/launch/"
if [[ -f "$INSTALL_DIR/share/adaptive_lio/package.xml" ]]; then
    cp "$INSTALL_DIR/share/adaptive_lio/package.xml" "$OUTPUT_DIR/share/adaptive_lio/"
elif [[ -f "$PKG_DIR/package.xml" ]]; then
    cp "$PKG_DIR/package.xml" "$OUTPUT_DIR/share/adaptive_lio/"
fi

# 复制 ament 环境钩子
if [[ -d "$INSTALL_DIR/share/adaptive_lio/environment" ]]; then
    cp "$INSTALL_DIR/share/adaptive_lio/environment/"* "$OUTPUT_DIR/share/adaptive_lio/environment/" 2>/dev/null || true
fi

# 复制 livox 消息定义
LIVOX_MSG_DIR="$LIVOX_WS/install/livox_ros_driver2/share/livox_ros_driver2/msg"
if [[ -d "$LIVOX_MSG_DIR" ]]; then
    cp -r "$LIVOX_MSG_DIR" "$OUTPUT_DIR/share/livox_ros_driver2/"
fi

# ==================== 生成 run.sh ====================
log_info "生成 SDK 启动脚本 (run.sh)..."
cat > "$OUTPUT_DIR/run.sh" << 'RUNSCRIPT_EOF'
#!/bin/bash
#
# Adaptive-LIO SLAM SDK 启动脚本
# 无需 colcon 工作空间，直接运行预编译的二进制文件
#
# 用法: ./run.sh [选项]
#   --rviz / --no-rviz      启用/禁用 RViz2 可视化 (默认启用)
#   --orin                  Orin NX 优化配置 (16GB)
#   --orin-nano             Orin Nano 极限优化配置 (8GB)
#   --driver                启动 Livox MID360 驱动
#   --record-bag [PATH]     录制 rosbag
#   --config FILE           使用自定义配置文件
#   --map-path PATH         地图保存路径 (默认: sdk/map/)
#   --bag PATH              播放 rosbag
#   --rate RATE             播放速率 (默认: 1.0)
#   -h, --help              显示帮助
#

set -e

SDK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_BIN="$SDK_DIR/bin"
SDK_LIB="$SDK_DIR/lib"
SDK_CONFIG="$SDK_DIR/config"
SDK_MAP="$SDK_DIR/map"

USE_RVIZ=true
MAP_PATH=""
BAG_PATH=""
BAG_RATE="1.0"
USE_ORIN=false
USE_ORIN_NANO=false
USE_DRIVER=false
RECORD_BAG=false
RECORD_BAG_PATH=""
CUSTOM_CONFIG=""
CONFIG_BACKUP=""

CONFIG_FILE="$SDK_CONFIG/mapping_m.yaml"
CONFIG_ORIN="$SDK_CONFIG/mapping_orin_nx.yaml"
CONFIG_ORIN_NANO="$SDK_CONFIG/mapping_orin_nano.yaml"

DRIVER_PID=""
RVIZ_PID=""
BAG_PID=""
RECORD_PID=""
NODE_PID=""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

show_help() {
    echo "Adaptive-LIO SLAM SDK 启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --rviz              启动 RViz2 可视化 (默认)"
    echo "  --no-rviz           不启动 RViz2"
    echo "  --orin              使用 Orin NX 优化配置 (16GB)"
    echo "  --orin-nano         使用 Orin Nano 极限优化配置 (8GB/低内存)"
    echo "  --driver            启动 Livox MID360 驱动 (实时模式需要)"
    echo "  --record-bag [PATH] 录制 rosbag (可选指定保存目录)"
    echo "  --config FILE       使用指定的配置文件"
    echo "  --map-path PATH     指定地图保存路径 (默认: sdk/map/)"
    echo "  --bag PATH          播放指定的 rosbag"
    echo "  --rate RATE         rosbag 播放速率 (默认: 1.0)"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --orin --driver --no-rviz --map-path ~/map"
    echo "  $0 --orin-nano --driver --no-rviz --map-path ~/map"
    echo "  $0 --bag /path/to/rosbag --rate 1.5"
    echo "  $0 --driver --record-bag ~/bags --map-path ~/map"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --rviz)         USE_RVIZ=true; shift ;;
        --no-rviz)      USE_RVIZ=false; shift ;;
        --orin)         USE_ORIN=true; shift ;;
        --orin-nano)    USE_ORIN_NANO=true; shift ;;
        --driver)       USE_DRIVER=true; shift ;;
        --record-bag)
            RECORD_BAG=true
            if [[ $# -ge 2 && "$2" != --* ]]; then
                RECORD_BAG_PATH="$2"; shift 2
            else
                shift
            fi
            ;;
        --config)       CUSTOM_CONFIG="$2"; shift 2 ;;
        --map-path)     MAP_PATH="$2"; shift 2 ;;
        --bag)          BAG_PATH="$2"; shift 2 ;;
        --rate)         BAG_RATE="$2"; shift 2 ;;
        -h|--help)      show_help; exit 0 ;;
        *)              log_error "未知选项: $1"; show_help; exit 1 ;;
    esac
done

if [[ -n "$CUSTOM_CONFIG" ]]; then
    [[ ! -f "$CUSTOM_CONFIG" ]] && { log_error "配置文件不存在: $CUSTOM_CONFIG"; exit 1; }
    CONFIG_FILE="$CUSTOM_CONFIG"
    log_info "使用自定义配置: $CONFIG_FILE"
elif [[ "$USE_ORIN_NANO" == true ]]; then
    [[ ! -f "$CONFIG_ORIN_NANO" ]] && { log_error "配置不存在: $CONFIG_ORIN_NANO"; exit 1; }
    CONFIG_FILE="$CONFIG_ORIN_NANO"
    log_info "使用 Orin Nano 极限优化配置"
elif [[ "$USE_ORIN" == true ]]; then
    [[ ! -f "$CONFIG_ORIN" ]] && { log_error "配置不存在: $CONFIG_ORIN"; exit 1; }
    CONFIG_FILE="$CONFIG_ORIN"
    log_info "使用 Orin NX 优化配置"
fi

cleanup() {
    log_info "正在清理..."
    [[ -n "$CONFIG_BACKUP" && -f "$CONFIG_BACKUP" ]] && { cp "$CONFIG_BACKUP" "$CONFIG_FILE"; rm -f "$CONFIG_BACKUP"; }
    [[ -n "$DRIVER_PID" ]] && kill -0 $DRIVER_PID 2>/dev/null && kill $DRIVER_PID 2>/dev/null || true
    [[ -n "$RVIZ_PID" ]] && kill -0 $RVIZ_PID 2>/dev/null && kill $RVIZ_PID 2>/dev/null || true
    [[ -n "$BAG_PID" ]] && kill -0 $BAG_PID 2>/dev/null && kill $BAG_PID 2>/dev/null || true
    if [[ -n "$RECORD_PID" ]] && kill -0 $RECORD_PID 2>/dev/null; then
        log_info "停止 rosbag 录制..."
        kill -INT $RECORD_PID 2>/dev/null || true
        for i in $(seq 1 10); do kill -0 $RECORD_PID 2>/dev/null || break; sleep 1; done
        kill -0 $RECORD_PID 2>/dev/null && kill -9 $RECORD_PID 2>/dev/null || true
        log_info "rosbag 录制已保存"
    fi
    if [[ -n "$NODE_PID" ]] && kill -0 $NODE_PID 2>/dev/null; then
        kill -INT $NODE_PID 2>/dev/null || true
        log_info "等待节点完成地图保存 (最多120秒)..."
        for i in $(seq 1 120); do kill -0 $NODE_PID 2>/dev/null || break; sleep 1; done
        if kill -0 $NODE_PID 2>/dev/null; then
            log_warn "节点未能正常退出，强制终止"
            kill -9 $NODE_PID 2>/dev/null || true
        fi
    fi
    log_info "清理完成"
}
trap cleanup EXIT INT TERM

check_environment() {
    log_info "检查运行环境..."
    if ! command -v ros2 &>/dev/null; then
        log_error "未找到 ros2 命令，请先: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    if [[ ! -x "$SDK_BIN/adaptive_lio_node" ]]; then
        log_error "未找到可执行文件: $SDK_BIN/adaptive_lio_node"
        exit 1
    fi
    export LD_LIBRARY_PATH="$SDK_LIB:${LD_LIBRARY_PATH:-}"
    export AMENT_PREFIX_PATH="$SDK_DIR:${AMENT_PREFIX_PATH:-}"
    LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
    if [[ -f "$LIVOX_WS/install/setup.bash" ]]; then
        source "$LIVOX_WS/install/setup.bash"
    fi
    log_info "环境检查通过"
}

modify_config() {
    [[ -z "$MAP_PATH" ]] && MAP_PATH="$SDK_MAP/"
    log_info "地图保存路径: $MAP_PATH"
    [[ "${MAP_PATH}" != */ ]] && MAP_PATH="${MAP_PATH}/"
    mkdir -p "$MAP_PATH"
    CONFIG_BACKUP="${CONFIG_FILE}.bak.$$"
    cp "$CONFIG_FILE" "$CONFIG_BACKUP"
    sed -i "s|map_save_path:.*|map_save_path: \"$MAP_PATH\"|" "$CONFIG_FILE"
}

start_driver() {
    if [[ "$USE_DRIVER" == true ]]; then
        LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
        if [[ ! -f "$LIVOX_WS/install/setup.bash" ]]; then
            log_error "未找到 livox_ros_driver2: $LIVOX_WS"
            log_error "请设置 LIVOX_WS_PATH 或确保 ~/livox_ws 存在"
            exit 1
        fi
        source "$LIVOX_WS/install/setup.bash"
        log_info "启动 Livox MID360 驱动..."
        ros2 launch livox_ros_driver2 msg_MID360_launch.py &
        DRIVER_PID=$!
        sleep 3
        kill -0 $DRIVER_PID 2>/dev/null || { log_error "驱动启动失败"; exit 1; }
        log_info "Livox 驱动已启动 (PID: $DRIVER_PID)"
    elif [[ -z "$BAG_PATH" ]]; then
        log_warn "未指定 --bag 且未启用 --driver，将等待外部数据源"
    fi
}

start_slam_node() {
    log_info "启动 Adaptive-LIO 节点..."
    log_info "配置文件: $(basename $CONFIG_FILE)"
    export ADAPTIVE_LIO_CONFIG="$CONFIG_FILE"
    "$SDK_BIN/adaptive_lio_node" --ros-args --remap __node:=adaptive_lio &
    NODE_PID=$!
    sleep 2
    if ! kill -0 $NODE_PID 2>/dev/null; then
        log_error "节点启动失败"
        log_error "请运行: ldd $SDK_BIN/adaptive_lio_node | grep 'not found'"
        exit 1
    fi
    log_info "节点已启动 (PID: $NODE_PID)"
}

start_rviz() {
    if [[ "$USE_RVIZ" == true ]]; then
        log_info "启动 RViz2..."
        RVIZ_CONFIG="$SDK_CONFIG/adaptive_lio.rviz"
        if [[ -f "$RVIZ_CONFIG" ]]; then
            ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
        else
            ros2 run rviz2 rviz2 &
        fi
        RVIZ_PID=$!
        log_info "RViz2 已启动 (PID: $RVIZ_PID)"
    else
        log_info "跳过 RViz2 (--no-rviz)"
    fi
}

play_bag() {
    if [[ -n "$BAG_PATH" ]]; then
        [[ ! -e "$BAG_PATH" ]] && { log_error "rosbag 不存在: $BAG_PATH"; exit 1; }
        log_info "播放 rosbag: $BAG_PATH (速率: ${BAG_RATE}x)"
        sleep 3
        ros2 bag play "$BAG_PATH" --rate "$BAG_RATE" &
        BAG_PID=$!
        log_info "rosbag 播放已启动 (PID: $BAG_PID)"
    fi
}

start_record_bag() {
    if [[ "$RECORD_BAG" == true ]]; then
        local bag_dir="${RECORD_BAG_PATH:-.}"
        mkdir -p "$bag_dir"
        local timestamp=$(date +%Y%m%d_%H%M%S)
        local bag_name="${bag_dir}/slam_record_${timestamp}"
        log_info "开始录制 rosbag: $bag_name"
        ros2 bag record /livox/lidar /livox/imu -o "$bag_name" &
        RECORD_PID=$!
        sleep 1
        kill -0 $RECORD_PID 2>/dev/null || { log_error "录制启动失败"; exit 1; }
        log_info "rosbag 录制已启动 (PID: $RECORD_PID)"
    fi
}

show_status() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Adaptive-LIO SLAM SDK 已启动${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "  SDK 路径:   $SDK_DIR"
    echo -e "  配置文件:   $(basename $CONFIG_FILE)"
    [[ "$USE_ORIN" == true ]] && echo -e "  模式:       ${YELLOW}Orin NX 优化模式${NC}"
    [[ "$USE_ORIN_NANO" == true ]] && echo -e "  模式:       ${YELLOW}Orin Nano 极限优化模式${NC}"
    echo -e "  Livox驱动:  $([ "$USE_DRIVER" == true ] && echo "启用" || echo "禁用")"
    echo -e "  RViz2:      $([ "$USE_RVIZ" == true ] && echo "启用" || echo "禁用")"
    echo -e "  录制rosbag: $([ "$RECORD_BAG" == true ] && echo "启用 (${RECORD_BAG_PATH:-.})" || echo "禁用")"
    echo -e "  地图路径:   ${MAP_PATH}"
    echo -e "  rosbag:     ${BAG_PATH:-"未指定 (等待实时数据)"}"
    [[ -n "$BAG_PATH" ]] && echo -e "  播放速率:   ${BAG_RATE}x"
    echo ""
    echo -e "  ${YELLOW}保存地图:${NC} ros2 service call /save_map std_srvs/srv/Trigger"
    echo -e "  ${YELLOW}退出:${NC} Ctrl+C"
    echo ""
}

main() {
    log_info "Adaptive-LIO SLAM SDK ($(uname -m))"
    check_environment
    modify_config
    start_driver
    start_slam_node
    start_rviz
    start_record_bag
    play_bag
    show_status

    if [[ -n "$BAG_PID" ]]; then
        wait $BAG_PID 2>/dev/null || true
        log_info "rosbag 播放完成"
        echo ""
        log_warn "rosbag 播放完成。按 Ctrl+C 退出并自动合并保存地图..."
        wait $NODE_PID 2>/dev/null || true
    else
        wait $NODE_PID 2>/dev/null || true
    fi
}

main
RUNSCRIPT_EOF
chmod +x "$OUTPUT_DIR/run.sh"

# ==================== 生成 check_env.sh ====================
log_info "生成环境检查脚本 (check_env.sh)..."
cat > "$OUTPUT_DIR/check_env.sh" << 'CHECKENV_EOF'
#!/bin/bash
SDK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0; FAIL=0; WARN=0

check_pass() { echo -e "  ${GREEN}[OK]${NC} $1"; PASS=$((PASS+1)); }
check_fail() { echo -e "  ${RED}[FAIL]${NC} $1"; FAIL=$((FAIL+1)); }
check_warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; WARN=$((WARN+1)); }

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Adaptive-LIO SDK 环境检查${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

echo -e "${CYAN}[系统信息]${NC}"
echo "  OS: $(lsb_release -d 2>/dev/null | cut -f2 || cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d'"' -f2)"
echo "  Arch: $(uname -m)"
echo "  Kernel: $(uname -r)"
echo "  RAM: $(free -h 2>/dev/null | awk '/Mem:/{print $2}' || echo 'unknown')"
echo ""

echo -e "${CYAN}[ROS2 环境]${NC}"
if command -v ros2 &>/dev/null; then
    check_pass "ros2 命令可用"
else
    check_fail "ros2 未找到 -> sudo apt install ros-humble-desktop"
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
    check_pass "ROS2 Humble 已安装"
else
    check_fail "ROS2 Humble 未安装"
fi
if [[ -n "$AMENT_PREFIX_PATH" ]]; then
    check_pass "ROS2 环境已加载"
else
    check_warn "ROS2 环境未 source -> source /opt/ros/humble/setup.bash"
fi
echo ""

echo -e "${CYAN}[系统库依赖]${NC}"
check_lib() {
    if ldconfig -p 2>/dev/null | grep -q "$1"; then
        check_pass "$1"
    else
        check_fail "$1 -> sudo apt install $2"
    fi
}
check_lib "libceres.so" "libceres-dev"
check_lib "libpcl_common.so" "libpcl-dev"
check_lib "libglog.so" "libgoogle-glog-dev"
check_lib "libyaml-cpp.so" "libyaml-cpp-dev"
echo ""

echo -e "${CYAN}[SDK 文件完整性]${NC}"
check_file() {
    if [[ -e "$SDK_DIR/$1" ]]; then check_pass "$2 ($1)"; else check_fail "$2 缺失: $1"; fi
}
check_file "bin/adaptive_lio_node" "SLAM 可执行文件"
check_file "config/mapping_m.yaml" "默认配置"
check_file "config/mapping_orin_nx.yaml" "Orin NX 配置"
check_file "config/mapping_orin_nano.yaml" "Orin Nano 配置"
check_file "lib/liblivox_ros_driver2__rosidl_typesupport_cpp.so" "Livox 消息类型库"
check_file "run.sh" "SDK 启动脚本"
echo ""

echo -e "${CYAN}[二进制信息]${NC}"
if [[ -x "$SDK_DIR/bin/adaptive_lio_node" ]]; then
    BIN_ARCH=$(file "$SDK_DIR/bin/adaptive_lio_node" | grep -oP '(x86-64|aarch64|ARM)')
    SYS_ARCH=$(uname -m)
    echo "  二进制架构: $BIN_ARCH"
    echo "  系统架构:   $SYS_ARCH"
    if [[ ("$SYS_ARCH" == "x86_64" && "$BIN_ARCH" == *"x86-64"*) || \
          ("$SYS_ARCH" == "aarch64" && ("$BIN_ARCH" == *"aarch64"* || "$BIN_ARCH" == *"ARM"*)) ]]; then
        check_pass "架构匹配"
    else
        check_fail "架构不匹配! 二进制=$BIN_ARCH 系统=$SYS_ARCH"
    fi

    export LD_LIBRARY_PATH="$SDK_DIR/lib:${LD_LIBRARY_PATH:-}"
    MISSING=$(ldd "$SDK_DIR/bin/adaptive_lio_node" 2>/dev/null | grep "not found" || true)
    if [[ -z "$MISSING" ]]; then
        check_pass "所有动态库依赖已满足"
    else
        check_fail "以下库缺失:"
        echo "$MISSING" | while read -r line; do echo -e "    ${RED}$line${NC}"; done
    fi
else
    check_fail "二进制不可执行"
fi
echo ""

echo -e "${CYAN}[Livox 驱动 (可选)]${NC}"
LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
if [[ -f "$LIVOX_WS/install/setup.bash" ]]; then
    check_pass "livox_ros_driver2: $LIVOX_WS"
else
    check_warn "livox_ros_driver2 未找到 (仅影响 --driver 模式)"
fi
echo ""

echo -e "${CYAN}========================================${NC}"
echo -e "  通过: ${GREEN}${PASS}${NC}  失败: ${RED}${FAIL}${NC}  警告: ${YELLOW}${WARN}${NC}"
echo -e "${CYAN}========================================${NC}"
if [[ $FAIL -eq 0 ]]; then
    echo -e "${GREEN}环境检查通过! 可以运行: ./run.sh${NC}"
    exit 0
else
    echo -e "${RED}存在 $FAIL 个问题，请先修复${NC}"
    exit 1
fi
CHECKENV_EOF
chmod +x "$OUTPUT_DIR/check_env.sh"

# ==================== 生成 SDK README ====================
log_info "生成 README.md..."
cat > "$OUTPUT_DIR/README.md" << READMEEOF
# Adaptive-LIO SLAM SDK

基于 CT-ICP + Ceres 多约束优化的激光惯性里程计，支持 Livox MID-360。
预编译二进制分发包，无需源码即可运行。

- 架构: \`$(uname -m)\`
- 打包时间: \`$(date '+%Y-%m-%d %H:%M:%S')\`

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- 系统库: libceres-dev libpcl-dev libgoogle-glog-dev libyaml-cpp-dev

\`\`\`bash
sudo apt install -y ros-humble-desktop libceres-dev libpcl-dev libgoogle-glog-dev libyaml-cpp-dev
\`\`\`

## 快速开始

\`\`\`bash
# 1. 环境检查
source /opt/ros/humble/setup.bash
./check_env.sh

# 2. 实时建图 (Orin NX)
./run.sh --orin --driver --no-rviz --map-path ~/map

# 3. 实时建图 (Orin Nano 8GB)
./run.sh --orin-nano --driver --no-rviz --map-path ~/map

# 4. 回放 rosbag
./run.sh --bag /path/to/data.db3 --map-path ~/map

# 5. 实时建图 + 录包
./run.sh --orin --driver --record-bag ~/bags --no-rviz --map-path ~/map
\`\`\`

## 启动选项

| 选项 | 说明 |
|------|------|
| \`--rviz\` | 启动 RViz2 可视化 (默认) |
| \`--no-rviz\` | 不启动 RViz2 |
| \`--orin\` | Orin NX 优化配置 (16GB) |
| \`--orin-nano\` | Orin Nano 极限优化配置 (8GB) |
| \`--driver\` | 启动 Livox MID360 驱动 |
| \`--record-bag [PATH]\` | 录制 rosbag |
| \`--config FILE\` | 使用自定义配置文件 |
| \`--map-path PATH\` | 地图保存路径 (默认: sdk/map/) |
| \`--bag PATH\` | 播放 rosbag |
| \`--rate RATE\` | 播放速率 (默认: 1.0) |

选项之间**顺序不限**。

## 平台配置对比

| 参数 | x86 默认 | Orin NX (16GB) | Orin Nano (8GB) |
|------|---------|----------------|-----------------|
| point_filter_num | 1 | 2 | 3 |
| max_num_iteration | 10 | 6 | 4 |
| surf_res | 0.5 | 0.6 | 0.8 |
| size_voxel_map | 0.5 | 0.6 | 0.8 |
| max_distance | 80m | 60m | 40m |
| max_num_residuals | 1200 | 800 | 500 |

## ROS2 接口

**订阅**: \`/livox/lidar\` (CustomMsg), \`/livox/imu\` (Imu)

**发布**: \`/cloud_registered\` (PointCloud2), \`/aft_mapped_to_init\` (Odometry), \`/path\` (Path)

**服务**: \`ros2 service call /save_map std_srvs/srv/Trigger\`

## 输出文件

退出时 (Ctrl+C) 自动保存到 \`--map-path\` 目录:
- \`combined_map.pcd\` - 全局点云地图
- \`trajectory.txt\` - TUM 格式轨迹

## 故障排查

| 问题 | 解决方案 |
|------|---------|
| 动态库找不到 | \`source /opt/ros/humble/setup.bash\` 然后 \`ldd bin/adaptive_lio_node \| grep 'not found'\` |
| 节点启动失败 | 运行 \`./check_env.sh\` 检查架构匹配和依赖 |
| 帧处理慢 | 使用 \`--orin\` 或 \`--orin-nano\`; 检查 \`free -h\` swap 使用 |
| 地图未保存 | Ctrl+C 后等待"清理完成"提示 (最多120秒)，不要多次 Ctrl+C |
| Livox 驱动问题 | 设置 \`LIVOX_WS_PATH\` 或确保 \`~/livox_ws\` 存在 |
READMEEOF

# ==================== 汇总 ====================
echo ""
log_step "打包完成!"
echo ""

FILE_COUNT=$(find "$OUTPUT_DIR" -type f | wc -l)
TOTAL_SIZE=$(du -sh "$OUTPUT_DIR" | cut -f1)

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  SDK 打包汇总${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "  输出目录:  $OUTPUT_DIR"
echo -e "  架构:      $ARCH ($ARCH_LABEL)"
echo -e "  文件数量:  $FILE_COUNT"
echo -e "  总大小:    $TOTAL_SIZE"
echo ""
echo -e "  ${GREEN}bin/${NC}      adaptive_lio_node ($(du -sh "$OUTPUT_DIR/bin/adaptive_lio_node" | cut -f1))"
echo -e "  ${GREEN}config/${NC}   mapping_m.yaml, mapping_orin_nx.yaml, mapping_orin_nano.yaml"
echo -e "  ${GREEN}lib/${NC}      livox_ros_driver2 库 ($LIVOX_SO_COUNT 个 .so)"
echo -e "  ${GREEN}scripts/${NC}  gui_launcher.py, run_slam.sh, etc."
echo ""

# 生成 tar.gz
if [[ "$CREATE_TAR" == true ]]; then
    TAR_NAME="adaptive_lio_sdk_${ARCH_LABEL}_$(date +%Y%m%d).tar.gz"
    TAR_PATH="$(dirname "$OUTPUT_DIR")/$TAR_NAME"
    log_info "生成压缩包: $TAR_PATH"
    tar -czf "$TAR_PATH" -C "$(dirname "$OUTPUT_DIR")" "$(basename "$OUTPUT_DIR")"
    TAR_SIZE=$(du -sh "$TAR_PATH" | cut -f1)
    echo -e "  压缩包:    $TAR_PATH ($TAR_SIZE)"
    echo ""
fi

echo -e "部署到目标设备:"
echo -e "  ${CYAN}scp -r $OUTPUT_DIR user@target:~/sdk/${NC}"
echo -e "  或: ${CYAN}scp $TAR_PATH user@target:~/ && ssh user@target 'tar xzf ~/$TAR_NAME'${NC}"
echo ""
echo -e "在目标设备运行:"
echo -e "  ${CYAN}source /opt/ros/humble/setup.bash${NC}"
echo -e "  ${CYAN}cd ~/sdk && ./check_env.sh && ./run.sh --orin --driver --no-rviz --map-path ~/map${NC}"
echo ""
