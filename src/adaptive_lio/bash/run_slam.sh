#!/bin/bash
#
# Adaptive-LIO SLAM 启动脚本
# 
# 用法:
#   ./run_slam.sh [选项]
#
# 选项:
#   --rviz          启动 RViz2 可视化 (默认)
#   --no-rviz       不启动 RViz2
#   --orin          使用 Orin NX 优化配置 (低功耗/边缘设备)
#   --config FILE   使用指定的配置文件
#   --map-path PATH 指定地图保存路径 (默认: 源码目录/map/)
#   --bag PATH      播放指定的 rosbag 文件
#   --rate RATE     rosbag 播放速率 (默认: 1.0)
#   -h, --help      显示帮助信息
#
# 示例:
#   ./run_slam.sh --rviz --map-path /tmp/my_map/
#   ./run_slam.sh --orin --no-rviz --bag /path/to/data.db3 --rate 2.0
#

set -e

# 脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"

# 默认参数
USE_RVIZ=true
MAP_PATH=""
BAG_PATH=""
BAG_RATE="1.0"
USE_ORIN=false
CUSTOM_CONFIG=""

# 配置文件路径
CONFIG_FILE="$PKG_DIR/config/mapping_m.yaml"
CONFIG_ORIN="$PKG_DIR/config/mapping_orin_nx.yaml"
CONFIG_BACKUP=""

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    echo "Adaptive-LIO SLAM 启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --rviz              启动 RViz2 可视化 (默认)"
    echo "  --no-rviz           不启动 RViz2"
    echo "  --orin              使用 Orin NX 优化配置 (边缘设备/低功耗)"
    echo "  --config FILE       使用指定的配置文件"
    echo "  --map-path PATH     指定地图保存路径"
    echo "  --bag PATH          播放指定的 rosbag 文件"
    echo "  --rate RATE         rosbag 播放速率 (默认: 1.0)"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --rviz"
    echo "  $0 --orin --no-rviz --bag /path/to/rosbag --rate 1.5"
    echo "  $0 --config /path/to/custom.yaml --map-path /tmp/slam_output/"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --rviz)
            USE_RVIZ=true
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --orin)
            USE_ORIN=true
            shift
            ;;
        --config)
            CUSTOM_CONFIG="$2"
            shift 2
            ;;
        --map-path)
            MAP_PATH="$2"
            shift 2
            ;;
        --bag)
            BAG_PATH="$2"
            shift 2
            ;;
        --rate)
            BAG_RATE="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 选择配置文件
if [[ -n "$CUSTOM_CONFIG" ]]; then
    if [[ ! -f "$CUSTOM_CONFIG" ]]; then
        log_error "配置文件不存在: $CUSTOM_CONFIG"
        exit 1
    fi
    CONFIG_FILE="$CUSTOM_CONFIG"
    log_info "使用自定义配置: $CONFIG_FILE"
elif [[ "$USE_ORIN" == true ]]; then
    if [[ ! -f "$CONFIG_ORIN" ]]; then
        log_error "Orin 配置文件不存在: $CONFIG_ORIN"
        exit 1
    fi
    CONFIG_FILE="$CONFIG_ORIN"
    log_info "使用 Orin NX 优化配置: $CONFIG_FILE"
fi

# 清理函数 - 退出时恢复配置文件
cleanup() {
    log_info "正在清理..."
    
    # 恢复配置文件
    if [[ -n "$CONFIG_BACKUP" && -f "$CONFIG_BACKUP" ]]; then
        cp "$CONFIG_BACKUP" "$CONFIG_FILE"
        rm -f "$CONFIG_BACKUP"
        log_info "已恢复配置文件"
    fi
    
    # 杀掉子进程
    if [[ -n "$RVIZ_PID" ]] && kill -0 $RVIZ_PID 2>/dev/null; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    if [[ -n "$BAG_PID" ]] && kill -0 $BAG_PID 2>/dev/null; then
        kill $BAG_PID 2>/dev/null || true
    fi
    if [[ -n "$NODE_PID" ]] && kill -0 $NODE_PID 2>/dev/null; then
        kill -INT $NODE_PID 2>/dev/null || true
        # 等待节点优雅退出（执行地图合并），最多等待30秒
        log_info "等待节点完成地图保存 (最多30秒)..."
        for i in $(seq 1 30); do
            if ! kill -0 $NODE_PID 2>/dev/null; then
                break
            fi
            sleep 1
        done
        # 如果还没退出，强制杀掉
        if kill -0 $NODE_PID 2>/dev/null; then
            log_warn "节点未能正常退出，强制终止"
            kill -9 $NODE_PID 2>/dev/null || true
        fi
    fi
    
    log_info "清理完成"
}

trap cleanup EXIT INT TERM

# 检查环境
check_environment() {
    log_info "检查环境依赖..."
    
    # 检查 ROS2
    if ! command -v ros2 &> /dev/null; then
        log_error "未找到 ros2 命令，请先 source ROS2 环境"
        exit 1
    fi
    
    # 检查工作空间是否已编译
    if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
        log_error "工作空间未编译，请先运行: colcon build"
        exit 1
    fi
    
    # source 工作空间
    source "$WS_DIR/install/setup.bash"
    
    # 检查包是否存在
    if ! ros2 pkg list 2>/dev/null | grep -q "adaptive_lio"; then
        log_error "未找到 adaptive_lio 包"
        exit 1
    fi
    
    log_info "环境检查通过"
}

# 修改配置文件中的地图保存路径
modify_config() {
    if [[ -n "$MAP_PATH" ]]; then
        log_info "设置地图保存路径: $MAP_PATH"
        
        # 确保路径以 / 结尾
        [[ "${MAP_PATH}" != */ ]] && MAP_PATH="${MAP_PATH}/"
        
        # 创建目录
        mkdir -p "$MAP_PATH"
        
        # 备份原配置
        CONFIG_BACKUP="${CONFIG_FILE}.bak.$$"
        cp "$CONFIG_FILE" "$CONFIG_BACKUP"
        
        # 修改配置文件中的 map_save_path
        sed -i "s|map_save_path:.*|map_save_path: \"$MAP_PATH\"|" "$CONFIG_FILE"
    fi
}

# 启动 SLAM 节点
start_slam_node() {
    log_info "启动 Adaptive-LIO 节点..."
    log_info "配置文件: $CONFIG_FILE"
    
    # 设置配置文件环境变量
    export ADAPTIVE_LIO_CONFIG="$CONFIG_FILE"
    
    ros2 run adaptive_lio adaptive_lio_node &
    NODE_PID=$!
    sleep 2
    
    if ! kill -0 $NODE_PID 2>/dev/null; then
        log_error "节点启动失败"
        exit 1
    fi
    log_info "节点已启动 (PID: $NODE_PID)"
}

# 启动 RViz2
start_rviz() {
    if [[ "$USE_RVIZ" == true ]]; then
        log_info "启动 RViz2..."
        RVIZ_CONFIG="$PKG_DIR/config/adaptive_lio.rviz"
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

# 播放 rosbag
play_bag() {
    if [[ -n "$BAG_PATH" ]]; then
        if [[ ! -e "$BAG_PATH" ]]; then
            log_error "rosbag 文件不存在: $BAG_PATH"
            exit 1
        fi
        
        log_info "播放 rosbag: $BAG_PATH (速率: ${BAG_RATE}x)"
        sleep 3  # 等待节点初始化
        ros2 bag play "$BAG_PATH" --rate "$BAG_RATE" &
        BAG_PID=$!
        log_info "rosbag 播放已启动 (PID: $BAG_PID)"
    fi
}

# 显示运行信息
show_status() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Adaptive-LIO SLAM 已启动${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "  配置文件:   $(basename $CONFIG_FILE)"
    [[ "$USE_ORIN" == true ]] && echo -e "  模式:       ${YELLOW}Orin NX 优化模式${NC}"
    echo -e "  RViz2:      $([ "$USE_RVIZ" == true ] && echo "启用" || echo "禁用")"
    echo -e "  地图路径:   ${MAP_PATH:-"默认 (源码目录/map/)"}"
    echo -e "  rosbag:     ${BAG_PATH:-"未指定 (等待实时数据)"}"
    [[ -n "$BAG_PATH" ]] && echo -e "  播放速率:   ${BAG_RATE}x"
    echo ""
    echo -e "  ${YELLOW}保存地图:${NC} ros2 service call /save_map std_srvs/srv/Trigger"
    echo -e "  ${YELLOW}退出:${NC} Ctrl+C"
    echo ""
}

# 主函数
main() {
    check_environment
    modify_config
    start_slam_node
    start_rviz
    play_bag
    show_status
    
    # 等待主节点退出
    if [[ -n "$BAG_PID" ]]; then
        # 如果有 rosbag，等待它播放完成
        wait $BAG_PID 2>/dev/null || true
        log_info "rosbag 播放完成"
        
        # 给用户时间保存地图
        echo ""
        log_warn "rosbag 播放完成。按 Ctrl+C 退出并自动合并保存地图..."
        wait $NODE_PID 2>/dev/null || true
    else
        # 没有 rosbag，等待节点
        wait $NODE_PID 2>/dev/null || true
    fi
}

main
