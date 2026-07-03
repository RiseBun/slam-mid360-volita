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
#   --orin          使用 Orin NX 优化配置 (16GB)
#   --orin-nano     使用 Orin Nano 极限优化配置 (8GB/低内存设备)
#   --driver        启动 Livox MID360 驱动 (实时模式需要)
#   --guard         启动 LiDAR/相机位姿仲裁节点 (默认启用)
#   --no-guard      不启动位姿仲裁节点
#   --record-bag [PATH]  录制 rosbag (可选指定保存目录，默认当前目录)
#   --dense [VOXEL]   启用 dense_map_mode（全分辨率PCD输出，无降采样）
#                     可选 VOXEL 参数指定合并时降采样体素大小（米）
#                     例: --dense 0.02 表示 2cm 体素降采样
#                     不带参数则不降采样（原始全量点）
#   --config FILE   使用指定的配置文件
#   --map-path PATH 指定地图保存路径 (默认: 源码目录/map/)
#   --bag PATH      播放指定的 rosbag 文件
#   --rate RATE     rosbag 播放速率 (默认: 1.0)
#   -h, --help      显示帮助信息
#
# 示例:
#   ./run_slam.sh --rviz --map-path /tmp/my_map/
#   ./run_slam.sh --orin --no-rviz --bag /path/to/data.db3 --rate 2.0
#   ./run_slam.sh --orin --driver --no-rviz --map-path ~/map  # 实时建图
#   ./run_slam.sh --orin-nano --driver --no-rviz              # Nano 实时建图
#   ./run_slam.sh --driver --record-bag ~/bags --map-path ~/map  # 建图+录包
#   ./run_slam.sh --driver --no-rviz                 # 实时建图 + 位姿仲裁
#   ./run_slam.sh --orin --driver --dense --no-rviz             # 全分辨率PCD（不降采样）
#   ./run_slam.sh --orin --driver --dense 0.02 --no-rviz       # 2cm体素降采样PCD
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
USE_ORIN_NANO=false
CONFIG_PROFILE="indoor"
USE_DRIVER=false
USE_DENSE=false
DENSE_VOXEL=""
RECORD_BAG=false
RECORD_BAG_PATH=""
CUSTOM_CONFIG=""
USE_GUARD=true

# 配置文件路径
CONFIG_FILE="$PKG_DIR/config/mapping_m.yaml"
CONFIG_OUTDOOR="$PKG_DIR/config/mapping_high_altitude.yaml"
CONFIG_ORIN="$PKG_DIR/config/mapping_orin_nx.yaml"
CONFIG_ORIN_NANO="$PKG_DIR/config/mapping_orin_nano.yaml"
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
    echo "  --orin              使用 Orin NX 优化配置 (16GB)"
    echo "  --orin-nano         使用 Orin Nano 极限优化配置 (8GB/低内存)"
    echo "  --driver            启动 Livox MID360 驱动 (实时模式需要)"
    echo "  --guard             启动 LiDAR/相机位姿仲裁节点 (默认启用)"
    echo "  --no-guard          不启动位姿仲裁节点"
    echo "  --dense             启用 dense_map_mode (全分辨率PCD输出)"
    echo "  --record-bag [PATH] 录制 rosbag (可选指定保存目录，默认当前目录)"
    echo "  --config FILE       使用指定的配置文件"
    echo "  --map-path PATH     指定地图保存路径"
    echo "  --bag PATH          播放指定的 rosbag 文件"
    echo "  --rate RATE         rosbag 播放速率 (默认: 1.0)"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --rviz"
    echo "  $0 --orin --no-rviz --bag /path/to/rosbag --rate 1.5"
    echo "  $0 --orin --driver --no-rviz --map-path ~/map  # 实时建图"
    echo "  $0 --driver --no-rviz                          # 雷达驱动 + SLAM + 位姿仲裁"
    echo "  $0 --orin-nano --driver --no-rviz              # Nano 实时建图"
    echo "  $0 --driver --record-bag ~/bags --map-path ~/map  # 建图+录包"
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
        --indoor)
            CONFIG_PROFILE="indoor"
            USE_ORIN=false
            USE_ORIN_NANO=false
            shift
            ;;
        --outdoor|--high-altitude)
            CONFIG_PROFILE="outdoor"
            USE_ORIN=false
            USE_ORIN_NANO=false
            shift
            ;;
        --orin)
            USE_ORIN=true
            USE_ORIN_NANO=false
            CONFIG_PROFILE="orin"
            shift
            ;;
        --orin-nano)
            USE_ORIN=false
            USE_ORIN_NANO=true
            CONFIG_PROFILE="orin-nano"
            shift
            ;;
        --driver)
            USE_DRIVER=true
            shift
            ;;
        --guard)
            USE_GUARD=true
            shift
            ;;
        --no-guard)
            USE_GUARD=false
            shift
            ;;
        --dense)
            USE_DENSE=true
            # 检查下一个参数是否是数字（体素大小）
            if [[ $# -ge 2 && "$2" =~ ^[0-9]*\.?[0-9]+$ ]]; then
                DENSE_VOXEL="$2"
                shift 2
            else
                shift
            fi
            ;;
        --record-bag)
            RECORD_BAG=true
            # 检查下一个参数是否是路径（不以 -- 开头且存在下一个参数）
            if [[ $# -ge 2 && "$2" != --* ]]; then
                RECORD_BAG_PATH="$2"
                shift 2
            else
                shift
            fi
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
elif [[ "$CONFIG_PROFILE" == "outdoor" ]]; then
    if [[ ! -f "$CONFIG_OUTDOOR" ]]; then
        log_error "高空配置文件不存在: $CONFIG_OUTDOOR"
        exit 1
    fi
    CONFIG_FILE="$CONFIG_OUTDOOR"
    log_info "使用高空建图配置: $CONFIG_FILE"
elif [[ "$USE_ORIN_NANO" == true ]]; then
    if [[ ! -f "$CONFIG_ORIN_NANO" ]]; then
        log_error "Orin Nano 配置文件不存在: $CONFIG_ORIN_NANO"
        exit 1
    fi
    CONFIG_FILE="$CONFIG_ORIN_NANO"
    log_info "使用 Orin Nano 极限优化配置: $CONFIG_FILE"
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
    if [[ -n "$DRIVER_PID" ]] && kill -0 $DRIVER_PID 2>/dev/null; then
        kill $DRIVER_PID 2>/dev/null || true
    fi
    if [[ -n "$RVIZ_PID" ]] && kill -0 $RVIZ_PID 2>/dev/null; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    if [[ -n "$BAG_PID" ]] && kill -0 $BAG_PID 2>/dev/null; then
        kill $BAG_PID 2>/dev/null || true
    fi
    if [[ -n "$GUARD_STATUS_PID" ]] && kill -0 $GUARD_STATUS_PID 2>/dev/null; then
        kill $GUARD_STATUS_PID 2>/dev/null || true
    fi
    if [[ -n "$GUARD_PID" ]] && kill -0 $GUARD_PID 2>/dev/null; then
        kill -INT $GUARD_PID 2>/dev/null || true
        sleep 1
        kill -9 $GUARD_PID 2>/dev/null || true
    fi
    # 先停止录制（SIGINT 让 ros2 bag record 正常刷盘关闭）
    if [[ -n "$RECORD_PID" ]] && kill -0 $RECORD_PID 2>/dev/null; then
        log_info "停止 rosbag 录制..."
        kill -INT $RECORD_PID 2>/dev/null || true
        # 等待录制进程正常退出（最多10秒）
        for i in $(seq 1 10); do
            if ! kill -0 $RECORD_PID 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 $RECORD_PID 2>/dev/null; then
            kill -9 $RECORD_PID 2>/dev/null || true
        fi
        log_info "rosbag 录制已保存"
    fi
    if [[ -n "$NODE_PID" ]] && kill -0 $NODE_PID 2>/dev/null; then
        kill -INT $NODE_PID 2>/dev/null || true
        # 等待节点优雅退出（执行地图合并），最多等待120秒
        log_info "等待节点完成地图保存 (最多120秒)..."
        for i in $(seq 1 120); do
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

    # Allow this script to be run directly from a fresh terminal.
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    fi
    
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
    
    # source livox 工作空间 (节点订阅 Livox CustomMsg，无论是否启动驱动都需要)
    LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
    if [[ -f "$LIVOX_WS/install/setup.bash" ]]; then
        source "$LIVOX_WS/install/setup.bash"
    fi

    # source DP180/Vilota ROS bridge workspace when available.
    DP180_WS="${DP180_WS_PATH:-$HOME/dp180_ws}"
    if [[ -f "$DP180_WS/install/setup.bash" ]]; then
        source "$DP180_WS/install/setup.bash"
    elif [[ "$USE_GUARD" == true ]]; then
        log_warn "未找到 DP180 ROS 工作空间: $DP180_WS"
        log_warn "仲裁节点仍可启动，但 /S1/vio_odom 需要由相机侧 ROS 桥发布"
    fi

    # source 工作空间
    source "$WS_DIR/install/setup.bash"
    
    # 检查包是否存在
    if ! ros2 pkg list 2>/dev/null | grep -q "adaptive_lio"; then
        log_error "未找到 adaptive_lio 包"
        exit 1
    fi

    if [[ "$USE_GUARD" == true ]] && ! ros2 pkg list 2>/dev/null | grep -q "pose_guard_mapper"; then
        log_error "未找到 pose_guard_mapper 包"
        log_error "请先运行: cd $WS_DIR && colcon build --packages-select pose_guard_mapper --symlink-install"
        exit 1
    fi
    
    log_info "环境检查通过"
}

# 启动 Livox MID360 驱动
start_driver() {
    if [[ "$USE_DRIVER" == true ]]; then
        # 查找 livox_ws
        LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
        if [[ ! -f "$LIVOX_WS/install/setup.bash" ]]; then
            log_error "未找到 livox_ros_driver2 工作空间: $LIVOX_WS"
            log_error "请设置 LIVOX_WS_PATH 环境变量或确保 ~/livox_ws 存在"
            exit 1
        fi
        source "$LIVOX_WS/install/setup.bash"

        log_info "启动 Livox MID360 驱动..."
        ros2 launch livox_ros_driver2 msg_MID360_launch.py &
        DRIVER_PID=$!
        sleep 3  # 等待驱动初始化并建立网络连接

        if ! kill -0 $DRIVER_PID 2>/dev/null; then
            log_error "Livox 驱动启动失败"
            exit 1
        fi
        log_info "Livox 驱动已启动 (PID: $DRIVER_PID)"
    elif [[ -z "$BAG_PATH" ]]; then
        log_warn "未指定 --bag 且未启用 --driver，节点将等待外部数据源"
        log_warn "如需实时建图，请添加 --driver 选项"
    fi
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
    
    # dense 模式：通过环境变量启用 (比 sed 修改 YAML 更可靠)
    if [[ "$USE_DENSE" == true ]]; then
        export ADAPTIVE_LIO_DENSE_MAP=1
        if [[ -n "$DENSE_VOXEL" ]]; then
            export ADAPTIVE_LIO_DENSE_VOXEL="$DENSE_VOXEL"
            log_info "Dense map mode 已启用 (合并时 ${DENSE_VOXEL}m 体素降采样)"
        else
            log_info "Dense map mode 已启用 (全分辨率PCD，无降采样)"
        fi
    fi
    
    ros2 run adaptive_lio adaptive_lio_node &
    NODE_PID=$!
    sleep 2
    
    if ! kill -0 $NODE_PID 2>/dev/null; then
        log_error "节点启动失败"
        exit 1
    fi
    log_info "节点已启动 (PID: $NODE_PID)"
}

# 启动 LiDAR/相机位姿仲裁节点，并在当前终端打印可信来源
start_pose_guard() {
    if [[ "$USE_GUARD" != true ]]; then
        log_info "跳过位姿仲裁节点 (--no-guard)"
        return
    fi

    log_info "启动 LiDAR/相机位姿仲裁节点..."
    mkdir -p "$WS_DIR/log"
    ros2 launch pose_guard_mapper pose_guard_mapper.launch.py \
        > "$WS_DIR/log/pose_guard_mapper.log" 2>&1 &
    GUARD_PID=$!
    sleep 2

    if ! kill -0 $GUARD_PID 2>/dev/null; then
        log_error "位姿仲裁节点启动失败，日志如下:"
        tail -80 "$WS_DIR/log/pose_guard_mapper.log" 2>/dev/null || true
        exit 1
    fi
    log_info "位姿仲裁节点已启动 (PID: $GUARD_PID)"
    log_info "仲裁输出: /trusted_odom, /guarded_map, /pose_guard/status"

    python3 - <<'PY' &
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

GREEN = "\033[0;32m"
YELLOW = "\033[0;33m"
CYAN = "\033[0;36m"
NC = "\033[0m"


class PoseGuardStatusPrinter(Node):
    def __init__(self):
        super().__init__("pose_guard_status_printer")
        self.create_subscription(String, "/pose_guard/status", self.on_status, 10)

    def on_status(self, msg):
        line = msg.data
        if "source=camera" in line:
            print(f"{YELLOW}[POSE_GUARD] 当前相信: CAMERA  | {line}{NC}", flush=True)
        elif "source=lidar" in line:
            print(f"{GREEN}[POSE_GUARD] 当前相信: LIDAR   | {line}{NC}", flush=True)
        else:
            print(f"{CYAN}[POSE_GUARD] {line}{NC}", flush=True)


rclpy.init()
node = PoseGuardStatusPrinter()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
    GUARD_STATUS_PID=$!
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

# 录制 rosbag
start_record_bag() {
    if [[ "$RECORD_BAG" == true ]]; then
        # 确定保存目录
        local bag_dir="${RECORD_BAG_PATH:-.}"
        mkdir -p "$bag_dir"

        # 生成带时间戳的 bag 名称
        local timestamp=$(date +%Y%m%d_%H%M%S)
        local bag_name="${bag_dir}/slam_record_${timestamp}"

        log_info "开始录制 rosbag: $bag_name"
        log_info "录制话题: /livox/lidar /livox/imu"

        ros2 bag record \
            /livox/lidar \
            /livox/imu \
            -o "$bag_name" &
        RECORD_PID=$!

        sleep 1
        if ! kill -0 $RECORD_PID 2>/dev/null; then
            log_error "rosbag 录制启动失败"
            exit 1
        fi
        log_info "rosbag 录制已启动 (PID: $RECORD_PID)"
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
    case "$CONFIG_PROFILE" in
        outdoor) echo -e "  模式:       ${YELLOW}高空建图模式${NC}" ;;
        orin) echo -e "  模式:       ${YELLOW}Orin NX 优化模式${NC}" ;;
        orin-nano) echo -e "  模式:       ${YELLOW}Orin Nano 极限优化模式${NC}" ;;
        *) echo -e "  模式:       室内默认模式" ;;
    esac
    echo -e "  Livox驱动:  $([ "$USE_DRIVER" == true ] && echo "启用" || echo "禁用")"
    echo -e "  位姿仲裁:   $([ "$USE_GUARD" == true ] && echo "${GREEN}启用${NC}" || echo "禁用")"
    echo -e "  RViz2:      $([ "$USE_RVIZ" == true ] && echo "启用" || echo "禁用")"
    if [[ "$USE_DENSE" == true ]]; then
        if [[ -n "$DENSE_VOXEL" ]]; then
            echo -e "  Dense模式:  ${GREEN}启用 (${DENSE_VOXEL}m 体素降采样)${NC}"
        else
            echo -e "  Dense模式:  ${GREEN}启用 (全分辨率，无降采样)${NC}"
        fi
    else
        echo -e "  Dense模式:  禁用"
    fi
    echo -e "  录制rosbag: $([ "$RECORD_BAG" == true ] && echo "启用 (${RECORD_BAG_PATH:-.})" || echo "禁用")"
    echo -e "  地图路径:   ${MAP_PATH:-"默认 (源码目录/map/)"}"
    echo -e "  rosbag:     ${BAG_PATH:-"未指定 (等待实时数据)"}"
    [[ -n "$BAG_PATH" ]] && echo -e "  播放速率:   ${BAG_RATE}x"
    echo ""
    echo -e "  ${YELLOW}保存地图:${NC} ros2 service call /save_map std_srvs/srv/Trigger"
    if [[ "$USE_GUARD" == true ]]; then
        echo -e "  ${YELLOW}保存仲裁地图:${NC} ros2 service call /pose_guard/save_map std_srvs/srv/Trigger"
        echo -e "  ${YELLOW}查看仲裁状态:${NC} ros2 topic echo /pose_guard/status"
    fi
    if [[ "$USE_DENSE" == true ]]; then
        echo -e "  ${CYAN}(全分辨率点云，无降采样，分段保存，退出时自动流式合并)${NC}"
    fi
    echo -e "  ${YELLOW}退出:${NC} Ctrl+C"
    echo ""
}

# 主函数
main() {
    check_environment
    modify_config
    start_driver
    start_slam_node
    start_pose_guard
    start_rviz
    start_record_bag
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
