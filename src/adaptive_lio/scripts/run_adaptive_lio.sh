#!/bin/bash
# ============================================================
# Adaptive-LIO ROS2 启动脚本
# 用法:
#   ./run_adaptive_lio.sh                         # 默认保存到 map/ 目录
#   ./run_adaptive_lio.sh /home/li/maps/test1     # 指定PCD保存路径
#   ./run_adaptive_lio.sh --bag /path/to/bag      # 同时播放数据包
#   ./run_adaptive_lio.sh --bag /path/to/bag --rate 0.5 --save /home/li/maps
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"

# 默认参数
SAVE_PATH=""
BAG_PATH=""
BAG_RATE="1.0"
BAG_LOOP=""
NO_RVIZ=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --save|-s)
            SAVE_PATH="$2"
            shift 2
            ;;
        --bag|-b)
            BAG_PATH="$2"
            shift 2
            ;;
        --rate|-r)
            BAG_RATE="$2"
            shift 2
            ;;
        --loop)
            BAG_LOOP="--loop"
            shift
            ;;
        --no-rviz)
            NO_RVIZ=true
            shift
            ;;
        --help|-h)
            echo "Adaptive-LIO ROS2 启动脚本"
            echo ""
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --save, -s <path>     PCD地图保存路径 (默认: <pkg>/map/)"
            echo "  --bag,  -b <path>     ROS2数据包路径 (自动播放)"
            echo "  --rate, -r <rate>     播放速率 (默认: 1.0)"
            echo "  --loop                循环播放"
            echo "  --no-rviz             不启动RViz2"
            echo "  --help, -h            显示帮助"
            echo ""
            echo "示例:"
            echo "  $0                                          # 实时传感器模式"
            echo "  $0 --bag ~/rosbag/staircase --rate 1.0      # 回放数据包"
            echo "  $0 --bag ~/rosbag/test --save ~/maps/run1   # 回放+指定保存路径"
            echo ""
            echo "运行中保存地图:  ros2 service call /save_map std_srvs/srv/Trigger"
            echo "Ctrl+C停止时自动保存地图"
            exit 0
            ;;
        *)
            # 向后兼容: 第一个无标记参数作为保存路径
            if [ -z "$SAVE_PATH" ]; then
                SAVE_PATH="$1"
            fi
            shift
            ;;
    esac
done

# 设置环境
echo "========================================="
echo "  Adaptive-LIO ROS2 启动"
echo "========================================="

source /opt/ros/humble/setup.bash

# 支持环境变量 LIVOX_WS_PATH，默认 ~/livox_ws
LIVOX_WS="${LIVOX_WS_PATH:-$HOME/livox_ws}"
if [ -f "$LIVOX_WS/install/setup.bash" ]; then
    source "$LIVOX_WS/install/setup.bash"
fi

source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# 创建保存目录
if [ -n "$SAVE_PATH" ]; then
    mkdir -p "$SAVE_PATH"
    echo "[INFO] 地图保存路径: $SAVE_PATH"
else
    mkdir -p "$PKG_DIR/map"
    SAVE_PATH="$PKG_DIR/map"
    echo "[INFO] 地图保存路径: $SAVE_PATH (默认)"
fi

# 清理函数
cleanup() {
    echo ""
    echo "[INFO] 正在停止..."

    # 停止bag播放
    if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
        kill -SIGTERM "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null || true
        echo "[INFO] 数据包播放已停止"
    fi

    # 停止rviz
    if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill -SIGTERM "$RVIZ_PID" 2>/dev/null
        wait "$RVIZ_PID" 2>/dev/null || true
    fi

    # 等待节点自动保存 (SIGTERM会触发析构函数保存)
    if [ -n "$NODE_PID" ] && kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[INFO] 等待地图自动保存..."
        kill -SIGTERM "$NODE_PID" 2>/dev/null
        # 给节点10秒时间保存地图
        for i in $(seq 1 10); do
            if ! kill -0 "$NODE_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        kill -9 "$NODE_PID" 2>/dev/null || true
    fi

    # 如果指定了保存路径且不是默认路径，复制地图
    DEFAULT_PCD="$PKG_DIR/map/global_map.pcd"
    TARGET_PCD="$SAVE_PATH/global_map.pcd"
    if [ -f "$DEFAULT_PCD" ] && [ "$(realpath "$SAVE_PATH")" != "$(realpath "$PKG_DIR/map")" ]; then
        cp "$DEFAULT_PCD" "$TARGET_PCD"
        echo "[INFO] 地图已复制到: $TARGET_PCD"
    fi

    if [ -f "$TARGET_PCD" ] || [ -f "$DEFAULT_PCD" ]; then
        echo "[INFO] 地图文件: $TARGET_PCD"
        echo "[INFO] 查看命令: pcl_viewer $TARGET_PCD"
    fi

    echo "[INFO] 已退出"
}

trap cleanup EXIT INT TERM

# 启动 Adaptive-LIO 节点
echo "[INFO] 启动 Adaptive-LIO 节点..."
ros2 run adaptive_lio adaptive_lio_node &
NODE_PID=$!
sleep 2

# 检查节点是否存活
if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "[ERROR] Adaptive-LIO 节点启动失败"
    exit 1
fi
echo "[INFO] 节点已启动 (PID: $NODE_PID)"

# 可选: 启动 RViz2
if [ "$NO_RVIZ" = false ]; then
    RVIZ_CONFIG="$WS_DIR/install/adaptive_lio/share/adaptive_lio/config/adaptive_lio.rviz"
    if [ -f "$RVIZ_CONFIG" ]; then
        echo "[INFO] 启动 RViz2..."
        rviz2 -d "$RVIZ_CONFIG" &
        RVIZ_PID=$!
    fi
fi

# 可选: 播放数据包
if [ -n "$BAG_PATH" ]; then
    if [ -d "$BAG_PATH" ] || [ -f "$BAG_PATH" ]; then
        echo "[INFO] 播放数据包: $BAG_PATH (速率: $BAG_RATE)"
        sleep 1
        ros2 bag play "$BAG_PATH" --rate "$BAG_RATE" $BAG_LOOP &
        BAG_PID=$!
    else
        echo "[WARNING] 数据包路径不存在: $BAG_PATH"
    fi
fi

echo ""
echo "========================================="
echo "  运行中..."
echo "  保存地图: ros2 service call /save_map std_srvs/srv/Trigger"
echo "  停止: Ctrl+C (自动保存地图)"
echo "========================================="
echo ""

# 等待主节点结束
wait $NODE_PID 2>/dev/null || true
