#!/bin/bash
# Adaptive-LIO ROS2 控制面板启动脚本

# 检查 python3-tk
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "缺少 python3-tk，正在安装..."
    sudo apt-get install -y python3-tk
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
python3 "$SCRIPT_DIR/gui_launcher.py"
