#!/usr/bin/env python3
"""
Adaptive-LIO ROS2 控制面板 v1.0
基于 CT-ICP + Ceres 多约束优化的激光惯性里程计

标签页结构：
1. 建图控制 - 节点启停、数据回放、录制、地图保存
2. 参数调节 - CT-ICP、约束权重、体素地图参数
"""

import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext, messagebox
import subprocess
import threading
import os
import signal
import time
import re
from pathlib import Path
from datetime import datetime


class AdaptiveLioLauncher:
    def __init__(self, root):
        self.root = root
        self.root.title("Adaptive-LIO ROS2 控制面板")
        self.root.geometry("920x720")
        self.root.resizable(True, True)

        # 进程管理
        self.processes = {
            'livox': None,
            'adaptive_lio': None,
            'bagplay': None,
            'rosbag': None,
            'rviz': None
        }

        # 配置路径
        self.ws_path = Path(__file__).resolve().parent.parent
        # 支持环境变量 LIVOX_WS_PATH，默认 ~/livox_ws
        livox_ws_env = os.environ.get('LIVOX_WS_PATH', '')
        self.livox_ws_path = Path(livox_ws_env) if livox_ws_env else Path.home() / "livox_ws"
        self.ros2_setup = "/opt/ros/humble/setup.bash"
        self.config_path = self.ws_path / "config" / "mapping_m.yaml"

        # 状态变量
        self.is_recording = False
        self.is_playing = False
        self.frame_count = 0

        # 创建界面
        self.create_widgets()

        # 启动时检查环境
        self.root.after(500, self.check_environment)

    def create_widgets(self):
        """创建所有界面组件"""
        # 顶部状态栏
        self.create_status_bar()

        # 主标签页容器
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 创建各个标签页
        self.create_mapping_tab()
        self.create_param_tab()

        # 底部日志区域
        self.create_log_area()

        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_status_bar(self):
        """创建顶部状态栏"""
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=5, pady=2)

        self.env_labels = {}
        env_items = [('ros2', 'ROS2'), ('livox', 'LIVOX'), ('ceres', 'Ceres'), ('adaptive_lio', 'Adaptive-LIO')]

        for key, name in env_items:
            self.env_labels[key] = ttk.Label(status_frame, text=f"[?] {name}", foreground='gray')
            self.env_labels[key].pack(side=tk.LEFT, padx=8)

        ttk.Label(status_frame, text="v1.0", foreground='blue').pack(side=tk.RIGHT, padx=5)
        ttk.Button(status_frame, text="检查环境", command=self.check_environment, width=8).pack(side=tk.RIGHT, padx=5)

    # ==================== 建图控制标签页 ====================
    def create_mapping_tab(self):
        """创建建图控制标签页"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  建图控制  ")

        # 左右分栏
        left_frame = ttk.Frame(tab)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_frame = ttk.Frame(tab, width=280)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_frame.pack_propagate(False)

        # ===== 左侧：节点控制 =====
        ctrl_frame = ttk.LabelFrame(left_frame, text="节点控制", padding="8")
        ctrl_frame.pack(fill=tk.X, pady=5)

        btn_row1 = ttk.Frame(ctrl_frame)
        btn_row1.pack(fill=tk.X, pady=3)

        self.btn_start_all = ttk.Button(btn_row1, text="一键启动全部", command=self.start_all, width=15)
        self.btn_start_all.pack(side=tk.LEFT, padx=3)

        self.btn_stop_all = ttk.Button(btn_row1, text="停止全部", command=self.stop_all_nodes, width=10)
        self.btn_stop_all.pack(side=tk.LEFT, padx=3)

        btn_row2 = ttk.Frame(ctrl_frame)
        btn_row2.pack(fill=tk.X, pady=3)

        self.btn_livox = ttk.Button(btn_row2, text="启动LIVOX", command=self.start_livox, width=12)
        self.btn_livox.pack(side=tk.LEFT, padx=3)

        self.btn_lio = ttk.Button(btn_row2, text="启动Adaptive-LIO", command=self.start_adaptive_lio, width=16)
        self.btn_lio.pack(side=tk.LEFT, padx=3)

        self.btn_rviz = ttk.Button(btn_row2, text="启动RViz2", command=self.start_rviz, width=10)
        self.btn_rviz.pack(side=tk.LEFT, padx=3)

        # 节点状态指示
        status_row = ttk.Frame(ctrl_frame)
        status_row.pack(fill=tk.X, pady=5)
        self.node_status_labels = {}
        for name, key in [('LIVOX', 'livox'), ('Adaptive-LIO', 'adaptive_lio'),
                          ('BagPlay', 'bagplay'), ('Record', 'rosbag')]:
            lbl = ttk.Label(status_row, text=f"{name}: --", foreground='gray', width=16)
            lbl.pack(side=tk.LEFT, padx=2)
            self.node_status_labels[key] = lbl

        # 数据包播放
        play_frame = ttk.LabelFrame(left_frame, text="数据包播放", padding="8")
        play_frame.pack(fill=tk.X, pady=5)

        path_row = ttk.Frame(play_frame)
        path_row.pack(fill=tk.X, pady=2)
        self.bag_path_var = tk.StringVar()
        ttk.Entry(path_row, textvariable=self.bag_path_var, width=40).pack(side=tk.LEFT, padx=3)
        ttk.Button(path_row, text="浏览", command=self.browse_bag_path, width=6).pack(side=tk.LEFT)

        play_row = ttk.Frame(play_frame)
        play_row.pack(fill=tk.X, pady=3)
        ttk.Label(play_row, text="速率:").pack(side=tk.LEFT, padx=3)
        self.play_rate_var = tk.StringVar(value="1.0")
        ttk.Combobox(play_row, textvariable=self.play_rate_var, values=["0.5", "1.0", "2.0"], width=5).pack(side=tk.LEFT)
        self.loop_play_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(play_row, text="循环", variable=self.loop_play_var).pack(side=tk.LEFT, padx=10)

        self.btn_start_play = ttk.Button(play_row, text="播放", command=self.start_bag_play, width=8)
        self.btn_start_play.pack(side=tk.LEFT, padx=5)
        self.btn_stop_play = ttk.Button(play_row, text="停止", command=self.stop_bag_play, width=8, state=tk.DISABLED)
        self.btn_stop_play.pack(side=tk.LEFT)

        # 数据录制
        record_frame = ttk.LabelFrame(left_frame, text="数据录制", padding="8")
        record_frame.pack(fill=tk.X, pady=5)

        rec_row = ttk.Frame(record_frame)
        rec_row.pack(fill=tk.X, pady=2)
        self.record_path_var = tk.StringVar(value="/home/li/rosbag")
        ttk.Entry(rec_row, textvariable=self.record_path_var, width=40).pack(side=tk.LEFT, padx=3)
        ttk.Button(rec_row, text="浏览", command=self.browse_record_path, width=6).pack(side=tk.LEFT)

        rec_btn_row = ttk.Frame(record_frame)
        rec_btn_row.pack(fill=tk.X, pady=3)
        self.btn_start_record = ttk.Button(rec_btn_row, text="开始录制", command=self.start_recording, width=10)
        self.btn_start_record.pack(side=tk.LEFT, padx=3)
        self.btn_stop_record = ttk.Button(rec_btn_row, text="停止录制", command=self.stop_recording, width=10, state=tk.DISABLED)
        self.btn_stop_record.pack(side=tk.LEFT, padx=3)
        self.record_status_label = ttk.Label(rec_btn_row, text="未录制", foreground='gray')
        self.record_status_label.pack(side=tk.LEFT, padx=10)

        # 地图保存
        save_frame = ttk.LabelFrame(left_frame, text="地图保存", padding="8")
        save_frame.pack(fill=tk.X, pady=5)

        save_path_row = ttk.Frame(save_frame)
        save_path_row.pack(fill=tk.X, pady=2)
        ttk.Label(save_path_row, text="保存路径:").pack(side=tk.LEFT, padx=3)
        self.map_save_path_var = tk.StringVar(value=str(self.ws_path / "map"))
        ttk.Entry(save_path_row, textvariable=self.map_save_path_var, width=35).pack(side=tk.LEFT, padx=3)
        ttk.Button(save_path_row, text="浏览", command=self.browse_map_save_path, width=6).pack(side=tk.LEFT)

        save_btn_row = ttk.Frame(save_frame)
        save_btn_row.pack(fill=tk.X, pady=3)
        self.btn_save_map = ttk.Button(save_btn_row, text="保存地图", command=self.save_map, width=12)
        self.btn_save_map.pack(side=tk.LEFT, padx=3)
        ttk.Button(save_btn_row, text="打开地图目录", command=self.open_map_folder, width=12).pack(side=tk.LEFT, padx=3)
        self.save_status_label = ttk.Label(save_btn_row, text="", foreground='gray')
        self.save_status_label.pack(side=tk.LEFT, padx=10)

        # ===== 右侧：实时状态监控 =====

        # 算法统计
        stats_frame = ttk.LabelFrame(right_frame, text="算法统计", padding="8")
        stats_frame.pack(fill=tk.X, pady=5)

        self.stats_labels = {}
        stats_items = [
            ('frames', '处理帧数', '0'),
            ('build', '建帧耗时', '-- ms'),
            ('optimize', '优化耗时', '-- ms'),
            ('pose', '位姿估计', '-- ms'),
        ]
        for key, name, default in stats_items:
            row = ttk.Frame(stats_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=10).pack(side=tk.LEFT)
            self.stats_labels[key] = ttk.Label(row, text=default, foreground='blue', width=12, anchor='e')
            self.stats_labels[key].pack(side=tk.RIGHT)

        # 位姿信息
        pose_frame = ttk.LabelFrame(right_frame, text="当前位姿", padding="8")
        pose_frame.pack(fill=tk.X, pady=5)

        self.pose_labels = {}
        for key, name in [('vel', '速度'), ('dist', '里程')]:
            row = ttk.Frame(pose_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=10).pack(side=tk.LEFT)
            self.pose_labels[key] = ttk.Label(row, text="--", foreground='green', width=12, anchor='e')
            self.pose_labels[key].pack(side=tk.RIGHT)

        # 地图信息
        map_info_frame = ttk.LabelFrame(right_frame, text="配置信息", padding="8")
        map_info_frame.pack(fill=tk.X, pady=5)

        self.map_info_labels = {}
        map_items = [
            ('icp_model', 'ICP模型', '--'),
            ('voxel_size', '体素大小', '-- m'),
            ('max_iter', '最大迭代', '--'),
        ]
        for key, name, default in map_items:
            row = ttk.Frame(map_info_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=10).pack(side=tk.LEFT)
            self.map_info_labels[key] = ttk.Label(row, text=default, foreground='purple', width=12, anchor='e')
            self.map_info_labels[key].pack(side=tk.RIGHT)

        # 快捷操作
        quick_frame = ttk.LabelFrame(right_frame, text="快捷操作", padding="8")
        quick_frame.pack(fill=tk.X, pady=5)

        ttk.Button(quick_frame, text="打开地图目录", command=self.open_map_folder, width=15).pack(pady=2)
        ttk.Button(quick_frame, text="刷新配置显示", command=self.reload_config_display, width=15).pack(pady=2)
        ttk.Button(quick_frame, text="打开配置文件", command=self.open_config_file, width=15).pack(pady=2)

    # ==================== 参数调节标签页 ====================
    def create_param_tab(self):
        """创建参数调节标签页"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  参数调节  ")

        # CT-ICP 参数
        cticp_frame = ttk.LabelFrame(tab, text="CT-ICP 参数", padding="10")
        cticp_frame.pack(fill=tk.X, pady=5)

        cticp_params = [
            ('icpmodel', 'ICP模型', 'CT_POINT_TO_PLANE', 'icpmodel'),
            ('max_num_iteration', '最大迭代次数', '10', 'max_num_iteration'),
            ('max_dist_to_plane', '点面距离阈值', '0.3', 'max_dist_to_plane_icp'),
            ('sampling_rate', '采样率', '1.5', 'sampling_rate'),
            ('motion_comp', '运动补偿', 'CONSTANT_VELOCITY', 'motion_compensation'),
            ('init_num_frames', '初始化帧数', '20', 'init_num_frames'),
        ]

        self.cticp_param_vars = {}
        for i, (key, label, default, yaml_key) in enumerate(cticp_params):
            row = i // 2
            col = i % 2

            frame = ttk.Frame(cticp_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky='w')

            ttk.Label(frame, text=label, width=16).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            self.cticp_param_vars[key] = (var, yaml_key)
            if key in ('icpmodel', 'motion_comp'):
                if key == 'icpmodel':
                    values = ['CT_POINT_TO_PLANE', 'POINT_TO_PLANE']
                else:
                    values = ['NONE', 'CONSTANT_VELOCITY', 'ITERATIVE', 'CONTINUOUS']
                ttk.Combobox(frame, textvariable=var, values=values, width=20).pack(side=tk.LEFT, padx=5)
            else:
                ttk.Entry(frame, textvariable=var, width=10).pack(side=tk.LEFT, padx=5)

        # 约束权重参数
        beta_frame = ttk.LabelFrame(tab, text="约束权重 (beta)", padding="10")
        beta_frame.pack(fill=tk.X, pady=5)

        beta_params = [
            ('beta_loc', '位置连续性', '0.001', 'beta_location_consistency'),
            ('beta_orient', '旋转连续性', '0.1', 'beta_orientation_consistency'),
            ('beta_cv', '恒速约束', '0.001', 'beta_constant_velocity'),
            ('beta_sv', '小速度惩罚', '0.01', 'beta_small_velocity'),
        ]

        self.beta_param_vars = {}
        for i, (key, label, default, yaml_key) in enumerate(beta_params):
            row = i // 2
            col = i % 2

            frame = ttk.Frame(beta_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky='w')

            ttk.Label(frame, text=label, width=14).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            self.beta_param_vars[key] = (var, yaml_key)
            ttk.Entry(frame, textvariable=var, width=10).pack(side=tk.LEFT, padx=5)

        # 体素地图参数
        voxel_frame = ttk.LabelFrame(tab, text="体素地图参数", padding="10")
        voxel_frame.pack(fill=tk.X, pady=5)

        voxel_params = [
            ('size_voxel', '体素大小', '0.5', 'size_voxel_map'),
            ('max_distance', '最大距离', '500.0', 'max_distance'),
            ('weight_alpha', '平面度权重', '0.9', 'weight_alpha'),
            ('weight_neig', '邻域权重', '0.1', 'weight_neighborhood'),
            ('power_plan', '平面度指数', '2.0', 'power_planarity'),
            ('max_neighbors', '最大邻居数', '20', 'max_number_neighbors'),
        ]

        self.voxel_param_vars = {}
        for i, (key, label, default, yaml_key) in enumerate(voxel_params):
            row = i // 2
            col = i % 2

            frame = ttk.Frame(voxel_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky='w')

            ttk.Label(frame, text=label, width=14).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            self.voxel_param_vars[key] = (var, yaml_key)
            ttk.Entry(frame, textvariable=var, width=10).pack(side=tk.LEFT, padx=5)

        # 参数说明
        help_frame = ttk.LabelFrame(tab, text="参数说明", padding="10")
        help_frame.pack(fill=tk.X, pady=5)

        help_text = """- CT_POINT_TO_PLANE: 连续时间点到平面ICP，精度高但计算量大
- POINT_TO_PLANE: 标准点到平面ICP，速度快
- beta权重越大约束越强: 位置/旋转连续性防跳变，恒速约束使运动平滑
- 平面度权重(weight_alpha): 控制平面特征点在优化中的影响力
- 运动补偿: CONSTANT_VELOCITY适合匀速, CONTINUOUS适合快速运动场景"""
        ttk.Label(help_frame, text=help_text, justify=tk.LEFT).pack(anchor='w')

        # 按钮
        btn_frame = ttk.Frame(tab)
        btn_frame.pack(fill=tk.X, pady=10)

        ttk.Button(btn_frame, text="应用参数", command=self.apply_params, width=12).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="恢复默认", command=self.reset_params, width=12).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="从文件重载", command=self.reload_params_from_file, width=12).pack(side=tk.LEFT, padx=10)

        self.param_status = ttk.Label(btn_frame, text="", foreground='green')
        self.param_status.pack(side=tk.LEFT, padx=20)

    # ==================== 日志区域 ====================
    def create_log_area(self):
        """创建底部日志区域"""
        log_frame = ttk.LabelFrame(self.root, text="运行日志", padding="5")
        log_frame.pack(fill=tk.X, padx=5, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, wrap=tk.WORD)
        self.log_text.pack(fill=tk.X, expand=True)

        btn_frame = ttk.Frame(log_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="清空", command=self.clear_log, width=8).pack(side=tk.LEFT, padx=3)
        ttk.Button(btn_frame, text="保存", command=self.save_log, width=8).pack(side=tk.LEFT, padx=3)

    # ==================== 工具方法 ====================

    def log(self, message, level="INFO"):
        """添加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] [{level}] {message}\n")
        self.log_text.see(tk.END)

    def clear_log(self):
        self.log_text.delete(1.0, tk.END)

    def save_log(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])
        if filepath:
            with open(filepath, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"日志已保存到: {filepath}")

    def get_source_cmd(self):
        cmd = f"source {self.ros2_setup} && "
        cmd += f"source {self.livox_ws_path}/install/setup.bash && "
        # Source adaptive_lio workspace
        ws_install = self.ws_path.parent.parent / "install" / "setup.bash"
        if ws_install.exists():
            cmd += f"source {ws_install} && "
        return cmd

    def check_environment(self):
        """检查环境依赖"""
        self.log("检查环境依赖...")

        ws_install = self.ws_path.parent.parent / "install"
        checks = [
            ('ros2', os.path.exists(self.ros2_setup)),
            ('livox', (self.livox_ws_path / "install").exists()),
            ('ceres', os.path.exists("/usr/lib/cmake/Ceres/CeresConfig.cmake") or
                      os.path.exists("/usr/lib/x86_64-linux-gnu/cmake/Ceres/CeresConfig.cmake") or
                      os.path.exists("/usr/local/lib/cmake/Ceres/CeresConfig.cmake")),
            ('adaptive_lio', ws_install.exists()),
        ]

        all_ok = True
        for key, ok in checks:
            self.env_labels[key].config(
                text=f"[{'OK' if ok else 'X'}] {key.upper()}",
                foreground='green' if ok else 'red'
            )
            all_ok = all_ok and ok

        self.log("环境检查完成: " + ("全部就绪" if all_ok else "部分缺失"),
                 "SUCCESS" if all_ok else "WARNING")
        self.reload_config_display()

    def reload_config_display(self):
        """重新加载配置文件显示"""
        try:
            if self.config_path.exists():
                with open(self.config_path, 'r') as f:
                    content = f.read()

                # ICP模型
                m = re.search(r'icpmodel:\s*(\S+)', content)
                if m:
                    self.map_info_labels['icp_model'].config(text=m.group(1))

                # 体素大小
                m = re.search(r'size_voxel_map:\s*([\d.]+)', content)
                if m:
                    self.map_info_labels['voxel_size'].config(text=f"{m.group(1)} m")

                # 最大迭代
                m = re.search(r'max_num_iteration:\s*(\d+)', content)
                if m:
                    self.map_info_labels['max_iter'].config(text=m.group(1))

                # 同步更新参数调节页
                self.reload_params_from_file()
        except Exception as e:
            self.log(f"加载配置失败: {e}", "WARNING")

    def update_node_status(self, node, running):
        """更新节点状态显示"""
        if node in self.node_status_labels:
            name_map = {
                'livox': 'LIVOX',
                'adaptive_lio': 'Adaptive-LIO',
                'bagplay': 'BagPlay',
                'rosbag': 'Record'
            }
            name = name_map.get(node, node.upper())
            if running:
                self.node_status_labels[node].config(text=f"{name}: 运行中", foreground='green')
            else:
                self.node_status_labels[node].config(text=f"{name}: 停止", foreground='gray')

    # ==================== 节点控制方法 ====================

    def start_livox(self):
        if self.processes['livox'] is not None:
            self.log("LIVOX驱动已在运行", "WARNING")
            return

        self.log("启动LIVOX驱动...")
        cmd = self.get_source_cmd() + "ros2 launch livox_ros_driver2 msg_MID360_launch.py"

        try:
            env = os.environ.copy()
            self.processes['livox'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                preexec_fn=os.setsid, env=env
            )
            self.update_node_status('livox', True)
            self.log("LIVOX驱动启动成功")
            threading.Thread(target=self.read_process_output,
                             args=(self.processes['livox'], 'LIVOX'), daemon=True).start()
        except Exception as e:
            self.log(f"启动失败: {e}", "ERROR")

    def start_adaptive_lio(self):
        if self.processes['adaptive_lio'] is not None:
            self.log("Adaptive-LIO已在运行", "WARNING")
            return

        self.log("启动Adaptive-LIO节点...")
        # Use ros2 run directly instead of ros2 launch to avoid DDS discovery issues
        cmd = self.get_source_cmd() + "ros2 run adaptive_lio adaptive_lio_node"

        try:
            env = os.environ.copy()
            self.processes['adaptive_lio'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid, env=env
            )
            self.update_node_status('adaptive_lio', True)
            self.log("Adaptive-LIO节点启动成功")
            threading.Thread(target=self.read_process_output,
                             args=(self.processes['adaptive_lio'], 'LIO'), daemon=True).start()
        except Exception as e:
            self.log(f"启动失败: {e}", "ERROR")

    def start_all(self):
        self.log("一键启动...")
        threading.Thread(target=self._start_all_sequence, daemon=True).start()

    def _start_all_sequence(self):
        self.start_livox()
        time.sleep(3)
        self.start_adaptive_lio()
        time.sleep(1)
        self.start_rviz()
        self.root.after(0, lambda: self.log("所有节点启动完成", "SUCCESS"))

    def start_rviz(self):
        if self.processes['rviz'] is not None:
            self.log("RViz2已在运行", "WARNING")
            return

        rviz_config = str(self.ws_path.parent.parent / "install" / "adaptive_lio" / "share" / "adaptive_lio" / "config" / "adaptive_lio.rviz")
        if not os.path.exists(rviz_config):
            self.log("RViz配置文件不存在，使用默认配置", "WARNING")
            rviz_config = ""

        self.log("启动RViz2...")
        rviz_arg = f"-d {rviz_config}" if rviz_config else ""
        cmd = self.get_source_cmd() + f"rviz2 {rviz_arg}"

        try:
            env = os.environ.copy()
            self.processes['rviz'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid, env=env
            )
            self.log("RViz2启动成功")
        except Exception as e:
            self.log(f"RViz2启动失败: {e}", "ERROR")

    def stop_node(self, name):
        if self.processes[name] is not None:
            try:
                os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
                self.processes[name] = None
                self.update_node_status(name, False)
                self.log(f"{name}节点已停止")
            except Exception as e:
                self.log(f"停止失败: {e}", "ERROR")

    def stop_all_nodes(self):
        self.log("停止所有节点...")
        for name in ['rosbag', 'bagplay', 'adaptive_lio', 'rviz', 'livox']:
            if self.processes[name] is not None:
                self.stop_node(name)
                time.sleep(0.3)

        self.is_playing = False
        self.btn_start_play.config(state=tk.NORMAL)
        self.btn_stop_play.config(state=tk.DISABLED)
        self.log("所有节点已停止")

    # ==================== 数据包播放方法 ====================

    def browse_bag_path(self):
        path = filedialog.askdirectory(title="选择ROS2数据包")
        if path:
            self.bag_path_var.set(path)

    def start_bag_play(self):
        if self.is_playing:
            return

        bag_path = self.bag_path_var.get()
        if not bag_path or not os.path.exists(bag_path):
            messagebox.showerror("错误", "请选择有效的数据包路径")
            return

        rate = self.play_rate_var.get()
        loop_flag = "--loop" if self.loop_play_var.get() else ""

        self.log(f"播放数据包: {bag_path}")
        cmd = self.get_source_cmd() + f"ros2 bag play {bag_path} --rate {rate} {loop_flag}"

        try:
            env = os.environ.copy()
            self.processes['bagplay'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid, env=env
            )
            self.is_playing = True
            self.btn_start_play.config(state=tk.DISABLED)
            self.btn_stop_play.config(state=tk.NORMAL)
            self.update_node_status('bagplay', True)
            threading.Thread(target=self._monitor_bag_play, daemon=True).start()
        except Exception as e:
            self.log(f"播放失败: {e}", "ERROR")

    def _monitor_bag_play(self):
        if self.processes['bagplay']:
            self.processes['bagplay'].wait()
            self.root.after(0, self._on_bag_play_finished)

    def _on_bag_play_finished(self):
        if self.is_playing:
            self.is_playing = False
            self.processes['bagplay'] = None
            self.btn_start_play.config(state=tk.NORMAL)
            self.btn_stop_play.config(state=tk.DISABLED)
            self.update_node_status('bagplay', False)
            self.log("数据包播放完成")

    def stop_bag_play(self):
        if not self.is_playing:
            return
        self.stop_node('bagplay')
        self.is_playing = False
        self.btn_start_play.config(state=tk.NORMAL)
        self.btn_stop_play.config(state=tk.DISABLED)

    # ==================== 数据录制方法 ====================

    def browse_record_path(self):
        path = filedialog.askdirectory()
        if path:
            self.record_path_var.set(path)

    def start_recording(self):
        if self.is_recording:
            return

        save_path = self.record_path_var.get()
        os.makedirs(save_path, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"adaptive_lio_{timestamp}"

        self.log(f"开始录制: {bag_name}")
        cmd = self.get_source_cmd() + f"ros2 bag record -o {save_path}/{bag_name} /livox/lidar /livox/imu"

        try:
            self.processes['rosbag'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid
            )
            self.is_recording = True
            self.btn_start_record.config(state=tk.DISABLED)
            self.btn_stop_record.config(state=tk.NORMAL)
            self.record_status_label.config(text="录制中...", foreground='red')
            self.update_node_status('rosbag', True)
        except Exception as e:
            self.log(f"录制失败: {e}", "ERROR")

    def stop_recording(self):
        if not self.is_recording:
            return
        self.stop_node('rosbag')
        self.is_recording = False
        self.btn_start_record.config(state=tk.NORMAL)
        self.btn_stop_record.config(state=tk.DISABLED)
        self.record_status_label.config(text="未录制", foreground='gray')
        self.log("录制已停止")

    # ==================== 地图保存方法 ====================

    def save_map(self):
        if self.processes['adaptive_lio'] is None:
            messagebox.showerror("错误", "Adaptive-LIO节点未运行")
            return

        save_path = self.map_save_path_var.get()
        if not save_path:
            messagebox.showerror("错误", "请指定地图保存路径")
            return

        os.makedirs(save_path, exist_ok=True)
        self.log(f"保存全局地图到: {save_path}")
        self.btn_save_map.config(state=tk.DISABLED, text="保存中...")
        self.save_status_label.config(text="保存中...", foreground='orange')

        threading.Thread(target=self._save_map_async, args=(save_path,), daemon=True).start()

    def _save_map_async(self, save_path):
        cmd = self.get_source_cmd()
        cmd += "ros2 service call /save_map std_srvs/srv/Trigger"

        try:
            result = subprocess.run(cmd, shell=True, executable='/bin/bash',
                                    capture_output=True, text=True, timeout=120)

            stdout = result.stdout if result.stdout else ""
            if "success=true" in stdout.lower() or "success: true" in stdout.lower():
                # Copy the saved PCD to user-specified path if different from default
                default_pcd = str(self.ws_path / "map" / "global_map.pcd")
                target_pcd = os.path.join(save_path, "global_map.pcd")
                if os.path.abspath(save_path) != os.path.abspath(str(self.ws_path / "map")):
                    if os.path.exists(default_pcd):
                        import shutil
                        shutil.copy2(default_pcd, target_pcd)
                self.root.after(0, lambda p=save_path: self._on_save_success(p))
            else:
                self.root.after(0, lambda: self._on_save_error(stdout or "未知错误"))
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._on_save_error(err))

    def _on_save_success(self, save_path):
        self.btn_save_map.config(state=tk.NORMAL, text="保存地图")
        self.save_status_label.config(text="保存成功", foreground='green')
        pcd_path = os.path.join(save_path, "global_map.pcd")
        self.log(f"地图保存成功: {pcd_path}", "SUCCESS")
        messagebox.showinfo("成功", f"地图已保存到:\n{pcd_path}")

    def _on_save_error(self, error_msg):
        self.btn_save_map.config(state=tk.NORMAL, text="保存地图")
        self.save_status_label.config(text="保存失败", foreground='red')
        self.log(f"保存失败: {error_msg}", "ERROR")

    # ==================== 参数方法 ====================

    def apply_params(self):
        """应用所有参数到配置文件"""
        try:
            with open(self.config_path, 'r') as f:
                content = f.read()

            all_params = {}
            all_params.update(self.cticp_param_vars)
            all_params.update(self.beta_param_vars)
            all_params.update(self.voxel_param_vars)

            for key, (var, yaml_key) in all_params.items():
                value = var.get()
                if key in ('icpmodel', 'motion_comp'):
                    content = re.sub(
                        rf'({yaml_key}:\s*)\S+',
                        lambda m, v=value: m.group(1) + v, content)
                else:
                    content = re.sub(
                        rf'({yaml_key}:\s*)[\d.]+',
                        lambda m, v=value: m.group(1) + v, content)

            with open(self.config_path, 'w') as f:
                f.write(content)

            self.param_status.config(text="参数已保存", foreground='green')
            self.log("参数已更新到配置文件")
            self.reload_config_display()
            messagebox.showinfo("成功", "参数已保存，重启Adaptive-LIO后生效")
        except Exception as e:
            self.log(f"保存参数失败: {e}", "ERROR")
            messagebox.showerror("错误", f"保存失败: {e}")

    def reset_params(self):
        """恢复默认参数"""
        defaults_cticp = {
            'icpmodel': 'CT_POINT_TO_PLANE',
            'max_num_iteration': '10',
            'max_dist_to_plane': '0.3',
            'sampling_rate': '1.5',
            'motion_comp': 'CONSTANT_VELOCITY',
            'init_num_frames': '20',
        }
        defaults_beta = {
            'beta_loc': '0.001',
            'beta_orient': '0.1',
            'beta_cv': '0.001',
            'beta_sv': '0.01',
        }
        defaults_voxel = {
            'size_voxel': '0.5',
            'max_distance': '500.0',
            'weight_alpha': '0.9',
            'weight_neig': '0.1',
            'power_plan': '2.0',
            'max_neighbors': '20',
        }
        for key, value in defaults_cticp.items():
            if key in self.cticp_param_vars:
                self.cticp_param_vars[key][0].set(value)
        for key, value in defaults_beta.items():
            if key in self.beta_param_vars:
                self.beta_param_vars[key][0].set(value)
        for key, value in defaults_voxel.items():
            if key in self.voxel_param_vars:
                self.voxel_param_vars[key][0].set(value)
        self.param_status.config(text="已恢复默认", foreground='blue')

    def reload_params_from_file(self):
        """从配置文件重新加载参数"""
        try:
            if not self.config_path.exists():
                return
            with open(self.config_path, 'r') as f:
                content = f.read()

            all_params = {}
            all_params.update(self.cticp_param_vars)
            all_params.update(self.beta_param_vars)
            all_params.update(self.voxel_param_vars)

            for key, (var, yaml_key) in all_params.items():
                if key in ('icpmodel', 'motion_comp'):
                    m = re.search(rf'{yaml_key}:\s*(\S+)', content)
                else:
                    m = re.search(rf'{yaml_key}:\s*([\d.]+)', content)
                if m:
                    var.set(m.group(1))
        except Exception:
            pass

    # ==================== 工具方法 ====================

    def open_map_folder(self):
        path = self.map_save_path_var.get()
        if os.path.exists(path):
            subprocess.run(['xdg-open', path], check=False)
        else:
            messagebox.showwarning("警告", f"目录不存在: {path}")

    def open_config_file(self):
        path = str(self.config_path)
        if os.path.exists(path):
            subprocess.run(['xdg-open', path], check=False)
        else:
            messagebox.showwarning("警告", "配置文件不存在")

    def browse_map_save_path(self):
        path = filedialog.askdirectory(title="选择地图保存目录")
        if path:
            self.map_save_path_var.set(path)

    # ==================== 进程输出读取 ====================

    def read_process_output(self, process, name):
        try:
            for line in iter(process.stdout.readline, b''):
                if line:
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        self.root.after(0, lambda t=text, n=name: self.log(f"[{n}] {t}"))

                        if name == 'LIO':
                            self.parse_lio_output(text)
        except Exception:
            pass

    def parse_lio_output(self, text):
        """解析Adaptive-LIO输出，更新统计面板"""
        try:
            # 帧计数 - 匹配 "poseEstimate" 或 frame index
            if "poseEstimate" in text:
                self.frame_count += 1
                self.root.after(0, lambda c=self.frame_count:
                                self.stats_labels['frames'].config(text=str(c)))

            # 解析耗时统计: [Timer] xxx: avg=xx.xxms
            avg_match = re.search(r'\[Timer\]\s*(\w+).*?avg=([\d.]+)ms', text)
            if avg_match:
                name = avg_match.group(1).lower()
                val = avg_match.group(2)
                if 'build' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['build'].config(text=f"{v} ms"))
                elif 'optim' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['optimize'].config(text=f"{v} ms"))
                elif 'pose' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['pose'].config(text=f"{v} ms"))

            # 解析 build frame / optimize / poseEstimate 耗时
            time_match = re.search(r'(build frame|optimize|poseEstimate).*?:\s*([\d.]+)\s*ms', text)
            if time_match:
                name = time_match.group(1).strip().lower()
                val = time_match.group(2)
                if 'build' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['build'].config(text=f"{v} ms"))
                elif 'optim' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['optimize'].config(text=f"{v} ms"))
                elif 'pose' in name:
                    self.root.after(0, lambda v=val: self.stats_labels['pose'].config(text=f"{v} ms"))

        except Exception:
            pass

    def on_closing(self):
        if messagebox.askokcancel("退出", "确定退出？将停止所有运行中的节点。"):
            self.stop_all_nodes()
            self.root.destroy()


def main():
    root = tk.Tk()
    app = AdaptiveLioLauncher(root)
    root.mainloop()


if __name__ == "__main__":
    main()
