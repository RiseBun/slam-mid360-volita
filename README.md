# SLAM-MID360-Volita

基于 CT-ICP + Ceres 多约束优化的激光惯性里程计系统，专为 Livox MID-360 优化，支持 ROS2 Humble。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-green)](LICENSE)

---

> **重要提示**：在 Orin 或开发板上运行时，务必配置 swap 分区！建议使用 8GB+ swap。可用 `free -h` 检查配置是否成功。

## 功能特点

- **CT-ICP 连续时间运动补偿**：扫描内每个点独立去畸变，高动态场景下精度更高
- **Ceres 多约束优化**：7 种约束因子联合优化，天然抗退化
- **多分辨率体素地图**：3 层分辨率 (0.2m/0.5m/1.2m)，平衡精度与效率
- **双分辨率地图架构**：前端 tracking map 保持轻量快速，输出 dense map 保留全分辨率
- **内存管理**：分段保存 + voxel map 大小限制，支持长时间大范围建图
- **室内 / 室外配置档位**：内置 `--indoor` 与 `--outdoor`，室外高空模式面向 70-80m 远场观测优化
- **边缘设备优化**：内置 Orin NX / Orin Nano 专用配置，支持资源受限平台
- **GUI 控制面板**：可视化参数调节、一键启停、实时状态监控

---

## 目录

- [环境要求](#环境要求)
- [依赖安装](#依赖安装)
- [编译安装](#编译安装)
- [快速开始](#快速开始)
- [配置模式](#配置模式)
- [Dense Map 模式](#dense-map-模式)
- [边缘设备部署](#边缘设备部署)
- [传感器配置](#传感器配置)
- [参数调优](#参数调优)
- [地图管理](#地图管理)
- [性能监控](#性能监控)
- [SDK 打包与分发](#sdk-打包与分发)
- [常见问题](#常见问题)
- [致谢](#致谢)

---

## 环境要求

| 组件 | 版本要求 |
|------|---------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| GCC | 11+ (C++17 支持) |
| CMake | 3.16+ |
| 内存 | 8GB+ 推荐 |

### 支持的硬件平台

| 平台 | 配置文件 | 说明 |
|------|---------|------|
| x86_64 (i5/i7/i9) | `mapping_m.yaml` | 默认配置，全功能 |
| Jetson AGX Orin | `mapping_m.yaml` | 可使用默认配置 |
| **Jetson Orin NX 16GB** | `mapping_orin_nx.yaml` | 专用优化配置 |
| **Jetson Orin Nano 8GB** | `mapping_orin_nano.yaml` | 极限优化配置 |

---

## 依赖安装

### 1. ROS2 Humble

```bash
# 请先安装 ROS2 Humble
# 参考: https://docs.ros.org/en/humble/Installation.html

# 配置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Livox SDK2 与驱动

#### 2.1 安装 Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

#### 2.2 安装 livox_ros_driver2

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

#### 2.3 配置 MID-360

编辑 `~/livox_ws/src/livox_ros_driver2/config/MID360_config.json`，将 `192.168.1.1xx` 替换为 MID-360 实际 IP。

### 3. 系统依赖库

```bash
sudo apt install -y \
  libceres-dev \
  libsuitesparse-dev \
  libpcl-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libyaml-cpp-dev \
  libeigen3-dev \
  libopencv-dev
```

---

## 编译安装

```bash
# 克隆项目
cd ~
git clone https://github.com/RiseBun/slam-mid360-volita.git
cd slam-mid360-volita

# 编译
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
colcon build --symlink-install

# 配置环境
echo "source ~/slam-mid360-volita/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 快速开始

### 方式一：启动脚本（推荐）

```bash
cd ~/slam-mid360-volita/src/adaptive_lio/bash

# 基本启动（带 RViz）
./run_slam.sh

# 播放 rosbag
./run_slam.sh --bag /path/to/rosbag --rate 1.0

# 指定地图保存路径
./run_slam.sh --map-path /tmp/my_map/

# 室内默认模式
./run_slam.sh --indoor --driver --no-rviz --map-path ~/map

# 室外/高空模式，面向 70-80m 远场观测
./run_slam.sh --outdoor --driver --no-rviz --map-path ~/map

# Orin NX 优化模式
./run_slam.sh --orin --no-rviz --bag /path/to/rosbag

# 实时建图（自动启动 Livox 驱动）
./run_slam.sh --orin --driver --no-rviz --map-path ~/map

# 实时建图 + 同时录制 rosbag
./run_slam.sh --driver --record-bag ~/bags --map-path ~/map

# Dense 模式（全分辨率 PCD 输出）
./run_slam.sh --dense --bag /path/to/rosbag --map-path ~/map

# Dense 模式 + 指定体素降采样（合并时降采样，运行时不受影响）
./run_slam.sh --dense 0.02 --bag /path/to/rosbag --map-path ~/map
```

#### 脚本参数说明

| 参数 | 说明 |
|------|------|
| `--rviz` | 启动 RViz2 可视化（默认） |
| `--no-rviz` | 不启动 RViz2 |
| `--indoor` | 使用室内默认配置 `mapping_m.yaml` |
| `--outdoor` / `--high-altitude` | 使用室外/高空配置 `mapping_high_altitude.yaml` |
| `--orin` | 使用 Orin NX 优化配置（16GB） |
| `--orin-nano` | 使用 Orin Nano 极限优化配置（8GB/低内存设备） |
| `--driver` | 启动 Livox MID360 驱动（实时建图时需要） |
| `--dense [VOXEL]` | Dense 模式（可选体素大小，见下文） |
| `--record-bag [PATH]` | 录制 rosbag（可选指定保存目录，默认当前目录） |
| `--config FILE` | 使用指定配置文件 |
| `--map-path PATH` | 指定地图保存路径 |
| `--bag PATH` | 播放指定 rosbag |
| `--rate RATE` | rosbag 播放速率（默认 1.0） |

### 方式二：ROS2 Launch

```bash
# 终端 1: 启动 Livox 驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2: 启动 SLAM
ros2 launch adaptive_lio run.launch.py

# 室外/高空模式
ros2 launch adaptive_lio run.launch.py config_mode:=outdoor
```

### 方式三：GUI 控制面板

```bash
python3 ~/slam-mid360-volita/src/adaptive_lio/scripts/gui_launcher.py
```

---

## 配置模式

### 室内模式 (`--indoor`)

室内模式使用 `config/mapping_m.yaml`，适合近距离、结构密集、平面约束稳定的场景，例如室内房间、走廊、楼梯和中低速平台测试。

主要特点：

- 近场精细地图范围：`near_range: 25.0`
- 邻域点要求更严格：`min_number_neighbors: 20`
- 搜索半径较小：`radius_max: 1.2`
- 更偏向精细、稳定的近距离匹配

### 室外/高空模式 (`--outdoor` / `--high-altitude`)

室外模式使用 `config/mapping_high_altitude.yaml`，面向高空、开阔区域、远距离结构较稀疏的场景。它不是把 MID360 的物理探测距离改大，而是让算法在远场点更稀疏时仍尽量形成可用约束。

当前 outdoor 的距离边界：

| 范围 | 实际含义 |
|------|---------|
| `0-40m` | 使用 near map 精细局部匹配，`near_range: 40.0` |
| `40-80m` | 使用普通 voxel map 参与主建图/定位 |
| `80m` | 正常 outdoor 主建图有效远场，`max_distance: 80.0` |
| `48m` | 内存压力触发 aggressive cleanup 时的可能退化保留半径，来自 `0.6 * max_distance` |

核心参数：

```yaml
map_options:
  neig_options:
    distance_max: 80
    radius_max: 2.5

odometry:
  max_distance: 80.0
  near_range: 40.0
  min_number_neighbors: 10
  max_number_neighbors: 35
```

因此 README 中的“室外有效距离”应理解为：

> 正常 outdoor 有效远场是 80m；内存压力触发 aggressive cleanup 时可能退到 48m。

MID360 标称可达约 100m，但高空 70-80m 是否稳定，仍取决于目标反射率、地面/建筑/树冠结构、姿态变化和点云密度。看得到远点不等于这些远点一定能稳定支撑定位建图。

---

## Dense Map 模式

### 架构：前端稀疏，输出稠密

本项目采用双分辨率地图架构，将 SLAM 跟踪与地图输出解耦：

```
LiDAR 点云
    |
    v
CT-ICP 配准 (tracking map, 降采样, 快速)
    |
    +---> 位姿估计 ---> /odom, /scan
    |
    +---> dense_buffer_ (原始点, 无降采样, 后台分段写盘)
              |
              v
         退出时合并 ---> global_map.pcd
```

| 组件 | Tracking Map | Dense Map |
|------|-------------|-----------|
| 用途 | ICP 配准匹配 | 最终点云输出 |
| 体素大小 | 0.5m / 0.25m (多分辨率) | 可配置 (0=不降采样) |
| 内存占用 | 滑动窗口自动裁剪 | 分段写盘, 峰值 ~32MB |
| CPU 开销 | CT-ICP 主要开销 | < 0.5ms/帧 (vector push_back) |

### 使用方法

```bash
# 全分辨率，不降采样（适合后期处理）
./run_slam.sh --dense --bag /path/to/bag --map-path ~/map

# 2cm 体素降采样（适合直接可视化查看）
./run_slam.sh --dense 0.02 --bag /path/to/bag --map-path ~/map

# 1cm 体素降采样（高精度 + 可直接查看）
./run_slam.sh --dense 0.01 --bag /path/to/bag --map-path ~/map
```

### 降采样参数选择

| `--dense` 参数 | 说明 | 预估文件大小 (30min 数据) |
|---|---|---|
| (不带参数) | 原始全量点 | ~3-5GB |
| `0.01` | 1cm 体素 | ~300-500MB |
| `0.02` | 2cm 体素，pcl_viewer 可直接打开 | ~80-150MB |

### 工作原理

1. **运行期间**：每帧点云在 `cloud_pub_func` 中以原始分辨率追加到 `dense_buffer_`
2. **分段保存**：buffer 达到 200 万点 (~32MB) 时，后台线程异步写出 `dense_segment_N.pcd`
3. **退出合并**：Ctrl+C 后自动执行流式合并（逐段读取，内存占用 ~1MB），若指定了体素大小则逐段降采样
4. **SLAM 零影响**：dense 路径不加锁，不影响前端配准速度

### 相关配置 (mapping_m.yaml)

```yaml
common:
  dense_map_mode: false              # 也可在 YAML 中直接启用
  dense_save_segment_points: 2000000 # 每段点数 (~32MB/段)
  dense_voxel_size: 0.0              # 合并降采样: 0=原始, 0.01=1cm, 0.02=2cm
```

---

## 边缘设备部署

### Jetson Orin NX 16GB 配置

```bash
# 使用 Orin NX 优化模式启动
./run_slam.sh --orin --no-rviz

# 实时建图（自动启动 Livox 驱动）
./run_slam.sh --orin --driver --no-rviz --map-path ~/map

# Dense 模式 + Orin NX
./run_slam.sh --orin --dense 0.02 --driver --no-rviz --map-path ~/map
```

#### Orin NX 配置优化项

| 参数 | 默认值 | Orin NX 值 | 说明 |
|------|--------|-----------|------|
| point_filter_num | 1 | 2 | 每2点取1，减少输入 |
| map_voxel_size | 0.05 | 0.1 | 降低地图密度 |
| max_num_iteration | 10 | 6 | 减少ICP迭代 |
| ceres_iterations | 5 | 3 | 减少Ceres迭代 |
| ceres_threads | 3 | 4 | 利用8核CPU |
| max_num_residuals | 1200 | 800 | 减少残差数量 |
| max_distance | 80 | 60 | 更激进FOV清理 |

### Jetson Orin Nano 8GB / 低内存设备配置

```bash
# 使用 Orin Nano 极限优化模式启动
./run_slam.sh --orin-nano --no-rviz

# 实时建图
./run_slam.sh --orin-nano --driver --no-rviz --map-path ~/map
```

#### Orin Nano 配置优化项（相比 Orin NX 进一步压缩）

| 参数 | Orin NX | Orin Nano | 说明 |
|------|---------|-----------|------|
| point_filter_num | 2 | **3** | 每3点取1，输入再减33% |
| map_voxel_size | 0.1 | **0.15** | 更稀疏的输出地图 |
| max_num_iteration | 6 | **4** | ICP迭代再减33% |
| surf_res | 0.6 | **0.8** | 更少keypoints |
| size_voxel_map | 0.6 | **0.8** | 更大voxel，更少内存 |
| max_num_points_in_voxel | 15 | **10** | 每voxel存更少的点 |
| max_num_residuals | 800 | **500** | Ceres残差再减37% |
| max_number_neighbors | 15 | **10** | 邻域搜索量减33% |
| max_distance | 60 | **40** | 更激进FOV清理 |

#### 三档配置对比

| 指标 | 默认 (x86) | Orin NX (16GB) | Orin Nano (8GB) |
|------|-----------|----------------|-----------------|
| 目标帧率 | 10Hz 实时 | 接近实时 | 接近实时 |
| 内存占用 | ~3GB | ~2GB | <4GB（避免swap） |
| 地图精度 | 最高 | 高 | 中等 |
| 适用场景 | 桌面/服务器 | Orin NX 16GB | Orin Nano 8GB / 低内存 |

#### 部署建议

1. **配置 swap**：至少 8GB swap
   ```bash
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

2. **低内存设备编译**：内存不足时限制编译线程
   ```bash
   MAKEFLAGS="-j1" colcon build --symlink-install
   ```

3. **测试时降低速率**：先用 `--rate 0.5` 测试稳定性

4. **关闭 RViz**：边缘设备上使用 `--no-rviz` 节省资源

5. **监控资源**：运行时观察 MONITOR 日志中的 RSS 和 frame_ms

---

## 传感器配置

### 支持的传感器

| 传感器类型 | lidar_type 值 | 话题格式 |
|-----------|---------------|---------|
| Livox AVIA/MID-360 | 1 | livox_ros_driver2/CustomMsg |
| Velodyne | 2 | sensor_msgs/PointCloud2 |
| Ouster | 3 | sensor_msgs/PointCloud2 |
| RoboSense | 4 | sensor_msgs/PointCloud2 |
| Hesai Pandar | 5 | sensor_msgs/PointCloud2 |

### Livox MID-360 配置

编辑 `config/mapping_m.yaml`：

```yaml
preprocess:
  lidar_type: 1          # Livox 系列
  blind: 0.1             # 近处盲区 (m)
  point_filter_num: 1    # 点云抽稀 (1=不抽稀, 2=每2点取1)

common:
  imu_topic: /livox/imu
  lid_topic: /livox/lidar
  gnorm: 1               # 加速度缩放因子
```

---

## 参数调优

### 核心参数 (config/mapping_m.yaml)

#### CT-ICP 配置

```yaml
odometry:
  icpmodel: CT_POINT_TO_PLANE    # CT_POINT_TO_PLANE(精确) / POINT_TO_PLANE(快速)
  max_num_iteration: 10          # ICP 最大迭代次数
  ceres_iterations: 5            # Ceres 优化迭代次数
  ceres_threads: 3               # Ceres 线程数
  max_dist_to_plane_icp: 0.3     # 点到平面距离阈值
  motion_compensation: CONSTANT_VELOCITY  # 运动补偿模式
```

#### 地图保存

```yaml
common:
  map_save_path: ""      # 空=默认路径，或指定绝对路径
  map_voxel_size: 0.05   # 稀疏模式降采样体素大小 (m)

  # Dense 模式 (--dense 或 dense_map_mode: true)
  dense_map_mode: false
  dense_save_segment_points: 2000000   # 每段点数
  dense_voxel_size: 0.0                # 合并降采样 (0=不降采样)
```

### 场景调参建议

| 场景 | 建议调整 |
|------|---------|
| **楼梯/退化场景** | `beta_orientation_consistency: 0.5`<br>`motion_compensation: CONTINUOUS` |
| **快速运动** | `max_dist_to_plane_icp: 0.2`<br>`max_num_iteration: 15` |
| **大范围建图** | `max_distance: 100`<br>`map_voxel_size: 0.1` |
| **高精度地图输出** | `--dense` 或 `--dense 0.01` |
| **边缘设备** | 使用 `--orin` / `--orin-nano` 或对应配置文件 |

---

## 地图管理

### 保存地图

```bash
# 运行中手动保存（会合并所有 segment）
ros2 service call /save_map std_srvs/srv/Trigger

# 检查保存状态
ros2 service call /save_map_status std_srvs/srv/Trigger
```

> Ctrl+C 停止节点时会自动保存并合并地图。

### 查看地图

```bash
# PCL Viewer（文件较大时用单个 segment 查看）
pcl_viewer map/global_map.pcd

# 或查看单个分段
pcl_viewer map/dense_segment_0.pcd
```

### 输出文件

#### 稀疏模式（默认）

```
map/
├── segment_0.pcd      # 分段点云
├── segment_1.pcd
├── global_map.pcd     # 合并地图 (降采样)
└── trajectory.txt     # TUM 格式轨迹
```

#### Dense 模式 (`--dense`)

```
map/
├── dense_segment_0.pcd    # 原始分段 (~31MB/段, 200万点)
├── dense_segment_1.pcd
├── ...
├── global_map.pcd         # 合并地图 (按 dense_voxel_size 降采样或原始)
└── trajectory.txt         # TUM 格式轨迹
```

> 轨迹文件随 `--map-path` 保存到同一目录，可直接用于 evo 等工具评估精度。

---

## 性能监控

系统内置实时性能监控，每 10 帧输出一次：

```
[MONITOR] frame=1000  frame_ms=85.3  RSS=512.5MB  voxel_map=15000  mmap_voxels=50000  mmap_points=500000
```

| 指标 | 说明 | 健康范围 |
|------|------|---------|
| frame_ms | 单帧处理时间 | <100ms (10Hz实时) |
| RSS | 进程内存占用 | <4GB |
| voxel_map | FOV内体素数 | 应稳定或缓慢增长 |
| mmap_points | 多分辨率地图点数 | 应有波动（清理生效） |

### 异常诊断

- **frame_ms 持续 >150ms**：考虑使用 `--orin` 或 `--orin-nano` 优化配置
- **RSS 线性增长**：检查 max_distance 是否过大
- **mmap_points 只增不减**：FOV 清理可能失效

---

## ROS2 接口

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | PointCloud2 | 当前帧点云 |
| `/odom` | Odometry | 里程计 |
| `/odometry_path` | Path | 累积轨迹 |
| `/velocity` | Float32 | 当前速度 |
| `/move_dist` | Float32 | 累计移动距离 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/save_map` | Trigger | 保存地图 |
| `/save_map_status` | Trigger | 查询保存状态 |

---

## SDK 打包与分发

项目提供 `pack_sdk.sh` 脚本，在编译完成后一键打包为可分发的 SDK，无需源码即可在同架构设备上运行。

### 打包前提

1. 已完成 `colcon build` 编译
2. 已安装 livox_ros_driver2（默认 `~/livox_ws`，可通过 `LIVOX_WS_PATH` 环境变量指定）

### 打包脚本用法

```bash
cd ~/slam-mid360-volita

# 基本打包 (输出到 ./sdk_arm64/ 或 ./sdk_x86_64/，自动检测架构)
./pack_sdk.sh

# 指定输出目录
./pack_sdk.sh -o ~/my_sdk

# 打包并生成 tar.gz 压缩包（方便 scp 传输）
./pack_sdk.sh --tar
```

### 部署到其他设备

```bash
# 传输
scp -r sdk_arm64/ user@target:~/sdk/

# 目标设备运行
source /opt/ros/humble/setup.bash
cd ~/sdk
./check_env.sh            # 环境检查
./run.sh --orin --driver --no-rviz --map-path ~/map
```

> 详见 `pack_sdk.sh -h` 获取完整参数说明。

---

## 目录结构

```
slam-mid360-volita/
├── src/adaptive_lio/
│   ├── config/
│   │   ├── mapping_m.yaml           # 默认配置 (x86/高性能设备)
│   │   ├── mapping_high_altitude.yaml # 室外/高空配置
│   │   ├── mapping_orin_nx.yaml     # Orin NX 16GB 优化配置
│   │   ├── mapping_orin_nano.yaml   # Orin Nano 8GB 极限优化配置
│   │   └── adaptive_lio.rviz        # RViz 配置
│   ├── bash/
│   │   └── run_slam.sh              # 启动脚本
│   ├── launch/
│   │   └── run.launch.py            # ROS2 Launch
│   ├── scripts/
│   │   └── gui_launcher.py          # GUI 控制面板
│   ├── src/
│   │   ├── apps/main_ros2.cpp       # ROS2 节点 (含 dense map 逻辑)
│   │   ├── lio/                     # 核心 LIO 算法
│   │   ├── algo/                    # ESKF 滤波器
│   │   └── common/                  # 工具库
│   └── map/                         # 地图输出 (gitignored)
├── pack_sdk.sh                      # SDK 打包脚本
├── .gitignore
└── README.md
```

---

## 技术架构

```
IMU数据 ──> ESKF状态预测 ──> 点云去畸变 ──> CT-ICP Ceres优化 ──> 多分辨率地图更新
              │                                  │                      │
          高频积分预测                    连续时间运动建模          自适应权重匹配
                                                                       │
                                                              ┌────────┴────────┐
                                                              │                 │
                                                        Tracking Map      Dense Map
                                                       (降采样, 快速)   (原始分辨率, 分段写盘)
```

### 与 FAST-LIO2 对比

| 维度 | FAST-LIO2 | 本项目 |
|------|-----------|--------|
| 优化方式 | IESKF (线性化) | Ceres (真非线性) |
| 运动模型 | 离散帧间 | 连续时间 (CT) |
| 地图结构 | ikd-tree | 多分辨率 Voxel 哈希 |
| 约束数量 | 1-2 种 | 7 种 |
| 地图输出 | 单一分辨率 | 双分辨率 (tracking + dense) |
| 边缘设备 | 需修改 | 内置 Orin 配置 |

---

## 常见问题

### Q: 编译时找不到 livox_ros_driver2

```bash
source ~/livox_ws/install/setup.bash
colcon build
```

### Q: 点云显示但位姿不更新

1. 检查 IMU 数据：`ros2 topic echo /livox/imu`
2. 确认 `gnorm` 参数正确
3. 保持静止 2-3 秒等待初始化

### Q: 地图保存后点很稀疏

使用 Dense 模式获取全分辨率输出：
```bash
./run_slam.sh --dense --bag /path/to/bag
```
或调小稀疏模式的 `map_voxel_size`：
```yaml
common:
  map_voxel_size: 0.02
```

### Q: Dense 模式 global_map.pcd 太大，pcl_viewer 打不开

指定合并降采样体素大小，或直接查看单个分段：
```bash
# 方法 1: 用 2cm 降采样运行
./run_slam.sh --dense 0.02 --bag /path/to/bag

# 方法 2: 直接查看单个分段 (~2M 点，可正常打开)
pcl_viewer map/dense_segment_0.pcd
```

### Q: 长时间运行内存不足

1. 使用 `--orin` 或 `--orin-nano` 优化模式
2. 增大 `map_voxel_size`
3. 减小 `max_distance`
4. 配置 swap（至少 8GB）

### Q: MID-360 连接不上

1. 检查网络：电脑 IP 应为 `192.168.1.50`
2. 确认 MID360_config.json 中 IP 正确
3. 放行 56100-56500 端口

---

## 致谢

本项目基于以下开源项目：

- [Adaptive-LIO](https://github.com/chengwei0427/Adaptive-LIO) - 原始 ROS1 实现
- [CT-ICP](https://github.com/jedeschaud/ct_icp) - 连续时间 ICP 算法
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) - Livox 官方驱动

---

## License

BSD-3-Clause License

---

## DP180-Pro + MID360 联合运行、PTP 时间同步与位姿仲裁

本节记录当前 Jetson 同时连接 DP180-Pro 相机和 Livox MID-360 雷达时的完整配置。目标不是让相机替代雷达建图，而是让雷达 SLAM 正常建图；当雷达里程计与相机 VIO 偏差过大时，系统切换为相信相机位姿，并继续把雷达点云累计到受保护地图 `/guarded_map`。

### 1. 当前硬件与网络拓扑

Jetson 主机：

- 用户：`li`
- 主机名：`ubuntu`
- 管理网络：`192.168.137.3`
- ROS2：Humble

DP180-Pro 相机：

- 设备 IP：`10.42.0.64`
- Jetson 连接相机的网口：`enP8p1s0`
- Jetson 在相机网段的 IP：`10.42.0.1/24`
- SSH 用户：`linaro`
- SSH 密码：`linaro`

MID-360 雷达：

- 雷达 IP：`192.168.1.118`
- Jetson USB 转网口：`enx9c69d34bdc5e`
- Jetson 在雷达网段的 IP：`192.168.1.5/24`

检查链路：

```bash
ping 10.42.0.64
ping 192.168.1.118
ip -br addr
```

### 2. DP180-Pro 登录账号来源

最初尝试 `compulab/compulab` 登录 DP180-Pro，但 SSH 认证失败。随后在设备 SSH 端口开放、且没有其它可用控制命令能进入系统的情况下，测试常见板卡出厂账号，`linaro/linaro` 登录成功。登录后确认：

```bash
hostname
# linaro-alip

id
# uid=1000(linaro) ... groups=..., sudo, ...
```

因此当前 DP180-Pro 的实际系统账号是：

```bash
ssh linaro@10.42.0.64
# password: linaro
```

### 3. 为什么必须先修 PTP

`pose_guard_mapper` 默认启用严格时间安全检查：

```yaml
require_header_time_near_now: true
max_wall_stamp_skew_sec: 5.0
```

也就是说，相机 VIO 的 `header.stamp` 必须接近 Jetson 当前系统时间，否则仲裁节点不会相信相机。之前的问题现象是：

- `/S1/imu`、`/S1/vio_odom` 都有频率，说明相机和 ROS 桥接不是根因；
- 但 `/S1/vio_odom` 的时间戳和 Jetson 当前时间相差约 372 天；
- `pose_guard_mapper` 输出 `camera_ok=no`、`comparable=no`。

根因是 DP180-Pro 的 Time Synchronizer 处于 `default` 模式，未作为 Jetson 的 PTP slave；同时 DP180-Pro 缺少 `bc`，导致 Vilota 自带脚本 `/opt/vilota/bin/vk_time_slave_ptp_software` 无法完成同步状态判断。

### 4. Jetson 侧 PTP master 配置

Jetson 使用软件时间戳模式作为 DP180-Pro 的 PTP grandmaster。服务文件：

```bash
/etc/systemd/system/dp180-ptp-master.service
```

内容：

```ini
[Unit]
Description=DP180-Pro PTP master on Jetson camera Ethernet
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -2 -i enP8p1s0 -S -m --step_threshold=1 -f /opt/vilota/configs/time/ptp4l_master.conf
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now dp180-ptp-master.service
```

检查：

```bash
systemctl is-active dp180-ptp-master.service
journalctl -u dp180-ptp-master.service -f
```

正常日志应包含：

```text
selected local clock ... as best master
port 1: assuming the grand master role
```

### 5. DP180-Pro 侧 Time Synchronizer 配置

DP180-Pro 使用 Vilota Manager 的 websocket/capnp 控制接口设置时间同步。核心设置：

- `TimeRole = slaveSoftware`
- `Autostart = true`
- reload time synchronizer

设置完成后查询结果应为：

```text
status: runningSynchronized
optRole: slaveSoftware
optAutostart: True
```

如果需要重新执行，可在 Jetson 上运行下面的 Python 脚本：

```bash
python3 - <<'PY'
import sys, asyncio, capnp
sys.path.append('/opt/vilota/messages')
capnp.add_import_hook()
import system_capnp
from websockets.asyncio.client import connect

async def send(ws, marker, field, value=None):
    cmd = system_capnp.ManagerCommand.new_message()
    cmd.marker = marker
    t = cmd.variant.init('time')
    if field == 'optRole':
        t.variant.optRole = value
    elif field == 'optAutostart':
        t.variant.optAutostart = value
    elif field == 'reload':
        t.variant.reload = None
    elif field == 'fetchSettings':
        t.variant.fetchSettings = None
    await ws.send(cmd.to_bytes())

async def main():
    async with connect('ws://10.42.0.64/socket', max_size=None) as ws:
        await send(ws, 201, 'optRole', 'slaveSoftware')
        await send(ws, 202, 'optAutostart', True)
        await send(ws, 203, 'reload')
        await send(ws, 204, 'fetchSettings')

asyncio.run(main())
PY
```

Jetson 如缺少依赖：

```bash
python3 -m pip install --user pycapnp websockets
```

DP180-Pro 缺少 `bc` 时，Vilota 时间脚本会报：

```text
/opt/vilota/bin/vk_time_slave_ptp_software: line 40: bc: command not found
```

修复方式是在 DP180-Pro 上安装 `bc`。如果 DP180-Pro 没有外网/DNS，可从其它机器下载 `bc_1.07.1-3_arm64.deb` 后传入：

```bash
scp bc_1.07.1-3_arm64.deb linaro@10.42.0.64:/tmp/
ssh linaro@10.42.0.64
sudo dpkg -i /tmp/bc_1.07.1-3_arm64.deb
which bc
```

注意：这里没有修改 DP180-Pro 的相机/VIO 源码，只做了系统工具安装和 Time Synchronizer 持久化设置。

### 6. DP180-Pro 相机与 VIO 启动

PTP 必须先稳定，再启动相机和 VIO。否则 VIO 在运行过程中遇到系统时间跳变，位姿会出现极大漂移。

推荐顺序：

```bash
systemctl is-active dp180-ptp-master.service

/opt/vilota/bin/vk_system_control -r 10.42.0.64 camera reload 0 vk180-pro_light_rectified.json
/opt/vilota/bin/vk_system_control -r 10.42.0.64 vio reload 0 vk180-pro_moderate_rectified.json
```

启动 ROS2 桥接：

```bash
tmux new-session -d -s dp180_bridge \
  "bash -lc 'source /opt/ros/humble/setup.bash && source /home/li/dp180_ws/install/setup.bash && ros2 launch vk_ros2_driver example.launch.py'"
```

检查相机 ROS 话题：

```bash
source /opt/ros/humble/setup.bash
source /home/li/dp180_ws/install/setup.bash

ros2 topic list | grep S1
ros2 topic hz /S1/imu
ros2 topic hz /S1/vio_odom
ros2 topic echo /S1/vio_odom --once
```

正常频率大致为：

- `/S1/imu`：约 190-200 Hz
- `/S1/vio_odom`：约 15-16 Hz
- `/S1/camd`、`/S1/stereo*`：约 16 Hz

正常时间戳应接近 Jetson 当前 epoch：

```bash
date '+JETSON epoch=%s'
ros2 topic echo /S1/vio_odom --once
```

### 7. MID-360 雷达配置

Livox 配置文件：

```bash
/home/li/livox_ws/src/livox_ros_driver2/config/MID360_config.json
/home/li/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

当前应配置为：

- host IP：`192.168.1.5`
- lidar IP：`192.168.1.118`

检查：

```bash
grep -R "192.168.1.5\|192.168.1.118" \
  /home/li/livox_ws/src/livox_ros_driver2/config/MID360_config.json \
  /home/li/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

如果 USB 网口重启后没有 IP，临时设置：

```bash
sudo ip addr add 192.168.1.5/24 dev enx9c69d34bdc5e
sudo ip link set enx9c69d34bdc5e up
```

检查雷达：

```bash
ping 192.168.1.118
```

仅修改 Livox JSON 配置通常不需要重新编译驱动；修改源码或消息定义才需要重新编译。

### 8. 位姿仲裁功能

新增包：

```bash
/home/li/slam-mid360-volita/src/pose_guard_mapper
```

主要文件：

```bash
src/pose_guard_mapper/src/pose_guard_mapper_node.cpp
src/pose_guard_mapper/config/pose_guard_mapper.yaml
src/pose_guard_mapper/launch/pose_guard_mapper.launch.py
src/pose_guard_mapper/scripts/check_pose_guard_inputs.sh
src/pose_guard_mapper/scripts/start_dp180_ptp_master.sh
```

输出：

```bash
/trusted_odom
/guarded_map
/pose_guard/status
/pose_guard/save_map
```

仲裁逻辑：

- 正常情况下相信 LiDAR SLAM；
- 当 LiDAR 与 DP180 VIO 的位姿偏差超过阈值时，切换为相信 camera；
- 点云仍来自 MID-360；
- `/guarded_map` 使用当前可信位姿累计点云；
- 如果相机时间戳不接近 Jetson 当前时间，直接判定 `camera_ok=no`，避免错误融合。

重新编译：

```bash
cd /home/li/slam-mid360-volita
source /opt/ros/humble/setup.bash
source /home/li/livox_ws/install/setup.bash
source /home/li/dp180_ws/install/setup.bash
colcon build --packages-select pose_guard_mapper --symlink-install
```

### 9. 一键启动雷达 SLAM + 仲裁

启动脚本已整合：

```bash
/home/li/slam-mid360-volita/src/adaptive_lio/bash/run_slam.sh
```

该脚本现在会自动 source：

```bash
source /opt/ros/humble/setup.bash
source /home/li/livox_ws/install/setup.bash
source /home/li/dp180_ws/install/setup.bash
source /home/li/slam-mid360-volita/install/setup.bash
```

默认启用 `pose_guard_mapper`，并在终端打印当前相信谁：

```text
[POSE_GUARD] 当前相信: LIDAR
[POSE_GUARD] 当前相信: CAMERA
```

启动：

```bash
cd /home/li/slam-mid360-volita
./src/adaptive_lio/bash/run_slam.sh --orin --driver --no-rviz --map-path /home/li/slam-mid360-volita/map
```

后台启动：

```bash
tmux new-session -d -s slam_guard \
  "bash -lc 'cd /home/li/slam-mid360-volita && ./src/adaptive_lio/bash/run_slam.sh --orin --driver --no-rviz --map-path /home/li/slam-mid360-volita/map'"
```

查看：

```bash
tmux attach -t slam_guard
```

只启动原雷达 SLAM、不启用仲裁：

```bash
./src/adaptive_lio/bash/run_slam.sh --orin --driver --no-rviz --no-guard
```

### 10. 推荐完整启动顺序

每次上电后按这个顺序最稳：

```bash
# 1. 确认 Jetson PTP master 正常
systemctl is-active dp180-ptp-master.service

# 2. 确认 DP180 时间同步正常
python3 - <<'PY'
import sys, asyncio, capnp
sys.path.append('/opt/vilota/messages')
capnp.add_import_hook()
import system_capnp
from websockets.asyncio.client import connect

async def main():
    async with connect('ws://10.42.0.64/socket', max_size=None) as ws:
        cmd = system_capnp.ManagerCommand.new_message()
        cmd.marker = 1
        t = cmd.variant.init('time')
        t.variant.fetchSettings = None
        await ws.send(cmd.to_bytes())
        for _ in range(8):
            msg = await ws.recv()
            with system_capnp.ManagerMessage.from_bytes(msg) as m:
                print(m.to_dict())

asyncio.run(main())
PY

# 3. 启动 DP180 camera/VIO
/opt/vilota/bin/vk_system_control -r 10.42.0.64 camera reload 0 vk180-pro_light_rectified.json
/opt/vilota/bin/vk_system_control -r 10.42.0.64 vio reload 0 vk180-pro_moderate_rectified.json

# 4. 启动 DP180 ROS2 bridge
tmux new-session -d -s dp180_bridge \
  "bash -lc 'source /opt/ros/humble/setup.bash && source /home/li/dp180_ws/install/setup.bash && ros2 launch vk_ros2_driver example.launch.py'"

# 5. 启动 MID-360 雷达 SLAM + 位姿仲裁
tmux new-session -d -s slam_guard \
  "bash -lc 'cd /home/li/slam-mid360-volita && ./src/adaptive_lio/bash/run_slam.sh --orin --driver --no-rviz --map-path /home/li/slam-mid360-volita/map'"
```

### 11. 运行中验证

相机：

```bash
source /opt/ros/humble/setup.bash
source /home/li/dp180_ws/install/setup.bash
ros2 topic hz /S1/imu
ros2 topic hz /S1/vio_odom
ros2 topic echo /S1/vio_odom --once
```

雷达：

```bash
source /opt/ros/humble/setup.bash
source /home/li/livox_ws/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /odom
```

仲裁：

```bash
source /opt/ros/humble/setup.bash
source /home/li/slam-mid360-volita/install/setup.bash
ros2 topic echo /pose_guard/status
ros2 topic hz /trusted_odom
```

正常状态示例：

```text
source=lidar reason=lidar camera_ok=yes comparable=yes aligned=yes
```

含义：

- `source=lidar`：当前相信雷达；
- `camera_ok=yes`：相机 VIO 数据存在且时间戳可信；
- `comparable=yes`：雷达和相机位姿可以比较；
- `aligned=yes`：两者偏差未超过阈值；
- `source=camera`：雷达和相机偏差过大时，当前相信相机。

### 12. 常见故障定位

#### `/S1/*` 有频率，但 `camera_ok=no`

优先检查时间戳：

```bash
date '+JETSON epoch=%s'
ros2 topic echo /S1/vio_odom --once
```

如果 stamp 和 Jetson 当前时间差超过几秒，先修 PTP，不要调仲裁阈值。

#### DP180 Time Sync 一直 `runningWaiting`

检查 DP180 是否缺少 `bc`：

```bash
ssh linaro@10.42.0.64
which bc
```

检查 Jetson master：

```bash
systemctl status dp180-ptp-master.service
journalctl -u dp180-ptp-master.service -n 50
```

检查 DP180 slave：

```bash
ssh linaro@10.42.0.64
ps -ef | grep -E 'ptp4l|vk_time' | grep -v grep
ls -l /run/lock/vk_timesync
```

#### VIO 位姿突然变成十几万米

这是 VIO 运行中遇到系统时间跳变造成的状态污染。处理方式：

```bash
/opt/vilota/bin/vk_system_control -r 10.42.0.64 camera reload 0 vk180-pro_light_rectified.json
/opt/vilota/bin/vk_system_control -r 10.42.0.64 vio reload 0 vk180-pro_moderate_rectified.json
tmux kill-session -t dp180_bridge
tmux new-session -d -s dp180_bridge \
  "bash -lc 'source /opt/ros/humble/setup.bash && source /home/li/dp180_ws/install/setup.bash && ros2 launch vk_ros2_driver example.launch.py'"
```

然后重启雷达 SLAM + 仲裁：

```bash
tmux kill-session -t slam_guard
tmux new-session -d -s slam_guard \
  "bash -lc 'cd /home/li/slam-mid360-volita && ./src/adaptive_lio/bash/run_slam.sh --orin --driver --no-rviz --map-path /home/li/slam-mid360-volita/map'"
```

#### 雷达连不上

```bash
ip -br addr show enx9c69d34bdc5e
ping 192.168.1.118
```

如果 USB 网口没有 `192.168.1.5/24`：

```bash
sudo ip addr add 192.168.1.5/24 dev enx9c69d34bdc5e
sudo ip link set enx9c69d34bdc5e up
```

#### 保存地图

原始雷达地图：

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

仲裁后地图：

```bash
ros2 service call /pose_guard/save_map std_srvs/srv/Trigger
```
