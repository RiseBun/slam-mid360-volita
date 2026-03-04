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
- **内存管理**：分段保存机制，支持长时间大范围建图
- **边缘设备优化**：内置 Orin NX 专用配置，支持资源受限平台
- **GUI 控制面板**：可视化参数调节、一键启停、实时状态监控

---

## 目录

- [环境要求](#环境要求)
- [依赖安装](#依赖安装)
- [编译安装](#编译安装)
- [快速开始](#快速开始)
- [边缘设备部署](#边缘设备部署)
- [传感器配置](#传感器配置)
- [参数调优](#参数调优)
- [地图管理](#地图管理)
- [性能监控](#性能监控)
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
| Jetson Orin NX 8GB | 需进一步优化 | 内存受限 |

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

# 不启动 RViz
./run_slam.sh --no-rviz

# 播放 rosbag
./run_slam.sh --bag /path/to/rosbag --rate 1.0

# 指定地图保存路径
./run_slam.sh --map-path /tmp/my_map/

# Orin NX 优化模式
./run_slam.sh --orin --no-rviz --bag /path/to/rosbag

# 使用自定义配置
./run_slam.sh --config /path/to/custom.yaml
```

#### 脚本参数说明

| 参数 | 说明 |
|------|------|
| `--rviz` | 启动 RViz2 可视化（默认） |
| `--no-rviz` | 不启动 RViz2 |
| `--orin` | 使用 Orin NX 优化配置 |
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
```

### 方式三：GUI 控制面板

```bash
python3 ~/slam-mid360-volita/src/adaptive_lio/scripts/gui_launcher.py
```

---

## 边缘设备部署

### Jetson Orin NX 16GB 配置

系统内置了针对 Orin NX 16GB 优化的配置文件：

```bash
# 使用 Orin 优化模式启动
./run_slam.sh --orin --no-rviz
```

#### Orin NX 配置优化项

| 参数 | 默认值 | Orin值 | 说明 |
|------|--------|--------|------|
| point_filter_num | 1 | 2 | 每2点取1，减少输入 |
| map_voxel_size | 0.05 | 0.1 | 降低地图密度 |
| max_num_iteration | 10 | 6 | 减少ICP迭代 |
| ceres_iterations | 5 | 3 | 减少Ceres迭代 |
| ceres_threads | 3 | 4 | 利用8核CPU |
| max_num_residuals | 1200 | 800 | 减少残差数量 |
| max_distance | 80 | 60 | 更激进FOV清理 |
| resolutions | 0.2/0.5/1.2 | 0.3/0.8/1.5 | 降低分辨率 |

#### 预期性能（Orin NX 16GB）

| 指标 | 默认配置 | Orin优化配置 |
|------|---------|-------------|
| 帧处理时间 | 300-500ms | 150-250ms |
| 内存占用 | ~3GB | ~1.5-2GB |
| 实时性 | 不足 | 接近实时 |

#### 部署建议

1. **配置 swap**：至少 8GB swap
   ```bash
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

2. **测试时降低速率**：先用 `--rate 0.5` 测试稳定性

3. **关闭 RViz**：边缘设备上使用 `--no-rviz` 节省资源

4. **监控资源**：运行时观察 MONITOR 日志中的 RSS 和 frame_ms

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
  map_voxel_size: 0.05   # 降采样体素大小 (m)，越小点越密集
```

### 场景调参建议

| 场景 | 建议调整 |
|------|---------|
| **楼梯/退化场景** | `beta_orientation_consistency: 0.5`<br>`motion_compensation: CONTINUOUS` |
| **快速运动** | `max_dist_to_plane_icp: 0.2`<br>`max_num_iteration: 15` |
| **大范围建图** | `max_distance: 100`<br>`map_voxel_size: 0.1` |
| **边缘设备** | 使用 `--orin` 或 `mapping_orin_nx.yaml` |

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
# PCL Viewer
pcl_viewer map/global_map.pcd

# RViz 查看
ros2 run pcl_ros pcd_to_pointcloud map/global_map.pcd 0.1 --ros-args -r cloud_pcd:=/map
```

### 内存管理机制

系统采用分段保存机制，每移动 50 米或点数超过 200 万时自动保存：

```
map/
├── segment_0.pcd      # 第一段
├── segment_1.pcd      # 第二段
├── ...
└── global_map.pcd     # 最终合并地图
```

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

- **frame_ms 持续 >150ms**：考虑使用 Orin 优化配置
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

调小 `map_voxel_size`：
```yaml
common:
  map_voxel_size: 0.02
```

### Q: 长时间运行内存不足

1. 使用 `--orin` 优化模式
2. 增大 `map_voxel_size`
3. 减小 `max_distance`

### Q: MID-360 连接不上

1. 检查网络：电脑 IP 应为 `192.168.1.50`
2. 确认 MID360_config.json 中 IP 正确
3. 放行 56100-56500 端口

---

## 目录结构

```
slam-mid360-volita/
├── src/adaptive_lio/
│   ├── config/
│   │   ├── mapping_m.yaml        # 默认配置
│   │   ├── mapping_orin_nx.yaml  # Orin NX 优化配置
│   │   └── adaptive_lio.rviz     # RViz 配置
│   ├── bash/
│   │   └── run_slam.sh           # 启动脚本
│   ├── launch/
│   │   └── run.launch.py         # ROS2 Launch
│   ├── scripts/
│   │   └── gui_launcher.py       # GUI 控制面板
│   ├── src/
│   │   ├── apps/                 # ROS2 节点
│   │   ├── lio/                  # 核心算法
│   │   ├── algo/                 # ESKF 滤波器
│   │   └── common/               # 工具库
│   └── map/                      # 地图输出
└── README.md
```

---

## 技术架构

```
IMU数据 ──> ESKF状态预测 ──> 点云去畸变 ──> CT-ICP Ceres优化 ──> 多分辨率地图更新
              │                                  │                      │
          高频积分预测                    连续时间运动建模          自适应权重匹配
```

### 与 FAST-LIO2 对比

| 维度 | FAST-LIO2 | 本项目 |
|------|-----------|--------|
| 优化方式 | IESKF (线性化) | Ceres (真非线性) |
| 运动模型 | 离散帧间 | 连续时间 (CT) |
| 地图结构 | ikd-tree | 多分辨率 Voxel 哈希 |
| 约束数量 | 1-2 种 | 7 种 |
| 边缘设备 | 需修改 | 内置 Orin 配置 |

---

## 致谢

本项目基于以下开源项目：

- [Adaptive-LIO](https://github.com/chengwei0427/Adaptive-LIO) - 原始 ROS1 实现
- [CT-ICP](https://github.com/jedeschaud/ct_icp) - 连续时间 ICP 算法
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) - Livox 官方驱动

---

## License

BSD-3-Clause License
