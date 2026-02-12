# SLAM-MID360-Volita

基于 CT-ICP + Ceres 多约束优化的激光惯性里程计系统，专为 Livox MID-360 优化，支持 ROS2 Humble。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-green)](LICENSE)

---

## 功能特点

- **CT-ICP 连续时间运动补偿**：扫描内每个点独立去畸变，高动态场景下精度更高
- **Ceres 多约束优化**：7 种约束因子联合优化，天然抗退化
- **多分辨率体素地图**：3 层分辨率 (0.2m/0.5m/1.2m)，平衡精度与效率
- **内存管理**：分段保存机制，支持长时间大范围建图
- **GUI 控制面板**：可视化参数调节、一键启停、实时状态监控

---

## 目录

- [环境要求](#环境要求)
- [依赖安装](#依赖安装)
  - [1. ROS2 Humble](#1-ros2-humble)
  - [2. Livox SDK2 与驱动](#2-livox-sdk2-与驱动)
  - [3. 系统依赖库](#3-系统依赖库)
- [编译安装](#编译安装)
- [快速开始](#快速开始)
- [传感器配置](#传感器配置)
- [参数调优](#参数调优)
- [地图管理](#地图管理)
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

---

## 依赖安装

### 1. ROS2 Humble

如果尚未安装 ROS2 Humble：

```bash
# 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-humble-pcl-ros ros-dev-tools -y

# 配置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Livox SDK2 与驱动

#### 2.1 安装 Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

#### 2.2 安装 livox_ros_driver2

```bash
# 创建 Livox 工作空间
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 编译
cd ~/livox_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 配置环境 (添加到 ~/.bashrc)
echo "source ~/livox_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3 配置 MID-360

编辑 `~/livox_ws/src/livox_ros_driver2/config/MID360_config.json`：

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip": "192.168.1.50",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0.0, "y": 0.0, "z": 0.0
      }
    }
  ]
}
```

> **注意**：将 `192.168.1.1xx` 替换为你的 MID-360 实际 IP 地址（通常为 `192.168.1.1xx`，xx 为设备序列号后两位）。

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
git clone https://github.com/YOUR_USERNAME/slam-mid360-volita.git
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

### 方式一：实时传感器

```bash
# 终端 1: 启动 Livox 驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2: 启动 SLAM
ros2 launch adaptive_lio run.launch.py
```

### 方式二：回放数据包

```bash
# 终端 1: 启动 SLAM + RViz
ros2 launch adaptive_lio run.launch.py

# 终端 2: 播放数据包
ros2 bag play /path/to/your/rosbag --rate 1.0
```

### 方式三：GUI 控制面板

```bash
cd slam-mid360-volita
python3 src/adaptive_lio/scripts/gui_launcher.py
```

### 方式四：命令行脚本

```bash
# 基本启动
./src/adaptive_lio/scripts/run_adaptive_lio.sh

# 指定数据包和保存路径
./src/adaptive_lio/scripts/run_adaptive_lio.sh --bag /path/to/bag --save /path/to/output
```

---

## 传感器配置

### 支持的传感器

| 传感器类型 | lidar_type 值 | 话题格式 |
|-----------|---------------|---------|
| Livox AVIA | 1 | livox_ros_driver2/CustomMsg |
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
  point_filter_num: 1    # 点云抽稀 (1=不抽稀)

common:
  imu_topic: /livox/imu
  lid_topic: /livox/lidar
  gnorm: 1               # 加速度缩放因子
```

### 外参标定

```yaml
mapping:
  extrinsic_est_en: true   # 在线外参估计
  extrinsic_T: [0, 0, 0]   # 初始平移 [x, y, z]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 初始旋转 (行主序)
```

---

## 参数调优

### 核心参数 (config/mapping_m.yaml)

#### CT-ICP 配置

```yaml
odometry:
  icpmodel: CT_POINT_TO_PLANE    # CT_POINT_TO_PLANE(精确) / POINT_TO_PLANE(快速)
  max_num_iteration: 10           # Ceres 最大迭代次数
  max_dist_to_plane_icp: 0.3     # 点到平面距离阈值
  motion_compensation: CONSTANT_VELOCITY  # 运动补偿模式
```

#### 约束权重

```yaml
  beta_location_consistency: 0.001    # 位置连续约束
  beta_orientation_consistency: 0.1   # 旋转连续约束
  beta_constant_velocity: 0.001       # 恒速约束
  beta_small_velocity: 0.01           # 小速度惩罚
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
| **楼梯/退化场景** | `beta_orientation_consistency: 0.5`<br>`beta_location_consistency: 0.01`<br>`motion_compensation: CONTINUOUS` |
| **快速运动** | `max_dist_to_plane_icp: 0.2`<br>`max_num_iteration: 15` |
| **大范围建图** | `max_distance: 500`<br>`map_voxel_size: 0.1` |
| **高精度建图** | `map_voxel_size: 0.02`<br>`point_filter_num: 1` |

---

## 地图管理

### 保存地图

```bash
# 运行中手动保存
ros2 service call /save_map std_srvs/srv/Trigger

# 检查保存状态
ros2 service call /save_map_status std_srvs/srv/Trigger
```

> Ctrl+C 停止节点时会自动保存地图。

### 查看地图

```bash
# PCL Viewer
pcl_viewer map/global_map.pcd

# 多文件查看
pcl_viewer map/segment_*.pcd

# RViz 查看
ros2 run pcl_ros pcd_to_pointcloud map/global_map.pcd 0.1 --ros-args -r cloud_pcd:=/map
# 然后在 RViz 中订阅 /map 话题
```

### 内存管理

系统采用分段保存机制，每移动 50 米或点数超过 200 万时自动保存并清理内存：

```
map/
├── segment_0.pcd      # 第一段
├── segment_1.pcd      # 第二段
├── ...
├── segment_N.pcd      # 最后一段
└── global_map.pcd     # 最终合并地图
```

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
| `/tf` | TF2 | map→base_link |

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | CustomMsg | Livox 点云 |
| `/livox/imu` | Imu | IMU 数据 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/save_map` | Trigger | 保存地图 |
| `/save_map_status` | Trigger | 查询保存状态 |

---

## 常见问题

### Q: 编译时找不到 livox_ros_driver2

确保已安装 livox_ros_driver2 并 source 其 setup.bash：

```bash
source ~/livox_ws/install/setup.bash
colcon build
```

### Q: 点云显示但位姿不更新

1. 检查 IMU 数据是否正常：`ros2 topic echo /livox/imu`
2. 确认 `gnorm` 参数设置正确（加速度模长为 1 时填 1，为 9.8 时填 9.805）
3. 保持静止 2-3 秒等待 IMU 初始化

### Q: 地图保存后点很稀疏

调小 `map_voxel_size` 参数：

```yaml
common:
  map_voxel_size: 0.02  # 更密集的点云
```

### Q: 长时间运行内存不足

系统已内置分段保存机制，每 50 米或 200 万点自动保存并清理。如仍有问题：

1. 增大 `map_voxel_size` 减少点数
2. 减小 `segment_distance_threshold`（代码中修改）

### Q: VoxelGrid 溢出警告

大范围地图可能触发此警告，系统已内置 `safeVoxelDownsample` 自动处理，不影响结果。

### Q: MID-360 连接不上

1. 检查网络配置：电脑 IP 应为 `192.168.1.50`
2. 确认 MID360_config.json 中 IP 地址正确
3. 防火墙放行 56100-56500 端口

---

## 目录结构

```
slam-mid360-volita/
├── src/adaptive_lio/
│   ├── config/
│   │   ├── mapping_m.yaml      # 主配置文件
│   │   └── adaptive_lio.rviz   # RViz 配置
│   ├── launch/
│   │   └── run.launch.py       # 启动文件
│   ├── scripts/
│   │   ├── gui_launcher.py     # GUI 控制面板
│   │   └── run_adaptive_lio.sh # 命令行脚本
│   ├── src/
│   │   ├── apps/               # ROS2 节点
│   │   ├── lio/                # 核心算法
│   │   ├── algo/               # ESKF 滤波器
│   │   ├── preprocess/         # 点云预处理
│   │   └── common/             # 工具库
│   ├── thirdparty/             # 第三方库
│   ├── map/                    # 地图输出
│   └── log/                    # 日志输出
├── install/                    # 编译输出
├── build/                      # 构建缓存
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
| 退化处理 | 简单 | 多约束天然抗退化 |

---

## 致谢

本项目基于以下开源项目：

- [Adaptive-LIO](https://github.com/chengwei0427/Adaptive-LIO) - 原始 ROS1 实现
- [CT-ICP](https://github.com/jedeschaud/ct_icp) - 连续时间 ICP 算法
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) - Livox 官方驱动

---

## License

BSD-3-Clause License
# slam-mid360-volita
# slam-mid360-volita
# slam-mid360-volita
# slam-mid360-volita
