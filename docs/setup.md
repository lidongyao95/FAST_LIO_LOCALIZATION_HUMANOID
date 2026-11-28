# 环境配置

分两部分：仿真电脑和真机。只做仿真的话，看第一部分就够了。

## 一、仿真电脑

需要一台 Ubuntu 22.04 的电脑（x86_64）。有没有显卡、有没有显示器都可以。

### 1. 安装 ROS 2 Humble 和系统依赖

把下面整段复制到终端运行：

```bash
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-rmw-fastrtps-cpp \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  python3-colcon-common-extensions python3-venv python3-pip python3-pytest \
  python3-numpy python3-opencv \
  build-essential cmake git libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev \
  libfmt-dev libglfw3-dev libglew-dev libegl1 \
  libspirv-cross-c-shared0 libmeshoptimizer1d libtinyexr1d libastcenc1d
```

这里用的是清华镜像。在国外可以把地址换成 `http://packages.ros.org/ros2/ubuntu`。

### 2. 下载代码并编译

```bash
git clone https://github.com/lidongyao95/FAST_LIO_LOCALIZATION_HUMANOID.git
cd FAST_LIO_LOCALIZATION_HUMANOID
./g1_slam setup
```

`setup` 会下载宇树的仿真模型和行走策略、编译控制器和 ROS 包，大约 10 分钟。最后一行显示“仿真环境检查通过”就装好了。

以后更新了代码，再运行一次 `./g1_slam setup` 即可。

### 3. 验证

```bash
./g1_slam test
```

几秒钟后显示 `passed`。接着去看[纯仿真](simulation.md)。

## 二、真机

G1 的导航程序运行在机器人背后的 PC2 上（ARM64，Ubuntu 20.04，ROS 2 Foxy）。你在自己的电脑上操作，通过网线把代码传到 PC2 并在上面编译。

### 1. 用网线连接 PC2

1. 用网线把你的电脑接到机器人的网口。
2. 把电脑有线网卡的地址设成 `192.168.123.100`，子网掩码 `255.255.255.0`，网关和 DNS 留空。
3. 测试连接：

```bash
ping -c 2 192.168.123.164
ssh unitree@192.168.123.164
```

`192.168.123.164` 是 PC2 的固定地址。

### 2. 传代码并编译

在你的电脑上，仓库根目录里依次运行：

```bash
./g1_slam robot deploy check    # 检查 PC2 的系统、依赖和正在运行的程序
./g1_slam robot deploy sync     # 把代码传到 PC2
./g1_slam robot deploy build    # 在 PC2 上编译，约 15 分钟
```

`sync` 以你电脑上的代码为准，PC2 上的地图和录制数据不受影响。`build` 结束时会自动检查环境，显示“PC2 环境检查通过”就完成了。

代码在 PC2 的 `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID`，地图和日志在 `~/ldy/g1_real_data`。

### 3. 切到无线（可选）

建图时机器人要走动，拖着网线不方便。PC2 连上 Wi-Fi 后，用无线地址登录：

```bash
# 在 PC2 上查看无线地址
ip -4 addr show wlan0
# 在你的电脑上
ssh unitree@<无线地址>
```

无线地址每次可能不同。确认无线能登录后再拔掉网线。

接着去看[真机部署](robot.md)。

一台全新的 PC2 还需要先装 Foxy、Nav2 和 ARM 版 Open3D，步骤见[PC2 部署记录](notes/g1_real_robot_slam_pc2_b300_guide.md)。
