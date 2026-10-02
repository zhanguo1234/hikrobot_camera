# hikrobot_camera

基于 **HIKROBOT MVS SDK** 封装的 ROS 2 Humble 功能包，把海康工业相机接入 ROS 2：
自动发现并连接相机、稳定取流、把图像发布为标准 `sensor_msgs/msg/Image`，
支持通过 ROS 2 参数在**运行中**动态调整曝光、增益、帧率、像素格式，
并具备**断线自动重连 + 重连后恢复参数**的能力。

> 本包已在 **MV-CS016-10UC**（USB3 Vision，1440×1080 彩色）上完成实测：
> 连接、取流、像素格式转换、`/image_raw` 发布、运行期调参、拔插重连全部验证通过。

---

## 1. 环境与依赖

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 海康 MVS | **MVS 5.1.0 Build 20260909**（工业相机 SDK **4.8.2.2 Build 20260901**） |
| 测试相机 | MV-CS016-10UC（USB3 Vision，序列号 DB0178696） |

ROS 侧依赖（已写入 `package.xml`，可由 rosdep 解析）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

即 `rclcpp`、`sensor_msgs`。

---

## 2. 安装 MVS SDK（**不能**用 rosdep 自动安装）

海康 MVS SDK 是厂商私有库，**没有对应的 rosdep 规则**，必须手动安装：

1. 到海康机器人官网下载 Linux 版 MVS（本包实测版本 **5.1.0**）：
   <https://www.hikrobotics.com/cn/machinevision/service/download>
2. 安装：

   ```bash
   tar -xzf MVS-x.x.x_x86_64_xxxxxx.tar.gz
   cd MVS-x.x.x_x86_64_xxxxxx
   sudo ./setup.sh
   ```

3. 默认安装到 `/opt/MVS`。确认关键文件存在：

   ```bash
   ls /opt/MVS/include/MvCameraControl.h
   ls /opt/MVS/lib/64/libMvCameraControl.so
   ```

> **若你的安装路径不是 `/opt/MVS`**，需要同步修改两处：
> `CMakeLists.txt` 里的 `/opt/MVS/include`、`/opt/MVS/lib/64`，
> 以及 `launch/hik_camera.launch.py` 里的 `MVS_LIB_DIR`。

---

## 3. 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hikrobot_camera
source install/setup.bash
```

---

## 4. 运行

### 4.1 用 launch 文件（推荐）

```bash
ros2 launch hikrobot_camera hik_camera.launch.py
```

带参数启动：

```bash
ros2 launch hikrobot_camera hik_camera.launch.py \
    camera_serial:=DB0178696 \
    exposure_time:=5000.0 \
    gain:=5.0
```

> `pixel_format` 默认留空（沿用相机自身的格式），一般不用指定。
> 需要时再加 `pixel_format:=Mono8` 等。

> launch 文件里已通过 `AppendEnvironmentVariable` 自动设置
> `LD_LIBRARY_PATH=/opt/MVS/lib/64`，**不需要你手动 export**。
> 原因见第 8.1 节。

查看全部可配置项：

```bash
ros2 launch hikrobot_camera hik_camera.launch.py --show-args
```

### 4.2 直接运行节点

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH   # 见 8.1
ros2 run hikrobot_camera hik_camera_node --ros-args \
    -p camera_serial:=DB0178696 -p exposure_time:=5000.0
```

### 4.3 查看图像

```bash
ros2 run rqt_image_view rqt_image_view      # 下拉框选 /image_raw
# 或
rviz2                                       # Add -> Image -> Topic: /image_raw

ros2 topic hz /image_raw                    # 看实际帧率
```

### 4.4 无相机时的冒烟测试

包内附带一个假图节点，用于在没有相机时验证 ROS 侧链路是否正常：

```bash
ros2 run hikrobot_camera fake_camera_publisher
```

它向 `/image_raw` 发布 640×480 的 `mono8` 灰色图（10 Hz）。
**排查思路**：先用它确认话题链路通畅；若它也正常而真节点异常，问题必在 SDK 侧。

---

## 5. 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `<topic_name>`（默认 `/image_raw`） | `sensor_msgs/msg/Image` | 相机图像 |
| `~/measured_frame_rate` | `std_msgs/msg/Float32` | **实际采集帧率**（1 Hz 刷新） |

> **设定帧率 vs 实际帧率**：参数 `frame_rate` 是**你想要**的帧率（写入相机的
> `AcquisitionFrameRate`）；而实际能跑多少还受**曝光时间、USB 带宽、主机处理速度**
> 限制，通常小于等于设定值。节点每秒统计一次真实帧率，同时：
>
> - 发布到 `~/measured_frame_rate`（即 `/hik_camera_node/measured_frame_rate`）
> - 在终端日志打印 `实际采集帧率 xx.xx fps（设定值 yy.yy fps）`
>
> 例如曝光设成 200000 µs（0.2 秒）时，设定 165 fps，实际只能跑到约 5 fps。

图像 `encoding` 取决于相机当前的像素格式：

| 相机像素格式 | 发布的 encoding | step |
|---|---|---|
| `Mono8` | `mono8` | `width` |
| `BGR8Packed` | `bgr8` | `width × 3` |
| `RGB8Packed` | `rgb8` | `width × 3` |
| `BayerRG8` / `BayerRG10` / `BayerRG12` / `YUV422*` | `bgr8`（由 SDK 转换） | `width × 3` |

---

## 6. 参数

所有参数均可在**运行中**通过 `ros2 param set` 修改（`topic_name` / `camera_serial` / `camera_ip` 除外，它们影响资源创建，需重启节点）。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `camera_serial` | string | `""` | 按序列号选择相机 |
| `camera_ip` | string | `""` | 按 IP 选择相机（GigE） |
| `topic_name` | string | `image_raw` | 图像话题名（**改动需重启**） |
| `frame_id` | string | `camera` | 图像消息的 `frame_id` |
| `pixel_format` | string | `""`（留空） | 留空 = **沿用相机自身的像素格式**；只有显式指定时才会去改相机 |
| `exposure_time` | double | `-1.0` | 曝光时间 µs，**负值表示不修改相机** |
| `gain` | double | `-1.0` | 增益 dB，**负值表示不修改相机** |
| `frame_rate` | double | `-1.0` | 采集帧率 fps，**负值表示不修改相机** |

`camera_serial` 与 `camera_ip` 都留空时，连接枚举到的第一台相机。

`pixel_format` 可填的值（**留空是最省事、兼容性最好的选择**）：
`Mono8`、`BayerRG8`、`BayerRG10`、`BayerRG12`、`RGB8`、`BGR8`、
`YUV422_YUYV`、`YUV422`

> **为什么默认留空**：不同型号支持的像素格式差别很大——黑白相机只有
> Mono 系列，没有 Bayer 系列。如果给一个 `BayerRG8` 之类的默认值，
> 换一台黑白相机就会在启动时报错。留空表示"相机原来是什么格式就用什么格式"，
> 兼容性最好。需要指定时再显式填。

### 6.1 运行期调参示例

```bash
# 读
ros2 param get /hik_camera_node exposure_time

# 写（成功）
ros2 param set /hik_camera_node exposure_time 80000.0
# -> Set parameter successful

# 写（超范围：会被拦下并给出明确原因）
ros2 param set /hik_camera_node exposure_time 99999999.0
# -> Setting parameter failed: exposure_time: 曝光 99999999.0 超出范围 [15.0, 9996427.0] us

# 切换像素格式
ros2 param set /hik_camera_node pixel_format Mono8
# -> Set parameter successful    （画面转为灰度，encoding 变为 mono8）

# 写错格式
ros2 param set /hik_camera_node pixel_format xyz
# -> Setting parameter failed: pixel_format: 不支持的像素格式: xyz
#    （可选 Mono8 / BayerRG8 / BayerRG10 / BayerRG12 / RGB8 / BGR8 / YUV422_YUYV / YUV422）
```

参数校验的两个要点：

1. **范围不是写死的**，每次设置前用 `MV_CC_GetFloatValue` 从相机读回
   `fMin` / `fMax` 再比对，换相机也不会失效。
2. **手动设置曝光/增益前会先关闭对应的自动模式**（`ExposureAuto=0` / `GainAuto=0`），
   否则相机在自动模式下会忽略写入。

### 6.2 对比设定帧率与实际帧率

```bash
# 设定值
ros2 param get /hik_camera_node frame_rate

# 实际值（节点每秒统计）
ros2 topic echo /hik_camera_node/measured_frame_rate

# 也可以直接看图像话题的实际频率
ros2 topic hz /image_raw
```

节点日志里每秒也会打印一行，两者直接对照：

```
[INFO] [...] 实际采集帧率 69.86 fps（设定值 165.00 fps）
```

---

## 7. 实测数据（MV-CS016-10UC）

| 项目 | 实测值 |
|---|---|
| 分辨率 | 1440 × 1080（Width 范围 32~1440，Height 范围 8~1080） |
| 曝光时间 | 15.00 ~ 9996427.00 µs（出厂默认 5000） |
| 增益 | 0.00 ~ 16.98 dB（出厂默认 0） |
| 采集帧率 | 0.10 ~ 100000.00 fps（出厂默认 165） |
| 支持的像素格式 | 10 种：Mono8、RGB8Packed、BGR8Packed、YUV422_YUYV_Packed、YUV422Packed、BayerRG8、BayerRG10、BayerRG10Packed、BayerRG12、BayerRG12Packed |

**帧率与 USB 带宽强相关**（同一台相机、同一分辨率，仅换插口）：

| 像素格式 | 数据量/帧 | USB 2.0 口 | USB 3.0 口 |
|---|---|---|---|
| `Mono8` | 1.55 MB | ~20 fps | **~85 fps** |
| `BGR8Packed` | 4.66 MB | 带不动 | ~70 fps |
| `BayerRG8`（转 BGR8 后发布） | 1.55 MB 采集 | 带不动 | ~70 fps |

> 结论：**务必插在 USB 3.0 口**（通常为蓝色口）。
> 用 `lsusb | grep 2bdf` 可以确认相机挂在哪个总线上：`Bus 002` 是 USB 3.0，`Bus 001` 是 USB 2.0。

---

## 7.1 适用型号与已知限制

**本包只在 `MV-CS016-10UC`（USB3、彩色、1440×1080）上做过完整实测**，
其余型号的情况是按海康 SDK 的通用接口设计的，属于推断而非实测结论。

### 与型号无关的通用机制

- 设备枚举、按序列号 / IP 匹配连接
- 取流（`MV_CC_StartGrabbing` / `MV_CC_GetImageBuffer` / `MV_CC_FreeImageBuffer`）
- 分辨率：从帧信息读取，不硬编码
- 参数量程：每次从相机 `MV_CC_GetFloatValue` 读回 `fMin`/`fMax`，不写死
- 像素格式转换：`MV_CC_ConvertPixelTypeEx`，任意源格式 → BGR8
- 断线重连与参数恢复

### 针对型号差异做的兼容处理

| 处理 | 解决什么问题 |
|---|---|
| 启动时下发 `TriggerMode = 0` | 相机若处于**触发模式**（软/硬触发），不关掉就一帧都收不到，且不报错。现在会自动改为连续采集 |
| `pixel_format` 默认**留空** | 黑白相机没有 Bayer 系列，若给默认值会启动报错。留空则沿用相机自身格式 |
| 启动时打印相机实际信息 | 日志会输出**当前像素格式、分辨率、曝光/增益/帧率量程**，换型号时一眼看清 |
| 参数设置前读回量程 | 不同型号的量程差异极大，不写死 |
| `AcquisitionFrameRateEnable` 失败只警告 | 部分型号没有这个节点，此时仍继续尝试写入帧率 |

### 尚未验证 / 未实现的部分

| 项目 | 说明 |
|---|---|
| **网口（GigE）相机** | 代码支持按 IP 匹配连接，但**未做 GigE 专属优化**（`GevSCPSPacketSize` 最优包大小、心跳超时、丢包重传）。接网口相机时可能丢包或帧率偏低，**未实测** |
| **黑白相机** | 设计上兼容（`Mono8` 直发、`pixel_format` 留空），但**未实测** |
| 其他像素格式 | 只内置了 8 种常见格式的名字映射；若相机使用 `Mono10`、`BayerGB8` 等，`pixel_format` 参数填不进去——但**留空即可正常出图**（此时若格式非 BGR8/Mono8/RGB8，会经 SDK 转换） |
| 多相机同时接入 | 单节点只连一台；多相机需启动多个节点，分别指定 `camera_serial` 与 `topic_name` |
| `camera_info` 标定 | 未实现 |

> **结论**：换用**连续采集模式下的海康标准 USB3 相机**（彩色或黑白）预期可直接使用；
> 换用**网口相机**需要额外验证。

---

## 8. 常见问题

### 8.1 `像素格式转换失败 0x8000000C`（`MV_E_LOAD_LIBRARY`）—— **最容易踩的坑**

MVS SDK 在做像素格式转换（如 Bayer → BGR）时，会在**运行时 `dlopen`**
`/opt/MVS/lib/64/libFormatConversion.so`。而 `dlopen` 只认**调用者自己**的
`RUNPATH`，**不认**可执行文件上设置的 `RUNPATH`；偏偏 MVS 又**没有**把自己
注册进 `/etc/ld.so.conf.d/`（`ldconfig -p | grep MvCamera` 为空）。因此必须显式给出库路径：

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
```

本包已针对这个问题做了两层处理：

- `CMakeLists.txt` 里用 `INSTALL_RPATH` 解决可执行文件找不到
  `libMvCameraControl.so` 的问题（这一层 `RUNPATH` 是有效的）；
- `launch/hik_camera.launch.py` 里用 `AppendEnvironmentVariable` 设置
  `LD_LIBRARY_PATH`，解决 `dlopen` 那一层（**这一层 `RUNPATH` 无效**）。

**只做第一层是不够的** —— 这正是本包同时做两层的原因。

### 8.2 帧率上不去（只有 ~20 fps）

```bash
lsusb | grep 2bdf
```

- 出现在 `Bus 001` → 插在 USB 2.0 口上，换到 **USB 3.0 口**（蓝色，应显示 `Bus 002`）；
- 也可能是曝光时间过长：曝光 5000 µs 时帧率上限约 200 fps，再高就得降曝光。

### 8.3 `未发现任何相机设备`

```bash
lsusb | grep 2bdf                       # 内核是否认到
# 权限：MVS 会安装 /etc/udev/rules.d/80-drivers-SDK-2bdf.rules (MODE=0666)
# 换 USB 3.0 口、换线，避免经过 USB Hub
sudo /opt/MVS/bin/set_usbfs_memory_size.sh   # 带宽不足时提高 usbfs 内存上限
```

### 8.4 `MV_CC_OpenDevice 失败 0x80000006 / 0x80000028`

相机是**独占**打开的（`MV_ACCESS_Exclusive`），同一时刻只能有一个程序使用。
先确认没有别的程序占用：

```bash
ros2 node list
ps -ef | grep -i hik | grep -v grep
```

关掉 MVS 客户端、或上一个没退干净的节点即可。

### 8.5 `MV_CC_OpenDevice 失败 0x80000301`（`MV_E_USB_WRITE`）

USB 通信异常。**先拔掉相机重新插一次**，一般即可恢复，不必改代码。

### 8.6 `error while loading shared libraries: libMvCameraControl.so`

可执行文件的 `INSTALL_RPATH` 没生效（例如手工把可执行文件拷到别处）。
临时解法同 8.1。

### 8.7 `ros2 param set` 长时间没有反应

说明节点进程已经不健康。本包已在取流线程与参数回调之间做了让步机制
（详见第 9 节），正常情况下不会出现。若出现，检查节点终端是否有报错。

---

## 9. 实现要点

- **连接**：`MV_CC_EnumDevices` 枚举 → 按序列号/IP 匹配 → `MV_CC_CreateHandle`
  → `MV_CC_OpenDevice(MV_ACCESS_Exclusive)` → 下发参数 → `MV_CC_StartGrabbing`。
- **取流**：独立线程循环 `MV_CC_GetImageBuffer`（200 ms 超时）/ `MV_CC_FreeImageBuffer`，
  不阻塞 ROS 执行器。`MV_E_NODATA`（超时无数据）视为正常。
- **像素格式**：`Mono8`/`BGR8`/`RGB8` 直接发布；其余格式用
  `MV_CC_ConvertPixelTypeEx` 转成 `BGR8` 后发布，转换缓冲区跨帧复用避免反复分配。
  **切换像素格式时必须先 `MV_CC_StopGrabbing`，设置完再 `MV_CC_StartGrabbing`。**
- **参数校验**：设置前从相机读回量程比对；手动曝光/增益前先关闭对应自动模式；
  每次 SDK 调用都检查返回值，失败时通过参数服务的 `reason` 返回明确原因。
- **参数声明顺序**：**先 `declare_parameter`，再 `add_on_set_parameters_callback`**。
  因为 `declare_parameter` 本身也会触发参数回调，若顺序反了，回调会在相机
  尚未打开时被调用、返回失败，导致 `declare_parameter` 抛异常、节点直接崩溃。
- **线程安全**：用一个 `std::mutex` 保护所有对相机句柄的访问（取流线程与参数
  回调都会用到）。同时用一个 `param_pending_` 让步标志避免**锁饥饿**——
  取流线程在 70 fps 紧循环里会不断重新抢锁，而 `std::mutex` 不保证公平，
  没有让步机制时参数回调会被无限期饿死（表现为 `ros2 param set` 永远不返回）。
- **断线重连**：取流返回非超时错误 → 关闭句柄并进入重连；
  另外**连续 5 秒收不到图**也会判定掉线（USB 被拔时 SDK 常常只报超时而
  不报错）。掉线后每 2 秒重新枚举并连接，连接成功后自动重新下发全部参数。
- **型号兼容性**：启动时主动下发 `TriggerMode = 0` 关闭触发（否则相机若处于
  触发模式会一帧都收不到且不报错）；`pixel_format` 默认留空、沿用相机自身设置，
  避免把黑白相机不支持的 Bayer 格式强加给它；连接成功后打印相机的
  当前像素格式、分辨率和各参数量程。

---

## 10. 验收自测步骤

```bash
# 1) 编译：零错误、零警告
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hikrobot_camera

# 2) 启动（无需手动 export LD_LIBRARY_PATH）
source install/setup.bash
ros2 launch hikrobot_camera hik_camera.launch.py

# 3) 话题与数据
ros2 topic list | grep image_raw
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once --field encoding      # bgr8
ros2 topic echo /image_raw --once --field width         # 1440

# 3b) 实际采集帧率（与设定值区分）
ros2 topic echo /hik_camera_node/measured_frame_rate

# 4) rqt_image_view / rviz2 中图像显示正常

# 5) 运行期调参生效
ros2 param set /hik_camera_node exposure_time 200000.0  # 画面明显变亮
ros2 param set /hik_camera_node gain 10.0               # 更亮并带轻微噪声
ros2 param set /hik_camera_node pixel_format Mono8      # 画面变灰，encoding -> mono8
ros2 param set /hik_camera_node pixel_format BGR8       # 恢复彩色

# 6) 参数校验
ros2 param set /hik_camera_node exposure_time 99999999.0
# -> Setting parameter failed: exposure_time: 曝光 ... 超出范围 [15.0, 9996427.0] us

# 7) 断线重连
ros2 param set /hik_camera_node exposure_time 150000.0  # 先设一个好认的值
#    拔掉相机 USB，约 5 秒后节点日志出现"判定为掉线，开始重连"
#    插回相机，日志出现"曝光已设为 150000.0 us""相机已重新连接，参数已恢复"
```

---

## 11. 已知限制

- **只在 `MV-CS016-10UC`（USB3、彩色）上做过完整实测**；网口相机与黑白相机
  属于设计上兼容但未实测，详见第 7.1 节。
- 断线重连采用「错误码 + 5 秒无图」双重判定；若把 `frame_rate` 设到
  低于 0.2 fps，5 秒无图会误判为掉线。正常使用（≥1 fps）不受影响。
- 不支持多相机同时接入同一节点；多相机请为每个节点指定不同的
  `camera_serial` 与 `topic_name`。
- 未使用 `image_transport` 压缩传输，发布的是原始 `Image`。
- 未实现相机标定（`camera_info`）发布。
- 网口相机缺少 GigE 专属优化（最优包大小、心跳超时），可能丢包或帧率偏低。

---

## 12. 目录结构

```
hikrobot_camera/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── hik_camera.launch.py      # 启动文件（含 LD_LIBRARY_PATH 处理）
└── src/
    ├── hik_camera_node.cpp       # 真实相机节点
    └── fake_camera_publisher.cpp # 无相机时的冒烟测试节点
```
