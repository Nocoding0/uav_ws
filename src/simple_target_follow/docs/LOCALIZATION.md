# 无人机 LIO 定位验收路线

当前可复用基线是 FAST-LIVO2 的纯 LIO 模式，而不是相机参与的 LIVO：

```text
Mid-360 points /livox/lidar + built-in IMU /livox/imu
  -> FAST-LIVO2 (img_en=0)
  -> /fast_livo/unconverted_vision_pose
  -> guarded bridge
  -> /mavros/vision_pose/pose
  -> PX4 EKF2
  -> /mavros/local_position/pose
```

Xsens 和 D435i 不参与这条定位基线。雷达到内置 IMU 外参使用此前手持验证过的
单位阵；任何机械安装、固件、时间同步或传感器变化都要求重新验收。

## 0. 每次测试前

- 拆桨并固定机架；连接 Mid-360 网线和飞控 USB。
- 有线网卡应为 `192.168.1.5`，Mid-360 配置为 `192.168.1.145`。
- 确认 `/dev/serial/by-id/` 中真实飞控路径；不要让 QGroundControl 和 MAVROS
  同时占用同一串口。
- 首次上电后保持静止，等 IMU 初始化和地图建立完成再移动。
- 不要与原 `drone_control` 或旧 `sensor_bringup.launch` 同时启动雷达、
  FAST-LIVO2 或 MAVROS；ROS master 需要可正常访问。

## 1. 仅验证 LIO，不连接 PX4

```bash
source /opt/ros/noetic/setup.bash
source ~/drone_ws/devel/setup.bash
source ~/uav_ws/devel/setup.bash
roslaunch simple_target_follow lio_bringup.launch
```

检查：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /fast_livo/unconverted_vision_pose
rostopic echo -n 1 /fast_livo/unconverted_vision_pose
```

此阶段默认不启动桥接，`/target_follow/localization_bridge_status` 不存在是
正常的。先运行静止测试：

```bash
roslaunch simple_target_follow localization_validation.launch \
  require_local:=false vision_topic:=/fast_livo/unconverted_vision_pose duration:=60
```

再拆桨手持做 +X、+Y、+Z 各 0.5 m 和逆时针 +90 度测试。X/Y/Z 的正方向定义
为机体前/左/上，yaw 的正方向为俯视逆时针：

```bash
roslaunch simple_target_follow localization_validation.launch run_quality:=false \
  run_axis:=true axis:=x expected_distance:=0.5 \
  pose_topic:=/fast_livo/unconverted_vision_pose
```

将 `axis` 依次改为 `y`、`z`；yaw 使用：

```bash
roslaunch simple_target_follow localization_validation.launch run_quality:=false \
  run_axis:=true axis:=yaw expected_distance:=1.5707963268 \
  pose_topic:=/fast_livo/unconverted_vision_pose
```

## 2. 接入 MAVROS，但先不向 PX4 发布外部位姿

```bash
roslaunch simple_target_follow lio_bringup.launch start_mavros:=true
```

确认 MAVROS 连接、飞控型号/固件版本和本地位置来源。此时
`publish_to_mavros=false`，可安全核对 PX4 参数，不会从本节点输入外部位姿。
上面的命令是**替换第 1 步的 roslaunch**，先停止旧实例，不能同时启动两个
`lio_bringup`。只用 USB 供电的拆桨阶段健康监视器有可能报告缺少电池；这不
代表能跳过装桨前的电池检查。

## 3. 向 PX4 发送已门控的位姿并审计 EKF

只有第 1、2 步通过后才运行：

```bash
roslaunch simple_target_follow lio_bringup.launch \
  start_mavros:=true publish_to_mavros:=true
roslaunch simple_target_follow px4_vision_audit.launch
```

报告保存为 `/tmp/uav_px4_vision_audit.json`。审计脚本只读取参数；PX4 新版通常
使用 `EKF2_EV_CTRL`，旧版可能使用 `EKF2_AID_MASK`。必须先记录实际固件版本和
参数集合，再在 QGroundControl 中决定位置、速度、航向和高度融合项，不能同时
启用未经对齐的航向源，也不能照抄另一版本的数值。
这里也应先停止第 2 步的 `lio_bringup`，再用本阶段参数启动一个新实例。
`px4_vision_audit` 从**另一终端**启动，输出的位姿新鲜/本地位姿标志或 PX4
参数值都不能单独证明 EKF 已融合外部视觉；必须结合飞控 EKF 状态、日志中的
创新/拒绝标志及拆桨移动测试共同判断。`/target_follow/localization_bridge_status`
可用于观察桥接是否持续输出；上游节点重启或桥接报跳变后先停机排查。

接着分别运行 60 秒静止质量检查和手持融合对比（后一项需要手持缓慢移动，
不要在无人机静止时期待它证明 PX4 接收了外部位姿）：

```bash
roslaunch simple_target_follow localization_validation.launch duration:=60
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_fusion:=true duration:=30
```

## 4. 定点效果测试顺序

1. 拆桨完成以上全部测试，并保存 JSON、rosbag 和 PX4 参数备份。
2. 装桨前先验证遥控器接管、模式开关和失去外部定位的 PX4 failsafe。
3. 保护场地内人工起飞，低高度切换 POSCTL，先观察 PX4 基于融合位置的定点。
4. 通过后再使用独立的 OFFBOARD 单点悬停脚本；不要启动旧赛题任务入口。

旧 `takeoff_hover_land.py` 的 `allow_arming:=true` 会主动切 OFFBOARD、解锁，
它不属于本定位验收流程。POSCTL 定点前也须核对当时的 PX4 模式是否真的使用
外部位置，而非其他已存在的位置来源。

当前没有证据表明此飞控已完成外部视觉融合配置，因此不能从“话题有数据”直接
跳到装桨定点。定位优化应基于每轮 rosbag/PX4 ulg 的漂移、创新、振动、丢帧和
CPU 数据，而不是先改滤波参数。
