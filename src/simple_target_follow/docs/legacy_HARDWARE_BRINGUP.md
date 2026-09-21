# 2025 电赛 H 题真机复现与自主飞行清单

## 当前进度（2026-07-23）

机载电脑已经与 Xsens MTi-680、Livox Mid-360 和 Intel D435i 完成物理连接。
这表示项目已经进入“传感器与定位联调”阶段，但不能仅凭设备在线就装桨执行
自主任务。允许自主飞行前，必须依次证明：

1. 相机、IMU、雷达数据持续、时间戳正常且无 NaN；
2. 定位算法持续输出机体位姿；
3. FAST-LIVO2 发布的位姿经过检查后送入 `/mavros/vision_pose/pose`；
4. PX4 EKF 接受外部视觉后，`/mavros/local_position/pose` 稳定且方向正确；
5. 遥控器模式开关、人工接管和急停已经实测；
6. 无桨 OFFBOARD 流程和所有软件安全门均通过。

本项目已经具备 9×7 方格航线、三个连续禁飞格绕行、1.2 m 定高、300 s
超时、相对起飞点围栏、异常降落和 AUTO.LAND。尚未完成上述定位闭环前，
`allow_arming` 必须保持为 `false`。

## 1. 复现目标

场地为 9×7 个 0.5 m 方格，巡查区域为 4.5 m × 3.5 m。任务程序根据现场
输入的三个连续禁飞格生成安全路线，覆盖其余 60 格，在 120±10 cm 高度
巡查，最后以约 45°轨迹返回红色起飞区并执行 PX4 `AUTO.LAND`。

推荐的真机数据链如下：

```text
D435i + MTi-680 + Mid-360
              ↓
        FAST-LIVO / VIO
              ↓  位姿、时间戳、外参和坐标系检查
  /mavros/vision_pose/pose
              ↓
            PX4 EKF
              ↓
 /mavros/local_position/pose
              ↓
 target_follow_mission.py → OFFBOARD 位置设定值
```

任务控制器只消费 PX4 融合后的局部位置，不直接用原始点云控制飞行。雷达的
作用是定位，不是避障；当前航线避让的是赛题指定禁飞方格。

## 2. 启动传感器并验收数据

先加载传感器驱动工作空间和本项目安装空间：

```bash
source /opt/ros/noetic/setup.bash
source ~/drone_ws/devel/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch simple_target_follow sensor_bringup.launch \
  use_livox:=true use_fast_livo:=false
```

第一轮只检查驱动，不启动 FAST-LIVO。另开终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/drone_ws/devel/setup.bash
source ~/catkin_ws/install/setup.bash
rostopic echo -n 1 /target_follow/sensor_status
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic hz /imu/data
rostopic hz /livox/lidar
```

`/target_follow/sensor_status` 的 `ready` 必须为 `true`，并且各数据流持续发布。
如果实际话题名不同，应修改 `sensor_bringup.launch` 的监控参数或对应驱动
配置，不能为了得到 `ready=true` 而关闭必需传感器。

确认 Mid-360 网络地址、供电以及相机、IMU、雷达到机体系的外参后，才启动：

```bash
roslaunch simple_target_follow sensor_bringup.launch \
  use_livox:=true use_fast_livo:=true
```

旧外参只能作为初始参考。传感器安装位置或朝向发生变化后必须重新标定。

## 3. 打通定位到 PX4 的闭环

本机 `/home/lfy/drone_ws/src/FAST-LIVO2-SE-CN` 在
`LIVMapper.cpp` 中产生：

- `/aft_mapped_to_init`：`nav_msgs/Odometry`；
- 隔离后的 `/fast_livo/unconverted_vision_pose`：
  `geometry_msgs/PoseStamped`；
- 坐标系名称为 `camera_init`。

2026-07-31 已验证该安装在单位原生 IMU 外参下的 X 前、Y 左、Z 上和正偏航
逆时针方向。项目内 `fast_livo_pose_bridge.py` 数值直通该 ENU 位姿到
`/mavros/vision_pose/pose`，同时拒绝陈旧时间戳、非法四元数和定位跳变。
仍须在真实飞控上确认 PX4 EKF 正确接受。启动后检查：

```bash
rostopic list | sort
rostopic list | grep -E 'odom|pose|livo|vision'
rostopic info /mavros/vision_pose/pose
rostopic info /mavros/local_position/pose
```

发布到 `/mavros/vision_pose/pose` 前后逐项确认：

- ROS 侧采用 ENU：X 向场地前方、Y 向左、Z 向上；
- 位姿表示机体在固定世界坐标系中的位置，而不是传感器自身坐标；
- `header.stamp` 使用采样时间，不使用零时间或明显滞后的时间；
- 四元数已归一化，静止时位置和姿态不会跳变；
- 相机、IMU、雷达到机体中心的外参方向正确；
- 定位重启、丢帧或重定位时不会突然改变原点。

特别注意：虽然配置文件名是 `mid360_mti680.yaml`，当前文件中的
`common/imu_topic` 实际为 `/livox/imu`。这表示 FAST-LIVO2 当前使用
Mid-360 内置 IMU，Xsens `/imu/data` 仅由传感器监控检查，并未参与该定位
算法。2026-07-31 的本机静止与约 90° 偏航测试确认：该原生 IMU 应使用
单位雷达到 IMU 外参。`sensor_bringup.launch` 已默认覆盖旧配置中的非单位
外参；单位外参下静止漂移约 2.1 mm、偏航约 89.5°、旋转假位移约 1.0 cm，
而旧外参产生约 1.45 m 假位移。不得在未完成时间同步、外参和噪声参数标定
的情况下直接把话题改成 `/imu/data`。如果最终决定使用 MTi-680 参与定位，
应另建经实测验证的配置，不要覆盖现有基线文件。

同日实测三轴位移分别为 X +0.509 m、Y +0.592 m、Z +0.563 m（人工目标约
0.5 m），方向全部符合 ENU；桥接输出的 30 秒静止验收为 10.004 Hz、95%
延迟 1.4 ms、最大单帧跳变 1.6 mm、总漂移 2.3 mm。最低频率门限设为
9.5 Hz，用于容纳标称 10 Hz 发布器的 5% 调度抖动。

连接飞控时保持拆桨、机架固定，只接 USB，不接动力电池：

```bash
roslaunch simple_target_follow hardware_bringup.launch
```

检查闭环：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /target_follow/hardware_status
```

随后手持整机完成静止 5 分钟、沿 X/Y/Z 各移动 0.5 m、旋转 90°的测试。
移动方向、距离和航向必须同时与场地坐标一致；静止漂移、延迟和跳变应记录
到验收表。任何轴颠倒、比例错误或突然重定位都禁止进入装桨测试。

PX4 的 EKF 外部视觉源、延迟和高度源参数必须根据实际 PX4 固件和传感器
测量结果在 QGroundControl 中配置并保存备份。不要照抄 SITL 参数，也不要
让多个未经对齐的位置或航向源同时参与融合。

## 4. 一体化无桨启动

项目提供默认禁止解锁的一体化入口，同时启动相机、Xsens、Mid-360、MAVROS、
健康监控和任务节点：

```bash
source /opt/ros/noetic/setup.bash
source ~/drone_ws/devel/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch simple_target_follow autonomous_bringup.launch
```

第一次运行默认不启动 FAST-LIVO。传感器数据验收、外参和雷达网络检查通过
后，使用：

```bash
roslaunch simple_target_follow autonomous_bringup.launch use_fast_livo:=true
```

两个命令均保持 `allow_arming=false`、不使用模拟检测器、不主动执行航线。
`use_fast_livo:=true` 时必须同时保持 `use_livox:=true`。

一体化入口还会强制等待 `/target_follow/sensor_status`：只有相机、深度、Xsens、
Mid-360 以及启用 FAST-LIVO 时的视觉位姿均持续新鲜且状态为 `ready=true`，
任务节点才会通过准备检查。状态缺失、超过 2 秒未更新、JSON 格式错误或任一
必需数据流异常都会拒绝进入 OFFBOARD/解锁流程。仿真入口不启用该真机门控。

## 5. 无桨飞控与 OFFBOARD 验收

1. 拆下全部螺旋桨，固定机架，只接 USB。
2. 备份原始 PX4 参数，记录飞控型号、固件版本和机架类型。
3. 完成加速度计、水平面、指南针和遥控器校准。
4. 设置并实测遥控器人工模式、位置模式、返航/降落和急停开关。
5. 核对 1～4 号电机序号、旋向及失控保护，但不安装螺旋桨。
6. 保持 `allow_arming:=false` 启动真机任务入口：

```bash
roslaunch simple_target_follow target_follow_hardware.launch \
  allow_arming:=false wait_for_start:=true
```

此时任务不得解锁。确认 `/mavros/setpoint_position/local` 能以约 20 Hz 发布，
断开定位、停止 MAVROS 或制造陈旧位姿时，任务应拒绝开始或进入异常降落
逻辑。无桨阶段不使用 `allow_arming:=true` 测试自动航线。

## 6. 装桨后的分级试飞

装桨测试必须在全包围桨叶保护罩、保护场地、系留条件或等效防护下进行，
并由一人专门持遥控器准备接管。每一级通过并保存日志后才能进入下一级：

1. 手动/位置模式悬停，验证方向、振动和人工接管；
2. 0.5 m 定高悬停；
3. 1.2 m 定高悬停，验证 120±10 cm；
4. OFFBOARD 起飞后保持单点，不执行巡查；
5. 单个 0.5 m 方格和单行航迹；
6. 三个禁飞格绕行；
7. 完整 60 格航线，不启用识别；
8. 启用真实模型和正式地面站；
9. 完整 300 s 赛题流程和 45°返航降落。

只有现场负责人确认电池、定位、围栏、遥控接管和降落区域均正常后，最后
才可显式允许任务解锁：

```bash
roslaunch simple_target_follow target_follow_hardware.launch \
  allow_arming:=true \
  wait_for_start:=true \
  use_ground_station:=true \
  use_detector:=true
```

`allow_arming:=true` 不应写入 launch 默认值或开机服务。正式任务必须等待
地面站的一键启动，禁止用模拟检测器生成比赛结果。

## 7. 每次飞行前的放飞门槛

以下条件必须全部满足：

- 螺旋桨、保护罩、机架、重心和电池固定可靠；
- 6S 电池遥测存在，电压不低于项目设置的 21 V，剩余电量满足任务要求；
- `/target_follow/sensor_status` 和 `/target_follow/hardware_status` 均为 ready；
- MAVROS 已连接，局部位姿新鲜且原点位于红色起飞区中心；
- 手持测试确认 X/Y/Z 和航向方向正确；
- PX4 与任务软件围栏均已检查；
- 遥控器接管、急停和失去定位后的降落策略已实测；
- 三个禁飞格与现场抽取结果一致；
- 起降区、赛场上空和围栏内无人。

任一条件不满足时不得通过降低安全阈值、关闭电池要求或关闭围栏来强行起飞。

## 8. 飞后验收与留档

任务结束后保存 rosbag、PX4 飞行日志和以下报告：

```text
/tmp/target_follow_mission_results.json
/tmp/target_follow_mission_assessment.json
```

检查 `/target_follow/assessment` 中的覆盖格数、禁飞区进入次数、高度误差、航迹
偏差、总时间、降落角和触地点。`automated_checks_pass=true` 只说明自动
飞行指标通过；真实动物识别与正式实体地面站也通过后，才能判定整题复现
完成。

## 9. 仍需完成的实体环节

- 实测 FAST-LIVO2 的 `camera_init` 坐标轴、机体位姿含义及 PX4 EKF 融合；
- 决定使用 Mid-360 内置 IMU 还是 MTi-680，并为最终方案重新核对外参与时延；
- 标定并量化定位漂移、延迟、重定位和传感器失效行为；
- 训练五类动物 ONNX 模型并完成现场精度验证；
- 完成 GPIO/PWM 降落 LED 和微控制器正式地面站；
- 完成 6S 供电、降压、续航、重心和电磁干扰测试；
- 按第 5 节逐级完成保护场地真机飞行。
