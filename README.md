# 无人机 LIO 定位与视觉目标跟随

Ubuntu 20.04 / ROS Noetic 工作空间，当前使用一个包 `simple_target_follow`。
支持 Mid-360 内置 IMU 纯 LIO、PX4 位姿桥接、只读自检、40 秒自动悬停，以及双圆环＋十字目标跟随任务。
原 `/home/lfy/catkin_ws` 保留不动；驱动和 FAST-LIVO2 依赖位于 `/home/lfy/drone_ws`。

视觉跟随目前使用彩色图像的中心和图案宽度，保持高度和航向；尚未接入深度测距或避障。
代码检查通过不代表安装外参、跟随方向或实体飞行已验收。

## 构建与终端环境

```bash
source /opt/ros/noetic/setup.bash
source /home/lfy/drone_ws/devel/setup.bash
cd /home/lfy/uav_ws
catkin_make
source devel/setup.bash
```

每个新终端都需要 source 上述三个环境。工作空间根目录的 `CATKIN_IGNORE`
用于隔离嵌套扫描，不影响从这里执行 `catkin_make`。

## 入口与任务边界

| 启动文件 | 用途 |
| --- | --- |
| `lio_bringup.launch` | 默认仅 Mid-360＋FAST-LIVO2 纯 LIO；可同时启用 MAVROS 和位姿桥接 |
| `hardware_bringup.launch` | 单独启动 MAVROS 和飞控健康监测 |
| `hover_preflight_check.launch` | 一次性只读自检，结果保存到 `/tmp/uav_hover_preflight.json` |
| `offboard_hover_test.launch` | 自动起飞 1.2 m、悬停 40 秒、AUTO.LAND |
| `marker_follow.launch` | 启动 RGB 相机和完整追踪任务；默认也可用于不飞行的图案识别 |
| `color_preview.launch` | 独立 HSV 色块检测，无飞行指令，与十字图案检测不同 |
| `localization_validation.launch` | 只读定位质量检查，可配置轴向和位姿对比测试 |
| `px4_vision_audit.launch` | 只读检查 PX4 外部视觉融合配置 |
| `sensor_bringup.launch` | 保留的 RealSense 深度＋Xsens 旧传感器入口，当前纯 LIO 和追踪任务不需要 |

默认启动 LIO：

```bash
roslaunch simple_target_follow lio_bringup.launch
```

完整定位链：

```bash
roslaunch simple_target_follow lio_bringup.launch \
  start_mavros:=true publish_to_mavros:=true \
  fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600
```

上述两种方式二选一，串口路径按实际设备确认。不要重复启动驱动、MAVROS 或控制节点。
完整链稳定后运行 `hover_preflight_check.launch`，然后选择一个飞行任务。
两种任务都要显式设置 `allow_arming:=true`，并单独调用各自的 `/start` Trigger **服务**才会起飞。
追踪任务另要求 `tracking_geometry_validated:=true`；该参数表示现场几何验证已完成。

追踪正常流程：起飞 → 悬停 3 秒 → 连续确认完整图案 → 最多跟随 30 秒 → 返回起飞悬停点 → AUTO.LAND。
首次 15 秒找不到目标时原地等待后返回；锁定后短时丢失保持设定点，丢失超过 3 秒返回。
人工切出 OFFBOARD 后停止控制，不重新夺回模式。

## 代码与文档

ROS 脚本负责启动节点；公共飞行生命周期、追踪任务和遥测收集放在 Python 模块中。
定位与感知的纯计算模块继续独立保留，便于离线测试。

- [代码架构](src/simple_target_follow/docs/ARCHITECTURE.md)：模块职责、调用关系和扩展位置。
- [定位验收](src/simple_target_follow/docs/LOCALIZATION.md)：LIO、桥接、轴向及 PX4 融合检查。
- [悬停任务](src/simple_target_follow/docs/OFFBOARD_HOVER_TEST.md)：启动服务、退出和接管。
- [视觉追踪](src/simple_target_follow/docs/MARKER_FOLLOW.md)：跟随规则、几何参数和 JSONL 日志。
- [迁移记录](src/simple_target_follow/docs/MIGRATION.md)：原始源码映射及历史哈希。
- [历史参考](src/simple_target_follow/reference/README.md)：保留的赛题控制器、参数和补丁。

`docs/legacy_*` 为旧资料快照，操作以当前文档为准。
原机械安装变化后的标定记录位于 `calibration_runs/`，与运行代码分开保存。

## 日志与测试

每次启动追踪节点，自动新建 `~/.ros/marker_follow_logs/marker_follow_*.jsonl`。
它记录识别状态、跟随计算、目标设定点与当时实测位置、等待和结束原因；不保存原始视频。
计算出的设定点不代表飞机已到达目标位置，实际运动还需结合 PX4 ULog 分析。

```bash
PYTHONPATH=src/simple_target_follow/src python3 -m unittest discover -s src/simple_target_follow/tests -v
```

该测试不会连接飞控、解锁或起飞。实体测试仍需完成定位与安装几何验收，并保留遥控接管。
