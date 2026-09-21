# 场地定位验收工具

这三个节点只订阅位姿并生成 JSON 报告，不发布位置设定点、不切换 PX4 模式、
不请求解锁。测试时仍须拆除全部螺旋桨。

## 启动数据链

当前安全基线使用 Mid-360 内置 IMU 的纯激光惯导（LIO）。D435i 仍以 30 Hz
运行，供目标跟踪和地面站使用，但默认不送入 FAST-LIVO2。运行完整定位测试
时需要启动 D435i、Mid-360、FAST-LIVO2 和 MAVROS；Xsens 建议同时启动用于
健康检查，但当前不参与 FAST-LIVO2 融合：

```bash
roslaunch simple_target_follow autonomous_bringup.launch \
  use_fast_livo:=true allow_arming:=false wait_for_start:=true
```

不得直接运行 FAST-LIVO2 自带的 `mapping_mid360_mti680.launch`：其中的
同名图像重发布会形成反馈环路，超大图像队列和无限点云保存会耗尽内存。
需要台架诊断视觉融合时，才显式设置 `fast_livo_use_image:=true`；此时项目
入口只发送 10 Hz 的节流副本，并且必须持续监视 FAST-LIVO2 内存占用。

确认以下话题持续存在后再运行测试：

```bash
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose
rostopic echo -n 1 /target_follow/sensor_status
rostopic echo -n 1 /target_follow/hardware_status
```

## 1. 静止定位质量

将整机固定并保持静止至少 30 秒：

```bash
roslaunch simple_target_follow localization_validation.launch
```

飞控尚未连接时，可以验收已经过安全门控、准备送给 MAVROS 的位姿：

```bash
roslaunch simple_target_follow localization_validation.launch \
  require_local:=false
```

`/fast_livo/unconverted_vision_pose` 保留为上游诊断话题。
`fast_livo_pose_bridge.py` 在原生单位外参和 ENU 轴向验证通过后数值直通到
默认 `/mavros/vision_pose/pose`，并拒绝陈旧、非法或不连续的数据。

检查频率、消息时间延迟、NaN、四元数归一化、单帧位置跳变和静止漂移。报告：

```text
/tmp/target_follow_localization_quality.json
```

## 2. FAST-LIVO2 与 PX4 融合位姿

拆桨后缓慢手持移动并旋转整机：

```bash
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_fusion:=true duration:=30
```

报告比较同步后的视觉位姿和 PX4 局部位姿，包括位置误差和航向误差：

```text
/tmp/target_follow_pose_fusion.json
```

如果两者原点定义不同，本测试会直接显示恒定位置误差。此时应先核对 PX4 EKF
原点和 FAST-LIVO2 坐标变换，不能简单放宽阈值。

## 3. 坐标轴方向与比例

每次只测试一个方向。启动后先静止 3 秒，看到终端提示后，在 10 秒内沿指定
场地方向移动 0.5 m：

```bash
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_axis:=true axis:=x expected_distance:=0.5 \
  pose_topic:=/fast_livo/unconverted_vision_pose
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_axis:=true axis:=y expected_distance:=0.5 \
  pose_topic:=/fast_livo/unconverted_vision_pose
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_axis:=true axis:=z expected_distance:=0.5 \
  pose_topic:=/fast_livo/unconverted_vision_pose
```

航向测试的正方向是 ENU 坐标系从上向下看逆时针约 90°。角度参数以弧度表示：

```bash
roslaunch simple_target_follow localization_validation.launch \
  run_quality:=false run_axis:=true axis:=yaw expected_distance:=1.5708 \
  pose_topic:=/fast_livo/unconverted_vision_pose
```

报告分别写入 `/tmp/target_follow_axis_x_test.json` 等文件。X、Y、Z 和 yaw 必须
全部通过，且人工观察方向与场地定义一致，才能继续无桨 OFFBOARD 验收。

### 2026-07-31 本机实测记录

- 30 秒原始 LIO 静止：10.004 Hz，漂移 2.1 mm，最大跳变 1.8 mm；
- 约 90° 逆时针偏航：估计 89.5°，旋转中心假位移约 1.0 cm；
- X 前移：+0.509 m，横向串轴 0.009 m，高度变化 0.011 m；
- Y 左移：+0.592 m，前后串轴 0.021 m，高度变化 0.011 m；
- Z 上移：+0.563 m，水平串轴约 0.047 m；
- 桥接输出 30 秒静止：10.004 Hz，95% 延迟 1.4 ms，最大跳变
  1.6 mm，总漂移 2.3 mm。

以上结果确认当前安装的 FAST-LIVO 世界轴符合 ROS ENU。它只完成
FAST-LIVO 到 MAVROS 输入侧验收；PX4 EKF 融合和
`/mavros/local_position/pose` 一致性仍必须在拆桨连接飞控后单独通过。

## 阈值

初始阈值位于 `config/localization_validation.yaml`。阈值用于发现坐标系、
时间同步和定位异常；测试失败时应先检查数据和配置，不应仅为了得到 `pass`
而放宽阈值。标称 10 Hz 的 LIO 最低门限为 9.5 Hz，只用于容纳 5% 的系统
调度抖动；延迟、跳变、漂移和姿态阈值未放宽。
