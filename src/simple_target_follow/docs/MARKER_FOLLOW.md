# 双圆环＋十字目标跟随（待安装外参复核）

`marker_follow.launch` 把 9 月 22 日的 Mid-360 内置 IMU 纯 LIO 定位、PX4
健康检查和 OFFBOARD 起飞流程，与旧任务的双圆环＋十字图案检测器接在同一控制节点中。
它启动相机和一个飞行控制节点；定位链需单独通过 `lio_bringup.launch` 启动。
代码入口仍是 `scripts/offboard_hover_test.py`；公共起飞与降落位于
`offboard_mission.py`，识别、跟随、返回和日志调用位于 `marker_mission.py`。
模块关系见 [代码架构](ARCHITECTURE.md)。
不要同时运行 `offboard_hover_test.launch` 或旧 `drone_control` 跟随任务，避免多个
节点向 `/mavros/setpoint_position/local` 发送不同设定点。

任务顺序：显式启动请求 → 起飞到相对高度 1.2 m → 原地悬停 3 秒 → 连续 5 帧
确认完整双圆环＋十字 → 前视相机根据图案左右偏差和图像大小，限速、限半径地水平跟随并保持高度和航向 → 目标短时
丢失时保持设定点 → 丢失超过 3 秒或任务到时，返回起飞悬停点 → AUTO.LAND。
跟随仅使用同时识别到十字和内外两个圆环的连续画面；任一部分丢失即暂停跟随，
恢复完整图案并再次连续确认 5 帧后才继续。局部识别结果仍发布供视觉检查，
不会单独驱动跟随。
初次未找到图案时原地等待最多 15 秒，不主动搜索移动。返回超时或定位异常时
请求 AUTO.LAND；人工切出 OFFBOARD 后节点停止夺回控制权。

## 当前不能直接飞行的配置

`marker_follow.launch` 默认 `allow_arming=false`，并且
`tracking_geometry_validated=false`。必须先确认新雷达安装后 LIO 的机体轴向、
旋转时假位移、PX4 视觉参考点偏移和融合质量。相机已确认为朝前，配置中的
`camera_orientation` 已设为 `forward`。还需拆桨确认图像右方与机体右方的
对应关系、相机偏航安装角度，以及期望跟随距离对应的图案图像宽度。
按实测结果设置 `config/marker_follow.yaml` 中的 `camera_yaw_offset_deg` 和
`desired_marker_size`。前视控制保持起飞高度；图像上下偏差不会驱动升降。
原雷达至内置 IMU
单位外参是传感器内部关系，不用传感器在机体上的位移替代它。

## 拆桨视觉检查

不连接 MAVROS 也可只启动相机与跟随节点，默认不会发起解锁：

```bash
source /opt/ros/noetic/setup.bash
source /home/lfy/drone_ws/devel/setup.bash
source /home/lfy/uav_ws/devel/setup.bash
roslaunch simple_target_follow marker_follow.launch
```

用 `rostopic echo /target_follow/marker_follow/marker` 检查图案是否被完整识别、
是否连续锁定。标记后的图像在 `/target_follow/marker_follow/image`。需要验证
定位时，在另一个终端启动：

```bash
roslaunch simple_target_follow lio_bringup.launch \
  start_mavros:=true publish_to_mavros:=true
```

接着执行 `docs/LOCALIZATION.md` 的拆桨轴向、静止质量、PX4 融合检查和
`hover_preflight_check.launch`。检查通过后，还要在实物上确认桨叶、场地、
遥控接管和 failsafe。完成新安装位姿标定后，才能显式启用两个启动开关；
它们不能写入默认值。最终任务状态和手动终止服务分别是
`/target_follow/marker_follow/status` 与 `/target_follow/marker_follow/abort`。

当前验证仅包括合成图像、跟随方向/速度/范围单元测试及 launch 解析；没有在
新安装的实机上完成相机外参、定位验收或飞行验证。

## 识别与跟随日志

每次启动 `marker_follow.launch` 都会在 `~/.ros/marker_follow_logs/` 新建一个
`marker_follow_*.jsonl` 文件，终端也会打印完整路径；旧文件不会被覆盖。
`marker_observation` 记录未识别、部分图案、完整但未锁定、已锁定、图像处理出错等状态，以及
原始归一化中心和图像宽度。状态改变时立即记录，稳定时每秒记录一次。
`follow_step` 在跟随阶段每次计算位置设定点时记录滤波后的中心与宽度、图像误差、
图像年龄、偏航、计算前后的 XY 设定点、设定点增量和 PX4 当时测得的 XY。
`tracking_hold` 记录未锁定时
的原地等待，`tracking_end` 记录结束原因，`mission_state` 记录任务阶段。
`tracking_hold.image_available=false` 表示没有新鲜且可用的图像。
这些是数值与设定点日志，不保存原始图像；`target_xy_after` 是计算出的 PX4 目标，
`actual_xy` 是当时的测量位置，不代表该步目标已被飞机到达。当前前后控制仍以
图像宽度为依据，不使用深度。

查看最新一轮：

```bash
latest_log=$(ls -t ~/.ros/marker_follow_logs/marker_follow_*.jsonl | head -n 1)
echo "$latest_log"
rg '"event": "(marker_observation|follow_step|tracking_hold|tracking_end|mission_state)"' "$latest_log" | tail -n 30
```

要判断是否真的跟随，查同一文件中是否有 `state: confirmed` 和
`follow_step`，再比较其中 `target_xy_before` 与 `target_xy_after`；仅出现
`mission_state: TRACKING` 不足以证明目标已锁定。
