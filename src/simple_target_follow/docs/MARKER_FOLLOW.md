# 双圆环＋十字目标跟随（待安装外参复核）

`marker_follow.launch` 把 9 月 22 日的 Mid-360 内置 IMU 纯 LIO 定位、PX4
健康检查和 OFFBOARD 起飞流程，与旧任务的双圆环＋十字图案检测器接在同一控制节点中。
它启动相机和一个飞行控制节点；定位链需单独通过 `lio_bringup.launch` 启动。
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
