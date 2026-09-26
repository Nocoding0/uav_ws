# 1.2 m OFFBOARD 定点与温和抗扰试验

本任务节点只运行于已验证的 PX4 v1.12.3 + Mid-360 纯 LIO 链路。它**会**在收到
明确启动请求后请求 OFFBOARD、解锁，并在完成后请求 AUTO.LAND；不修改 PX4 参数。
目标高度是进入 OFFBOARD、解锁后采集的 PX4 local Z 加 1.2 m，XY 和 yaw 固定在
当时位置。达到目标并稳定 1.5 秒后才开始计时 40 秒。

## 飞行前

1. 在拆桨状态先验证完整链、遥控切出 OFFBOARD 的人工接管和定位停止时的 PX4
   failsafe；检查真实电池遥测、电量门限、桨叶/场地/人员隔离及 PX4 ULog 记录。
2. 当前 PX4 高度源为 `EKF2_HGT_MODE=3`（外部视觉），且
   `EKF2_AID_MASK=24`（视觉位置+航向）。第一轮不改这些参数。必须确认本场景中
   雷达 Z 轴、水平位置及航向均正确，静止时没有发散。
3. 已读取的配置为 `COM_OF_LOSS_T=1.0`、`COM_OBL_RC_ACT=0`（遥控可用时切
   POSCTL）、`COM_RC_OVERRIDE=1`（**未**启用 OFFBOARD 摇杆接管 bit）。
   放飞前实测模式开关能切回已经验证的人工/定高模式，且定位丢失时可安全着陆；
   切走 OFFBOARD 后本脚本不重新夺回控制权。别依赖拨杆以外的摇杆接管。
4. `lio_bringup.launch` 默认不启动 MAVROS、桥接；本任务要求**完整模式**，且不能
   有第二个 MAVROS/Livox/FAST-LIVO 实例占用硬件。

## 启动与观察

先在终端 A 启动已验证的定位链（飞控路径按实际 `/dev/serial/by-id/` 选择）：

```bash
source /opt/ros/noetic/setup.bash
source /home/lfy/drone_ws/devel/setup.bash
source /home/lfy/uav_ws/devel/setup.bash
roslaunch simple_target_follow lio_bringup.launch \
  start_mavros:=true publish_to_mavros:=true \
  fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600
```

终端 B source 相同环境后启动任务节点。仅 launch 不会解锁，必须显式允许解锁：

先运行一次只读自检。它会等待雷达、内置 IMU、FAST-LIVO、位姿桥接、飞控、
估计器、电池和静止稳定窗口，并以退出码 0/2 表示通过/失败：

```bash
roslaunch simple_target_follow hover_preflight_check.launch
```

详细结果同时保存到 `/tmp/uav_hover_preflight.json`。自检通过只表示当前软件数据
条件满足；它不会检查桨叶、场地、人员隔离或遥控器真实模式映射，也不会切模式、
解锁或发送设定点。

随后启动任务节点：

```bash
roslaunch simple_target_follow offboard_hover_test.launch allow_arming:=true
```

观察状态，确认 `phase=READY`、`health_reason=null` 后，**最后一次现场确认安全**，
再从第三个已 source 的终端手动发出一次启动请求：

```bash
rostopic echo /target_follow/hover_test/status
rosservice call /target_follow/hover_test/start
```

悬停计时内只做短暂、温和的水平扰动；不要敲打机体、触碰桨叶或遮挡雷达。松开后
脚本持续发原地固定设定点，PX4 的位置控制环负责回位。状态记录扰动次数、最大
水平误差与最近一次恢复时间；超过 0.8 m 水平偏差或 0.4 m 高度误差会请求降落。
需要主动结束任务时：

```bash
rosservice call /target_follow/hover_test/abort
```

若已由遥控器切换模式，本节点不会再请求降落或恢复 OFFBOARD；由飞手接管。
本节点不在空中强制解锁/解除锁定。定位或电池异常时请求 AUTO.LAND，若 PX4
拒绝则停止设定点交由其已配置的 OFFBOARD 丢流保护，飞手必须随时准备介入。
任务完成或飞手接管后，节点保留最终状态但不再发控制指令；在确认落地/人工
接管后可 Ctrl+C 关闭任务节点。

建议先做一轮**不扰动**的自动起飞/定点/降落，再安排抗扰轮次；同步记录
`/mavros/state`、`/mavros/estimator_status`、`/mavros/local_position/pose`、
`/mavros/vision_pose/pose`、`/mavros/battery`、`/target_follow/hover_test/status`
和 PX4 ULog，复核创新、外部视觉拒绝、定点偏差与降落结果。
