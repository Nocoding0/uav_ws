# 无人机定位与目标跟随开发工作空间

从原半成品赛题工程中提取必要代码。原 `/home/lfy/catkin_ws/src` 保留不动。
工作空间迁移后位于 `/home/lfy/uav_ws`。当前 ROS/Python 包名仍为
`simple_target_follow`，后续可再按定位、感知、控制拆包；继续使用
Ubuntu 20.04 / ROS Noetic。

## 当前状态

已具备：独立的 Mid-360 + FAST-LIVO2 纯 LIO 启动入口、定位桥接、PX4
外部视觉只读审计、定位验收工具；以及暂缓开发的蓝色色块只读检测。

尚未实现：色块三维测距、闭环跟随控制、避障。`reference/` 的原控制器仅供下一步重构，不安装、不由新 launch 启动。不能认为复制了控制代码就已经完成实体飞行验收。

当前阶段先验收机载定位和 PX4 融合，不启动视觉跟随。后续视觉实验可用棍子固定一块约 15–20 cm 的蓝色哑光板；目标检测使用 HSV 阈值和轮廓，不需要模型、CUDA、PyTorch 或额外 GPU。

新增的 [OFFBOARD 定点试验](src/simple_target_follow/docs/OFFBOARD_HOVER_TEST.md)
是独立的高风险飞行任务入口：在完整 LIO/MAVROS 位姿链完成验收后，需显式设置
`allow_arming:=true` 并单独调用启动服务才会切模式和解锁。不要把它当作定位
只读检查命令；正式飞行必须先拆桨验证、保留遥控人工接管，并记录 PX4 ULog。

纯 LIO 的完整操作与分级验收命令见 [定位验收路线](src/simple_target_follow/docs/LOCALIZATION.md)。默认入口仅运行 Mid-360 + FAST-LIVO2，不启动飞行任务，也不发送位姿给飞控。

## 文件结构

```text
uav_ws/
  CATKIN_IGNORE                 # 工作空间根目录的隔离标记
  README.md
  src/simple_target_follow/
    launch/                     # 纯 LIO、PX4 审计、传感器及色块预览
    config/                     # 色块阈值、传感器和定位检查配置
    scripts/                    # 安装到 ROS 的节点，不包含飞行控制器
    src/simple_target_follow/   # 图像检测、定位检查、安全检查的纯逻辑
    tests/                      # 迁移的基础测试 + 色块测试
    docs/LOCALIZATION.md        # 首先运行的定位验收步骤
    docs/MIGRATION.md           # 原文件到新文件映射及原文件哈希
    docs/legacy_*               # 旧验收资料，仅供参考
    reference/                 # 原跟随器、双圆标记检测、参数及 FAST-LIVO 补丁
```

## 构建

在新的终端使用传感器工作空间作为下层环境；无需 source 原比赛工程。

```bash
source /opt/ros/noetic/setup.bash
source /home/lfy/drone_ws/devel/setup.bash
cd /home/lfy/uav_ws
catkin_make
source devel/setup.bash
rospack find simple_target_follow
```

这是独立 catkin 工作空间，不是在旧工作空间 `src` 内再复制一个同名包。工作空间根部的 `CATKIN_IGNORE` 不影响以这里的 `src` 为源码目录的独立构建。

## 定位优先

拆桨和检查雷达网线后，先执行 `roslaunch simple_target_follow lio_bringup.launch`。
确定纯 LIO 正常后，才进入 MAVROS 连接及 PX4 参数、融合检查；分步命令和
不能省略的验收条件见 `docs/LOCALIZATION.md`。**不应启动旧赛题任务验证定位。**

## 后续的色块检测预览

```bash
roslaunch simple_target_follow color_preview.launch
```

已有相机节点或在回放 bag 时：

```bash
roslaunch simple_target_follow color_preview.launch start_camera:=false
```

另一个已 source 新工作空间的终端：

```bash
rostopic echo /target_follow/target
rosrun rqt_image_view rqt_image_view
```

在图像工具中选择 `/target_follow/image` 查看框，选择 `/target_follow/mask` 查看二值图。如未安装 rqt_image_view，可使用已有图像查看工具。

默认 640×480、30 FPS，处理上限 10 Hz，以设备实际支持的模式为准。默认相机序列号来自旧工程，可通过 `realsense_serial:=...` 覆盖。其他图像话题可通过 `image_topic:=...` 指定。

修改 `config/color_target.yaml` 后重启节点。H 范围 0–179，S/V 范围 0–255；红色可以设置跨越零的 hue 范围，如 `[170,100,60]` 到 `[10,255,255]`。这不是物体身份识别：不同时间出现的同色目标仍可能被重新确认。

状态 `/target_follow/target` 为 JSON 字符串：

- `valid`：连续确认且当前图像有效；它只表示二维检测有效，不表示可以飞行。
- `stamp`：输入图像时间戳；消费者必须独立检查新鲜度，检测节点停止后不会继续发送失效消息。
- `frame_id`：原图坐标系。
- `center_normalized`：0–1 的图像中心，右/下为正。
- `bbox`：原图像素 `[x,y,width,height]`。
- `reason`：`confirmed`、`confirming`、`no_target`、`ambiguous_targets`、`no_image` 等。

无效状态不应使用上一帧目标继续运动。多个大小相近的色块、过小/过大、贴边或非实心候选会被拒绝。该节点没有深度、距离、目标速度或飞行指令输出。

回放 bag 时，在启动节点前设置 `/use_sim_time=true` 并使用 `rosbag play --clock`，避免旧时间戳被判为过期。

## 保留的定位与飞控工具

```bash
roslaunch simple_target_follow lio_bringup.launch
roslaunch simple_target_follow hardware_bringup.launch
roslaunch simple_target_follow localization_validation.launch
```

这些入口分别启动雷达/定位、MAVROS、只读定位质量检查；不启动飞行任务。
`lio_bringup` 默认只输出原始位姿，独立启动 `hardware_bringup` 并不等于已开始 PX4 外部视觉融合；参考定位验收路线逐步启用桥接。不要与旧工程重复启动同一硬件或同一 MAVROS 实例。

外部依赖仍位于 `/home/lfy/drone_ws`：RealSense 驱动、Livox 驱动、Xsens 驱动、FAST-LIVO2 及其配置/已有补丁。本次没有复制或重新安装它们；新工程不是可以脱离这些依赖随意搬机运行的完整镜像。

沿用了旧工程的 Mid-360 内置 IMU、已验证单位外参假设和纯 LIO 默认模式。机械安装、IMU 来源变化后必须重新校准。电池阈值沿用原机 21 V，改变电池配置时应先核对 `launch/hardware_bringup.launch` 的阈值，不能直接套用。

`docs/legacy_*` 是旧资料快照，部分命令路径和内容仍反映旧工程，请以本 README 的入口为准。

## 下一步开发顺序

1. 用真实相机验证色块、光照、误检、遮挡、重新出现的检测行为。
2. 接入 D435i 对齐深度，仅取色块掩膜内的有效深度；同步 RGB/深度，并标定相机到机体外参。
3. 独立输出目标三维位置和新鲜度；手持整机移动/旋转，验证坐标方向。
4. 从 reference 控制器提取独立控制节点，去除蓝牙、赛题搜索、定时返航和“框大小当距离”的假设。先实现固定高度、固定距离、限速/限加速度、丢失悬停的低速跟随。
5. 用参考节点的检查逻辑验证定位、飞控融合与控制方向，再进行分阶段实体闭环验证。
6. 之后可将色块检测替换为 ArUco 或人员检测，同时复用距离估计和控制接口。

如果相机朝下：优先做平面上的目标居中/定高跟随；如果朝前：优先做朝向目标和固定距离跟随。实际方向确定前，不把两套坐标规则混用。

## 测试

```bash
cd /home/lfy/uav_ws
PYTHONPATH=src/simple_target_follow/src python3 -m unittest discover -s src/simple_target_follow/tests -v
```

测试通过只验证代码逻辑，不代表传感器、外参或真实飞行已验收。
