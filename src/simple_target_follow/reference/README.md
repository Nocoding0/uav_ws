# 历史参考源码

本目录保留从旧 `drone_control` 迁移时的原始赛题控制器、图案检测器、参数和 FAST-LIVO 补丁。
这些文件不参与 Python 包安装，也没有当前 launch 启动它们；不要直接作为当前飞行入口。

当前控制流程位于 `src/simple_target_follow/offboard_mission.py` 和 `marker_mission.py`。
当前图案检测器位于 `src/simple_target_follow/marker_detection.py`，从历史实现发展而来，已单独测试。
本目录的参数只用于追溯旧配置；当前任务使用 `config/offboard_hover_test.yaml` 和 `config/marker_follow.yaml`。

补丁仅存档，不能据此判断下层 FAST-LIVO 当前是否已应用补丁。
迁移来源与当时哈希见 [迁移记录](../docs/MIGRATION.md)。模块关系见 [代码架构](../docs/ARCHITECTURE.md)。
