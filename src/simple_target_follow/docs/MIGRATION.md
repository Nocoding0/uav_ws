# 迁移记录

迁移日期：2026-09-18。来源：`/home/lfy/catkin_ws/src/drone_control`。

只复制必要源码，不复制构建产物、缓存、历史备份、巡查赛题、蓝牙桥、动物检测器或地面站。原项目未修改。

自动替换：Python/ROS 包名为 `simple_target_follow`，健康状态前缀为 `/target_follow/`，launch/config 使用标准目录。相机、MAVROS、Livox、FAST-LIVO 外部话题保持原有接口。

原飞行控制器和标记检测器仅保存在 `reference/`，未安装，没有新 launch 启动它们；它们仍包含赛题流程，尚未改成色块控制。原 FAST-LIVO 补丁也仅存档，没有重新应用。

|来源相对路径|新路径|原文件 SHA256|
|---|---|---|
|`src/drone_control/mission_safety.py`|`src/simple_target_follow/mission_safety.py`|`b03a1c27b613471168c4fd6d74715f47481e15fcbe8ad9ee3d2b98ab7b9a0c21`|
|`src/drone_control/pose_bridge.py`|`src/simple_target_follow/pose_bridge.py`|`0d656a70b7cc6ec1b8a90cb32a1373649ddf30b787ce11df5d714a266e660bd2`|
|`src/drone_control/localization_validation.py`|`src/simple_target_follow/localization_validation.py`|`d7c79e9e4a4c64e35333440e7f23c4cbbb58f309c53429a34007f38e20d78bbb`|
|`scripts/fast_livo_pose_bridge.py`|`scripts/fast_livo_pose_bridge.py`|`63a6267704622309343565cfe13c6c84101ff03b4cc84a3ab76109a445be9767`|
|`scripts/hardware_monitor.py`|`scripts/hardware_monitor.py`|`488a79f32eb7dce5a39e5b7e7ea0a5f40fee6348de45abc6e2e8d19a554cd9bf`|
|`scripts/sensor_monitor.py`|`scripts/sensor_monitor.py`|`c83ba1a8dfee428ab6b2845475629b6da0dc5e4b3f387b26ce6e08c1b885c01a`|
|`scripts/localization_quality_test.py`|`scripts/localization_quality_test.py`|`f1c2203b9d46a75e44ac41a99caa21a757d1ce5173ed26829836772bb802e2eb`|
|`scripts/pose_fusion_test.py`|`scripts/pose_fusion_test.py`|`a5aa4305214ceb527c0d8c4011b9ec6200750b6b80289369e04b81f91d1380a5`|
|`scripts/axis_motion_test.py`|`scripts/axis_motion_test.py`|`9862dae032b7604bdae4d29049e411be6aaed6d6979ef6918d7455b03ed029af`|
|`scripts/launch/sensor_bringup.launch`|`launch/sensor_bringup.launch`|`1fde2a1f4db467452c12412c8ff8579e1a86d3d79f162bf55e9680e2adf7dd28`|
|`scripts/launch/hardware_bringup.launch`|`launch/hardware_bringup.launch`|`165e8c0183acb559274c258dba88105d5ab06c2bc10f99fb3510ddb13e805b30`|
|`scripts/launch/localization_validation.launch`|`launch/localization_validation.launch`|`cb064de47b676fdabcabf6327f8125a2b8d0702b5a25aa92cbb77b7fc55f74d3`|
|`scripts/config/xsens_mti680.yaml`|`config/xsens_mti680.yaml`|`d7f076ab5f6930b4bab6a8c34122240be27a9a9bae170e79f093bdf81e21b78c`|
|`scripts/config/localization_validation.yaml`|`config/localization_validation.yaml`|`703add22b096d287246045e36163679c6bc51fea5b0f0567affcbc05f8a52968`|
|`scripts/tests/test_mission_safety.py`|`tests/test_mission_safety.py`|`ad61fc558c1eb4bc93474833556543c2f08fac5276c66b0ecc1bbef3cd9d31d2`|
|`scripts/tests/test_pose_bridge.py`|`tests/test_pose_bridge.py`|`ab32a8c3e75a54674bf89cffa69d2741cd16914a139e7cba4178259c2b93abd4`|
|`scripts/tests/test_localization_validation.py`|`tests/test_localization_validation.py`|`66b8f35c3d82ab6802c0d68db12436dc7d82287f84b96fee95227cc1cbfd13e2`|
|`scripts/takeoff_hover_land.py`|`reference/takeoff_hover_land.py`|`7e16188f360290ccd7563e9cc816ff3ac8ddd6eb06ee19cb15b14e9eaa627e7d`|
|`src/drone_control/marker_detection.py`|`reference/marker_detection.py`|`34038c73d9cbaac508b89cad58eb84f8267831969ddc9f6a970ef960f6ca3f73`|
|`scripts/config/forward_marker_follow.yaml`|`reference/forward_marker_follow.yaml`|`8a9e9895e20760eda536266d830768d8846ea8c9f4e610e47338e91541533198`|
|`scripts/config/downward_marker_follow.yaml`|`reference/downward_marker_follow.yaml`|`b336c3f18bd56f64b075f44b0e360e55ff687eaee039a29b90bd450f7cc6fbc9`|
|`scripts/config/takeoff_hover_land.yaml`|`reference/takeoff_hover_land.yaml`|`8585bd9008e6839a00e6c7280b559b2ea02394cc2552e3898876bbb92700f71c`|
|`scripts/FIELD_VALIDATION.md`|`docs/legacy_FIELD_VALIDATION.md`|`8b545e66ad0a3b75ed56f18e05d2dd97ccf05f711cf5bbe9034421b5ccf9711c`|
|`scripts/HARDWARE_BRINGUP.md`|`docs/legacy_HARDWARE_BRINGUP.md`|`9e4e77092568df76428316572e1ca9c3804f86e91adb2fcd47455a9d4ff2fef6`|
|`patches/fast_livo_bounded_queues.patch`|`reference/patches/fast_livo_bounded_queues.patch`|`0336bb5e12a88485778b2028732981955666e4de8090403e8ea6d558143b4cf7`|
|`patches/fast_livo_visual_memory.patch`|`reference/patches/fast_livo_visual_memory.patch`|`c140f89aee10b04b68156958d763588a2edd18da0fb3e626fd5df553cb02eeca`|
|`patches/fast_livo_internal_buffers.patch`|`reference/patches/fast_livo_internal_buffers.patch`|`47092db4449e28aa2b41aa52182a52bdd93bf108620d6406998e9ba915e96271`|
