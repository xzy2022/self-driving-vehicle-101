# simulation_bringup

统一管理 Gazebo 世界、车辆模型以及底层状态节点的 bringup 包。`config/vehicles.yaml` 与 `config/environments.yaml` 将车辆阶段（底盘、传感器配置）和仿真场景解耦，为后续实验脚本提供可选项。

- `launch/stage04_box_demo.launch`、`stage05_basic_car.launch`、`stage06_3d_car.launch` 分别复刻 `04_intro_to_urdf`、`05_basic_car`、`06_3d_car_model` 的主体功能，`session_mode` (`runtime`/`test`) 让同一入口兼顾长期调用与短期验证。
- `launch/simulation_bringup.launch` 提供 `vehicle_id` 与 `world_id` 的入口参数，可由 CLI 或 `experiment_orchestrator` 调整；`vehicles.yaml` 中的 `long_term_args` 与 `test_args` 是调用参考。
- `scripts/environment_loader.py` 在启动后加载 YAML，发布当前选中的环境/车辆信息，方便 RViz/日志模块订阅。

未来可在 launch 文件内 `<include>` 仓库里已有的 `car_model`、`waypoint_loader` 等包，实现真正一键启动。
