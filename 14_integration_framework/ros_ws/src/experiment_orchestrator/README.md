# experiment_orchestrator

`experiment_orchestrator` 通过一个 YAML 表描述完整实验（环境 + 车辆 + 控制算法 + 参数集 + 日志输出位置）。`scripts/experiment_orchestrator.py` 提供 CLI/ROS 参数入口，自动调用 `simulation_bringup` 与 `algorithm_manager` 的接口，实现一键切换。

功能概览：
- `config/experiments.yaml`：集中定义实验元数据，便于版本管理与结果复现。
- `scripts/experiment_orchestrator.py`：解析 YAML、发送话题/服务请求到其他包，并在 `logs/` 目录内创建对应的记录。
- `launch/experiment.launch`：提交 `experiment_id` 参数后即可启动 orchestrator 节点。

> 当前仅提供骨架逻辑（参数写入 + ROS topic/service 交互）。日志记录与 bag 管理需在后续迭代补齐。
