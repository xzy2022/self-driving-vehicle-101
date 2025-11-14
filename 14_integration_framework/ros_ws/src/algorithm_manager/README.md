# algorithm_manager

- 通过 `config/controllers.yaml` 声明可用控制算法、依赖话题、参数文件和对应的 launch/include 节点。
- `scripts/algorithm_switcher.py` 在运行时读取 YAML，监听服务与参数更新请求，根据需要 `roslaunch` 或停止某个算法。
- `launch/algorithm_manager.launch` 用于在更高层的 `experiment_orchestrator` 中被 `<include>`，同时提供默认参数空间（如 `~active_controller`）。

> 目前仅包含骨架代码，尚未绑定具体的控制算法，待与 `pure_persuit` 或新包接入。
