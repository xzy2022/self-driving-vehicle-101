# 自主驾驶实验一体化框架（草案）

该目录包含一个新的 ROS Noetic 集成框架，用于在 Gazebo 仿真下快速复用仓库中已有的数据、车辆模型与算法包。`ros_ws` 是全量 catkin 工作空间，目前主要提供三个功能包：

| 包 | 作用 | 关键文件 |
| --- | --- | --- |
| `simulation_bringup` | 管理不同车辆版本、环境世界、底层传感器/控制节点的组合；对齐 Gazebo 与 RViz 的启动。 | `config/vehicles.yaml`、`config/environments.yaml`、`launch/simulation_bringup.launch` |
| `algorithm_manager` | 以配置驱动的方式加载不同路径跟踪控制节点（Pure Pursuit、MPC 等），支持运行时切换与参数刷新。 | `config/controllers.yaml`、`scripts/algorithm_switcher.py` |
| `experiment_orchestrator` | 在一条 launch/脚本指令内绑定「环境 + 车辆 + 控制算法 + 参数集 + 日志」，统一输出实验记录。 | `config/experiments.yaml`、`scripts/experiment_orchestrator.py` |

## 工作区结构

```
ros_ws/
 ├── src/CMakeLists.txt        # catkin 初始化
 ├── src/algorithm_manager/…
 ├── src/experiment_orchestrator/…
 └── src/simulation_bringup/…
```

- 可以通过 `cd 14_integration_framework/ros_ws && catkin_make` 生成 `devel` 与 `build` 目录。
- 如需复用旧实验，可将 `11~13` 中的包复制到 `ros_ws/src`，或通过 `environments.yaml`/`experiments.yaml` 直接指向原目录。
- `tests/` 目录提供了面向 `04~08` 阶段的快捷验证脚本（headless test mode），运行前请先 `source` 工作区。

## 下一步建议
1. 在 `simulation_bringup/launch` 中补全实际 `<include>`，引用仓库现有包以验证车辆与状态反馈链路。
2. 为 `algorithm_manager` 注册至少两种控制算法（可直接引入 `pure_persuit` 并再扩展一个新算法），并测试话题切换的连贯性。
3. 在 `experiment_orchestrator` 中补齐日志落盘逻辑（ROS bag + YAML/CSV 元数据），打通一键复现实验的流程。
