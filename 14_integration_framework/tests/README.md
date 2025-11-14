# Integration Tests


## 模型加载测试
以下测试面向 04~06 阶段的主体功能。所有脚本默认将 `SELF_DRIVING_REPO_ROOT` 设置为仓库根路径，随后调用对应 launch：

1. **stage04_box_demo**（URDF 方块）：`bash tests/stage04_box_demo.sh`
2. **stage05_basic_car**（基础 smart 车体）：`bash tests/stage05_basic_car.sh`
3. **stage06_3d_car**（带 mesh 的车辆）：`bash tests/stage06_3d_car.sh`

脚本内部使用 `session_mode:=test`，即 headless + paused，更适合快速验证；如需长期运行，可将脚本中的 `session_mode` 调整为 `runtime` 或直接执行 `roslaunch simulation_bringup stageXX.launch session_mode:=runtime`。

在执行前请确保：
- 已在 `14_integration_framework/ros_ws` 内运行过 `catkin_make` 并 `source devel/setup.bash`。
- Gazebo、roslaunch 等依赖均可用。
- 若要可视化（长期模式），可自定义传参 `session_mode:=runtime` 并增加 `gui:=true`。

```shell
# 两个终端分别执行
rosrun gazebo_ros gzclient
bash tests/stage04_box_demo.sh

# 停止前面的两个程序，继续分别执行
rosrun gazebo_ros gzclient
bash tests/stage05_basic_car.sh

# 停止前面的两个程序，继续分别执行
rosrun gazebo_ros gzclient
bash tests/stage06_3d_car.sh
```