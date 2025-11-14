# 项目结构与关键文件速览

以下内容按目录划分，帮助快速定位可复用的 ROS/Gazebo 资源及已经实现的算法模块。所有路径均相对仓库根目录。

## 04_intro_to_urdf
- `launch/spawn_box_urdf.launch`、`launch/spawn_box_xacro.launch`、`launch/spawn_xacro.launch`：演示如何通过 `roslaunch` 在 Gazebo 中加载简单 box 模型，可借鉴 launch 架构与参数传递方式。
- `urdf/box.urdf`、`urdf/box.xacro`：最基础的 URDF/Xacro 模型文件，便于了解机器人模型描述的起点。

## 05_basic_car
- `urdf/smart.xacro`：首次出现的车辆 Xacro，定义车体尺寸、质量、轮胎参数与转向机构，是后续控制实验的底座模型。
- `launch/spawn_car.launch`：调用 `car_model` 包的 `spawn_xacro.launch`，展示如何通过 `<include>` 引入共用 spawn 脚本。

## 06_3d_car_model
- `meshes/vehicle_body.dae`、`meshes/wheel.dae`：提供更逼真的车体/轮胎网格。
- `urdf/smart.xacro`：在基本模型上引用 3D 网格，实现 Gazebo 视觉效果增强；可复用材质及 mesh 引用方式。

## 07_control_joints_with_ros_topic
- `config/smart_control_config.yaml`：首次给出四个轮胎+转向关节的 PID 配置，供 `ros_control` 载入。
- `launch/control.launch`：负责加载 PID 参数、启动控制器以及 `robot_state_publisher`。为后续节点（速度/方向闭环）提供标配 launch 模板。

## 08_control_direction_and_speed
- `scripts/cmdvel2gazebo.py`：将 `/smart/cmd_vel` 的 `Twist` 指令转换为左右转向角与后轮速度，并发布到各控制器话题，实现 Ackermann 几何控制。理解该节点有助于自定义上层轨迹跟踪器输出接口。
- `launch/control.launch`：在 07 基础上加入 `cmdvel2gazebo` 节点，展示动力学/控制解耦方式。

## 09_simple_localization
- `scripts/vehicle_pose_and_velocity_updater.py`：订阅 `/gazebo/model_states`，计算车辆中心与后轴坐标（`/smart/center_pose`、`/smart/rear_pose`），并发布 `/smart/velocity`。这是实现路径跟踪闭环的关键状态来源。
- `launch/control.launch`：默认启动 `cmdvel2gazebo` 与 `vehicle_pose_and_velocity_updater`，形成「命令 → Gazebo → 状态反馈」闭环。

## 10_show_car_in_rviz
- `rviz_config/smart.rviz`：预设 RViz 视图，叠加车辆姿态与路径。便于调试路径跟踪算法时可视化轨迹。
- `launch/control.launch`：在 09 基础上预留了 `transform_publisher` 节点位置（注释掉），提示可以扩展 TF 广播链路。

## 11_path_following_part_1
- `waypoint_loader/scripts/waypoint_loader.py`：将 CSV （`waypoint_loader/waypoints/waypoints.csv`）解析为 `styx_msgs/Lane`，并发布 `/base_waypoints` 与 `/base_path`，其中 `Lane.msg`、`Waypoint.msg` 位于 `styx_msgs/msg`。
- `car_model/scripts/cmdvel2gazebo.py`、`vehicle_pose_and_velocity_updater.py`：延续上一阶段的控制与定位节点，形成可复用的底层平台。
- `styx_msgs` 包：定义 `Lane.msg`、`Waypoint.msg` 等自定义消息，供路径模块共享接口。

## 12_path_following_part_2
- `waypoint_updater/scripts/waypoint_updater.py`：利用 SciPy `KDTree` 查找最近航点，并发布定长的 `final_waypoints`（加上 `final_path` 供可视化）。`LOOKAHEAD_WPS` 常量控制窗口长度，是后续控制器的目标序列来源。
- `waypoint_loader`、`car_model`、`styx_msgs`：与 Part 1 相同，但 `waypoint_updater` 形成了多节点联动（`rear_pose` → KDTree → `final_waypoints`）。

## 13_path_following_part_3_pure_persuit
- `pure_persuit/scripts/pure_persuit.py`：已经提供的路径跟踪算法。它订阅 `/smart/rear_pose`、`/smart/velocity` 与 `/final_waypoints`，使用纯追踪（Pure Pursuit）逻辑计算转角（`theta`）并发布 `Twist` 到 `/smart/cmd_vel`。关键参数 `HORIZON=6.0` 控制前视距离，车辆轴距常数写死为 `1.868m`。
- `waypoint_updater`、`waypoint_loader`、`car_model`、`styx_msgs`：延续 Part 2，组成完整的多进程 ROS 流水线（路径加载 → 局部航点 → 控制器 → Gazebo）。

## 现有能力与后续可扩展点
- 车辆建模与 Gazebo 集成（04~08）已提供从基础 URDF 到 Ackermann 控制翻译的完整链路，可直接复用 `car_model` 包搭建仿真环境。
- 状态采集（09~10）通过 `vehicle_pose_and_velocity_updater.py` 计算中心/后轴姿态，是实现后续 MPC/Stanley/RL 算法的唯一真实状态入口。
- 路径规划/跟踪模块（11~13）已经具备航点加载、最近点搜索以及纯追踪控制器，方便在此基础上实现新的控制算法：
  - 可在 `pure_persuit` 同级目录下新增 `stanley_controller`、`mpc_controller` 等包，输入输出接口与 `pure_persuit.py` 对齐即可接入现成下游节点。
  - `waypoint_loader/waypoints/*.csv` 当前仅包含简单直线，可扩展为圆形或曲率变化曲线，满足题目对不同路径的需求。
- 如需进行不同速度/不同曲线的性能对比，可复用 `waypoint_loader` 的 `~velocity` 参数与 CSV 数据源，外加 RViz (`10_show_car_in_rviz/rviz_config/smart.rviz`) 观察路径误差。

> 现状：仓库仅实现了 Pure Pursuit 一种路径跟踪算法。为满足“至少两种算法 + 多速度/曲线对比”的目标，可基于 `13_path_following_part_3_pure_persuit` 搭建新的 ROS 节点或 C++ 可执行文件，再复用 `car_model`/`waypoint_updater` 提供的接口迅速验证控制效果。
