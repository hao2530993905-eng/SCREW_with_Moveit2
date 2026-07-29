# MoveIt 路径规划说明

## 1. ROS2 RViz 场景

ROS2 机器上的场景包是 `ur3_workcell_scene`。障碍物和工具不是持续发布的普通话题节点，而是一次性调用 `/apply_planning_scene`，把对象写入 MoveIt Planning Scene；节点退出后场景仍由 MoveIt 保留。

| 内容 | 文件 |
| --- | --- |
| 一键启动 UR3、MoveIt、RViz、场景节点 | `/home/liuhao/桌面/ROS2/moveit/src/ur3_workcell_scene/launch/saved_workcell_scene.launch.py` |
| 恢复障碍物节点 | `/home/liuhao/桌面/ROS2/moveit/src/ur3_workcell_scene/ur3_workcell_scene/restore_saved_obstacles.py` |
| 附着中间件、电批、批头节点 | `/home/liuhao/桌面/ROS2/moveit/src/ur3_workcell_scene/ur3_workcell_scene/attach_screwdriver.py` |
| 障碍物配置 | `/home/liuhao/桌面/ROS2/moveit/src/ur3_workcell_scene/config/saved_scene.yaml` |

ROS2 障碍物配置如下，单位为米，`position` 是盒体中心：

```yaml
Box_0: size [0.20, 0.20, 0.02], position [-0.04, -0.04, -0.01]
Box_1: size [1.00, 1.00, 0.60], position [ 0.30, -0.40, -0.31]
Box_2: size [0.30, 0.30, 0.08], position [ 0.00, -0.32,  0.00]
Box_3: size [0.03, 0.03, 1.00], position [ 0.47,  0.11,  0.48]
Box_4: size [0.30, 0.30, 0.08], position [ 0.37, -0.33,  0.00]
```

ROS2 工具模型由三个沿 `tool0` Z 轴排列的圆柱组成：

```text
中间件: 长 0.046 m，半径 0.0369 m
电批本体: 长 0.045 m，半径 0.0238 m
批头: 长 0.125 m，半径 0.0130 m
总伸出长度: 0.216 m
```

历史差异：ROS2 原始 `attach_screwdriver.py` 的 `touch_links` 包含 `tool0`、`flange`、`wrist_3_link`，因此允许工具与法兰、腕 3 接触。ROS1 当前版本已经修正为只允许抽象坐标系 `tool0`，禁止中间件、电批和批头与所有真实机械臂链接接触。

### 纯 UR3 末端是否包含 flange？

包含。物理裸机械臂链条是：

```text
... -> wrist_3_link -> flange -> tool0
```

- `flange`：真实存在的法兰盘，属于裸机械臂，应参与碰撞检测。
- `tool0`：法兰前方定义的坐标系，通常没有实体碰撞几何。
- 中间件、电批、批头：附着在 `tool0` 上的外部工具。
- 示教器 TCP：由示教器标定的工作点，不等于 `tool0`。

## 2. ROS1 远程机的 MoveIt 集成

ROS1 项目根目录：

```text
C:\Users\刘浩\Desktop\SCREW\SCREW_with_Moveit2
/home/sjh/SCREW_with_Moveit2
```

| 用途 | 文件 |
| --- | --- |
| ROS1 总启动文件 | `ros1_ws/src/screw_moveit_integration/launch/moveit_rtde_planning.launch` |
| 障碍物与完整工具碰撞模型 | `ros1_ws/src/screw_moveit_integration/scripts/apply_workcell_scene.py` |
| MoveIt JSON/TCP 桥接服务 | `ros1_ws/src/screw_moveit_integration/scripts/moveit_bridge_server.py` |
| MoveIt 轨迹转 RTDE `servoJ` | `apps/screw_demo/moveit_ros1_rtde_driver.py` |
| 共享关节限位配置 | `ros1_ws/src/screw_moveit_integration/config/ur3_joint_limits.yaml` |
| RViz 目标/TCP 标记配置 | `ros1_ws/src/screw_moveit_integration/config/moveit_target.rviz` |
| 手动发布 TCP/目标标记工具 | `ros1_ws/src/screw_moveit_integration/scripts/publish_tcp_markers.py` |

ROS1 当前障碍物与工具配置在 `apply_workcell_scene.py` 中。相比 ROS2，`Box_0` 和 `Box_1` 下移了 1 cm，避免固定底座被误判为与工作台相撞：

```text
Box_0: [0.20, 0.20, 0.02], [-0.04, -0.04, -0.02]
Box_1: [1.00, 1.00, 0.60], [ 0.30, -0.40, -0.32]
Box_2: [0.30, 0.30, 0.08], [ 0.00, -0.32,  0.00]
Box_3: [0.03, 0.03, 1.00], [ 0.47,  0.11,  0.48]
Box_4: [0.30, 0.30, 0.08], [ 0.37, -0.33,  0.00]

完整工具 collision object: screwdriver_tool
附着链接: tool0
允许接触链接: 仅 tool0
```

因此当前规划检查：

```text
UR3 自碰撞
UR3 <-> Box_0 ... Box_4
中间件/电批/批头 <-> Box_0 ... Box_4
中间件/电批/批头 <-> flange、wrist_3、前臂、上臂等所有真实机器人链接
```

当前工具碰撞圆柱总长是 `0.216 m`，而示教器 TCP 偏移约为 `0.247 m`。若实际批头尖端达到 TCP，必须把批头碰撞体长度补到实测尺寸。

## 3. 完整控制流程

```text
启动相机后端服务
启动电批服务（按需要）
  -> 运行 parallel 主脚本
  -> 启动 ROS1 MoveIt、RViz
  -> 加载 UR3 标定、共享限位、障碍物、完整工具碰撞体
  -> 相机采集 与 机械臂/电批连接并行
  -> 点云圆心、平面法向、手眼变换
  -> 生成示教器 TCP 目标位姿
  -> RViz 显示目标 TCP 与当前 TCP
  -> MoveIt IK 与无碰撞路径规划
  -> RTDE servoJ 执行规划关节轨迹
  -> 原 moveL 插入、电批转动、力矩控制
  -> 停止、保存数据、清理
```

### MoveIt 相关细节

1. `--start-moveit-stack` 启动 `moveit_rtde_planning.launch`。
2. 启动文件用实体 UR3 标定 YAML 生成 `robot_description`；标定保证 RViz 连杆几何与实体机一致。
3. `ur3_joint_limits.yaml` 是唯一关节限位来源：Xacro/MoveIt 与 RTDE 驱动都读取它。示教器 `Safety -> Joint Limits` 的六轴上下限应填入此文件。
4. `apply_workcell_scene.py` 调用 `/apply_planning_scene`，写入五个障碍物与 `screwdriver_tool`。
5. RTDE 持续读取真实关节角与当前示教器 TCP，并通过本机 `127.0.0.1:5061` 发送给 `moveit_bridge_server.py`。
6. 相机返回圆心和外圈法向；脚本通过手眼变换转换到机器人 `base`，再生成 `[x, y, z, rx, ry, rz]` 的 TCP 目标。
7. MoveIt 的 IK 目标是 `tool0`。驱动先读取当前示教器 TCP 偏移，再计算：

   ```text
   tool0_target = tcp_target * inverse(active_tcp_offset)
   ```

   因而最终到达目标的是示教器标定的 TCP 点，而不是法兰中心。
8. IK 使用多种种子：上次成功解、当前关节角及备用姿态，降低 KDL 对单一初始种子敏感而失败的概率。
9. 多圈 RTDE 关节读数会规范化为 MoveIt 可理解的等价角；执行时只选择满足共享关节上下限的等价角，避免 MoveIt 与示教器一边允许、一边拒绝。
10. 规划器按顺序尝试：
    - `RRTConnect`：请求参数默认 `5 秒 / 10 次`
    - `RRTstar`：至少 `12 秒 / 30 次`
    - `PRM`：至少 `12 秒 / 30 次`
11. 只有得到完整、无碰撞的 `JointTrajectory` 才执行；近似解不执行。
12. MoveIt 只负责规划；`moveit_ros1_rtde_driver.py` 将轨迹映射为满足实际圈数和限位的关节值，再通过 RTDE `servoJ` 执行。
13. 抵达接近位姿后，MoveIt 结束职责。原有插入逻辑保持不变：按当前 TCP 局部 Z 轴计算插入方向，采用原 `moveL/servoL` 前进，电批同步旋转，以力矩阈值、滤波、连续样本确认、降速与爬行策略结束。

## 4. record_camera_insert_dataset_parallel.py 的运行逻辑

文件：`lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py`

1. 解析主参数：相机、圆环选择、相机服务、标定、`--start-moveit-stack`、`--skip-screwdriver`、日志目录；`--` 后参数交给 `record_main_insert_dataset.py`。
2. 创建实验目录与 `run.log`，将终端输出同步写入日志。
3. 使用 `--start-moveit-stack` 时，强制下游使用 `--motion-backend moveit`，并启动 ROS1 `moveit_rtde_planning.launch`。
4. 等待桥接服务、五个障碍物和 `screwdriver_tool` 就绪。
5. 使用两个并行任务：
   - 相机任务：请求相机常驻服务 `http://127.0.0.1:5060/capture`，或执行旧相机脚本。
   - 设备任务：导入 `record_main_insert_dataset.py`、预加载 LeRobotDataset、连接机器人；正常模式下连接并预检电批。
6. 从相机结果选择 `inner`、`outer` 或 midpoint 圆心；结合平面法向、基础姿态和手眼标定生成接近 TCP 位姿。
7. 若机器人后端支持，立刻发布目标 TCP Marker 到 RViz。
8. 移除旧 `--approach-pose`，将新的相机接近位姿注入下游参数。
9. 复用已连接的机器人与电批对象，调用 `record_main_insert_dataset.run(...)`，避免重复连接。
10. 下游先执行 `MOVE_APPROACH_MOVEJ`：
    - `motion-backend=moveit`：MoveIt IK、无碰撞路径、RTDE `servoJ`。
    - `motion-backend=rtde`：旧的直接 RTDE moveJ。
11. 第二次人工确认接近孔位后：
    - `--skip-screwdriver`：安全结束，不运行 moveL 与电批。
    - 正常模式：开始数据记录、手动微调、原始 moveL 插入、电批转动与力矩控制。
12. 无论成功、失败或中止，脚本都会停止线性运动、电批、心跳线程和 MoveIt 子进程，并输出阶段耗时统计。
