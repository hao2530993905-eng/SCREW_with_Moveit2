# SCREW07091444

这是一个分层验证中的自动螺丝装配项目，目标硬件为 UR3 六轴机械臂、Damiao DM4310 电批和相机；LeRobot 用于记录示范数据及后续策略学习。

当前版本最重要的事实是：项目存在两条不同的执行路径，电批只在其中一条路径中被自动启动。

| 入口 | 机械臂 | 电批 | 用途 |
|---|---|---|---|
| `apps/screw_demo/main.py` | `moveJ` 接近、键盘 XY 微调、工具 Z 方向 `moveL` 预插 | 不控制，不调用 `set_speed`/`forward`/`reverse` | 验证 TCP、坐标方向、预插深度和 TCP 力数据 |
| `lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py` | 与基础 Demo 相同的接近、对准和插入动作 | 按 `P` 后设置目标转速，达到扭矩阈值时同步停止 `moveL` 和电批 | 采集带电批状态的 LeRobotDataset |
| `record_camera_insert_dataset.py` / `record_camera_insert_dataset_parallel.py` | 相机生成或传入接近位姿，随后复用记录入口 | 间接复用上一行的电批逻辑 | 相机引导的数据采集与启动耗时测试 |

因此，基础 Demo 的“预插成功”不代表螺丝已经旋入或拧紧；当前仓库还没有螺丝啮合检测、拧紧扭矩闭环或拧紧完成判定。

## 项目结构

```text
SCREW07091444/
├── apps/
│   ├── screw_demo/
│   │   ├── main.py              # 基础人工对准 + 工具轴预插 Demo
│   │   ├── ur_driver.py         # UR3 RTDE 状态读取和运动封装
│   │   ├── screw_client.py      # 电批 JSON/TCP Socket 客户端
│   │   └── tests/               # UR 和电批客户端分层测试
│   ├── screw_driver/
│   │   ├── src/screw_server.cpp # 电批 TCP JSON 服务端
│   │   ├── src/screw_driver.cpp # 控制状态、超时保护和 OpenArm/CAN 后端
│   │   └── include/              # 配置及状态定义
│   └── camera/                   # 相机采集、圆环检测和相机服务
├── lerobot_robot_screw/
│   ├── src/lerobot_robot_screw/  # LeRobot robot wrapper、状态和动作特征
│   └── scripts/recording/        # 人工、相机及并行数据采集入口
├── artifacts/                    # 数据集、日志、相机中间结果和模型
├── archive/legacy/               # 历史脚本，仅用于追溯
└── third_party/                  # openarm_can 等第三方代码
```

## 当前电批控制逻辑

### 1. 物理控制链路

真实电批控制链路如下：

```text
LeRobot/采集脚本
    -> ScrewClient
    -> TCP Socket 127.0.0.1:5055（每条 JSON 一行）
    -> screw_server
    -> ScrewDriverBackend
    -> OpenArm CAN
    -> DM4310 电机
```

`apps/screw_driver/src/screw_server.cpp` 是一个顺序处理客户端连接的 TCP 服务端。它按换行拆分请求，并返回一行 JSON；响应包含 `ok`、`message` 和当前 `status`。

支持的命令为：

| 命令 | 参数 | 实际作用 |
|---|---|---|
| `status` | 无 | 返回目标/实测转速、位置、速度、扭矩、温度、故障和 tick |
| `set_speed` | `speed`，rpm，可正可负 | 正值正转，负值反转，0 为目标停转；速度会被限幅 |
| `forward` | `speed`，rpm | 强制取正值后调用 `set_speed` |
| `reverse` | `speed`，rpm | 强制取负值后调用 `set_speed` |
| `heartbeat` | 无 | 刷新运动命令计时，维持当前目标转速 |
| `hold` / `stop` | 无 | 将目标转速设为 0，模式设为 `HOLDING` |
| `clear_fault` | 无 | 非急停时清除软件故障标志 |
| `estop` | `active`，默认 true | 设置软件急停状态并将目标转速清零 |

### 2. C++ 后端状态机和保护

`ScrewDriverBackend` 启动后创建一个 500 Hz 控制线程，控制周期为 2 ms。默认配置如下：

```text
CAN 接口       can0
CAN-FD         开启
电机类型       1（DM4310）
发送 ID        0x06
接收 ID        0x16
最大绝对转速   1500 rpm
扭矩停止阈值   0（关闭）
命令超时       0.5 s
转速换算       1 rpm = 0.104719755 rad/s
```

使用 `--openarm` 时，服务启动阶段会：

1. 检查 CAN 接口存在且处于 UP 状态。
2. 创建 OpenArm CAN 对象，初始化一个 DM4310 电机及发送/接收 CAN ID。
3. 将回调模式设为 `STATE`，控制模式切换为 `VEL`（速度模式）。
4. 接收状态、使能电机，然后启动控制线程。

控制线程在正常状态下反复向电机发送目标角速度并读取反馈，更新 `measured_speed_rpm`、位置、速度、扭矩和温度。目标转速由 `--max-rpm` 限幅，默认最大值为 ±1500 rpm。

命令接受条件是“没有软件急停且没有软件故障”。当进入急停或故障状态时，状态反馈中的实测速度、角速度、电流和扭矩被置零；服务停止时还会尝试发送零速度、等待反馈并 disable 电机。

命令超时保护只在目标转速非零时生效：如果超过 `--command-timeout` 没有新的运动命令或 `heartbeat`，后端会把目标转速清零并进入 `HOLDING`，故障信息写为 `motion command timeout, auto hold`。因此持续旋转必须持续发送心跳。记录脚本的独立心跳线程默认每 0.1 s 发送一次；记录器采样线程在目标速度非零时也会刷新一次心跳。

底层 `--torque-stop` 大于 0 时，控制线程会在反馈扭矩绝对值达到阈值后把目标转速清零并进入 `HOLDING`。记录流程使用更上层的 `--torque-stop-nm`，由采集脚本同时停止机械臂 `moveL` 和电批 `hold()`；底层阈值可作为额外的电批侧保护，但不负责停止机械臂。

未使用 `--openarm` 时，C++ 后端不连接 CAN，而是在内存中用一阶响应模拟实测转速、速度、电流和扭矩。`screw_client.py --dry-run` 则完全不连接 C++ 服务，使用 Python 内存对象模拟响应。

### 3. LeRobot 采集入口如何启动电批

`record_main_insert_dataset.py` 的电批阶段是：

```text
连接 UR 和 ScrewClient
    -> moveJ 到孔前接近位姿
    -> 人工在 TCP 工具坐标系 XY 平面微调
    -> Enter 接受对准结果
    -> 按 P
       -> recorder phase = INSERT
       -> 发送 set_speed(screw_speed_rpm)
       -> 启动 100 ms 周期 heartbeat
       -> 沿工具 Z 轴向最大安全行程执行 moveL
       -> 轮询电批 status.torque_nm
    -> abs(torque_nm) >= torque_stop_nm
       -> stopL 停止机械臂
       -> 发送 hold() 停止电批
    -> 未达到扭矩但到达最大安全行程/异常
       -> stopL + hold()
       -> 可选地继续记录 post-insert-record-s 秒
       -> 保存数据集并再次 hold/close
```

这里的正常完成条件已经改为“电批反馈扭矩达到阈值”。最大插入行程只是安全上限，不是正常拧紧完成条件。触发阈值后会同时停止机械臂和电批。`--screw-speed-rpm` 默认 60 rpm；`--torque-stop-nm` 必须在真实机器人模式下显式设置为正值。该阈值应由现场空载、装配和扭矩标定确定，不能直接照抄示例值。

数据记录中的动作是 7 维：6 维 TCP 位姿增量加 1 维 `screw_speed_rpm`。电批观测状态是 7 维：目标转速、实测转速、位置、角速度、电流、扭矩和故障标志。`mode`、温度、急停和故障文本会保存在原始 JSONL 的完整 `screw_status` 中。

### 4. LeRobot wrapper 的动作逻辑

`lerobot_robot_screw/src/lerobot_robot_screw/screw_robot.py` 把动作定义为 7 维。前 6 维是 TCP 增量，最后 1 维是电批目标转速：

1. 平移增量限制在 `max_translation_step_m`，旋转增量限制在 `max_rotation_step_rad`。
2. 平移目标还会限制在孔原点周围 `max_align_offset_m` 范围内。
3. 如果 TCP 增量非零，且允许 RTDE 控制，则调用 UR driver 的 `send_action`、`move_l` 或 `servo_l`。
4. 无论 TCP 是否移动，只要存在电批客户端，就调用 `set_speed(applied_screw_speed)`。
5. 电批速度限制在 `max_screw_speed_rpm`，LeRobot 配置默认是 ±300 rpm。
6. `disconnect()` 会先调用 `hold()`，再关闭电批客户端。

该 wrapper 的 `heartbeat_screw_driver()` 只是显式刷新心跳；它不会自动创建独立心跳线程。真正的采集脚本会在旋转期间启动 `ScrewHeartbeat`。

### 5. 连接方式

当前不再依赖机械臂/电批后台常驻连接。每次采集流程通过统一工厂创建 RTDE 或 MoveIt 规划加 RTDE 执行的机械臂后端，并创建 ScrewClient。MoveIt 模式只替换孔前接近：MoveIt 生成碰撞检查后的关节轨迹，原 URDriver 通过 RTDE 执行；人工对孔和旋入仍使用原 `moveL/servoL/stopL`。相机并行入口使用 `--start-moveit-stack` 时会自动启动并回收纯规划 MoveIt、RViz、电批碰撞体和保存的障碍物，不启动 `ur_robot_driver`、ros2_control 或 MoveIt Servo。

## 运行前准备

真实设备运行前必须确认：UR3 已上电并允许 RTDE 控制；TCP 已正确标定到批头尖端；工具 Z 方向、最大安全行程、扭矩阈值和工作空间都已单独验证；CAN、DM4310 驱动板及电批服务可用；急停可用且周围无人。

先进行 dry-run、状态读取和空载低速测试。第一次真实拧紧必须使用较低转速、较低插入速度、较小最大安全行程和经过标定的较低扭矩阈值。不要把 `actual_tcp_pose` 直接当作批头尖端位姿，也不要把到达最大安全行程当作拧紧完成。

## 常用命令

### 基础 Demo（不会启动电批）

基础 Demo 的真实 UR 单位是米和弧度：

```bash
python apps/screw_demo/main.py --help
python apps/screw_demo/main.py --enable-robot --robot-host 192.168.1.5 \
  --approach-pose X Y Z RX RY RZ \
  --insert-sign 1 --insert-depth 0.002 --insert-speed 0.003
```

### 构建并启动电批服务

以下命令在 Ubuntu 控制机执行，具体 SocketCAN 配置以现场硬件为准：

```bash
cmake -S apps/screw_driver -B apps/screw_driver/build -DUSE_OPENARM_CAN=ON
cmake --build apps/screw_driver/build
sudo third_party/openarm_can-1.1.0/setup/configure_socketcan.sh can0 -fd
apps/screw_driver/build/screw_server --openarm --can can0 \
  --send-id 0x06 --recv-id 0x16 --motor-type 1 --port 5055 \
  --feedback-timeout 0.1
```

如果硬件只支持 classic CAN，启动服务时使用 `--no-can-fd`。需要启用扭矩停止时显式增加 `--torque-stop <Nm>`；需要调整看门狗时使用 `--command-timeout <秒>`。

### 带电批的 LeRobot 采集

该入口的 XYZ、最大插入安全行程和插入速度使用 mm、mm/s，脚本内部会在进入 UR driver 前转换成 m、m/s。真实机器人模式必须提供正的 `--torque-stop-nm`：

```bash
python lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py \
  --enable-robot --robot-host 192.168.1.5 \
  --approach-pose X_MM Y_MM Z_MM RX RY RZ \
  --max-insert-depth 20 --insert-speed 3 --insert-sign 1 \
  --screw-host 127.0.0.1 --screw-port 5055 --screw-speed-rpm 60 \
  --torque-stop-nm TORQUE_THRESHOLD_NM \
  --torque-filter-window 5 --torque-confirm-samples 3 --torque-min-active-s 0.2 \
  --dataset-root artifacts/datasets/real_insert_dataset \
  --repo-id local/real_screw_insert_001 --fps 20
```

相机入口会负责生成接近位姿，之后把参数传给同一条记录流程；并行入口会在当前进程内直接连接 UR 和电批。相机自动定位必须配合有效的相机到机器人标定文件，默认接近位姿只适合流程验证。

### 相机并行入口 + MoveIt 一键流程

首先按照 `ros2/screw_moveit_integration/README.md` 提取当前实体 UR3 的标定
参数，并确保机械臂允许原项目通过 `ur_rtde` 发送脚本。这个模式不运行
`ur_robot_driver`，也不需要播放 External Control 节点。然后从已经 source
ROS 2 工作空间的终端运行：

```bash
cd ~/桌面/ROS2/moveit/SCREW_with_power_control
source /opt/ros/humble/setup.bash
source ~/桌面/ROS2/moveit/install/setup.bash

python3 lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py \
  --start-moveit-stack \
  --moveit-kinematics-params-file \
    $HOME/桌面/ROS2/moveit/config/ur3_calibration.yaml \
  --camera-ip 192.168.1.66 \
  -- \
  --enable-robot \
  --robot-host 192.168.1.5 \
  --max-insert-depth 20 \
  --insert-speed 3 \
  --insert-sign 1 \
  --screw-host 127.0.0.1 \
  --screw-port 5055 \
  --screw-speed-rpm 60 \
  --torque-stop-nm TORQUE_THRESHOLD_NM
```

`--start-moveit-stack` 会自动向内层记录流程注入
`--motion-backend moveit`。每次实验的 MoveIt 输出单独保存在实验目录的
`moveit_launch.log`。如果 MoveIt 已由另一个终端启动，则省略
`--start-moveit-stack`，并在 `--` 后显式添加
`--motion-backend moveit`。

## 键盘交互

| 按键 | 作用 |
|---|---|
| `Enter` | 确认安全检查或接受人工对准结果 |
| 方向键 | 在 TCP 工具坐标系 XY 平面内执行受限 `moveL` 微调 |
| `P` | 在带电批记录流程中设置转速、启动心跳并开始扭矩控制插入；达到扭矩阈值时同时停机械臂和电批 |
| `Q` | 中止流程；异常时还应准备使用机器人控制柜急停 |

## 当前限制

- 基础 Demo 不接入视觉目标位姿，也不控制 DM4310 旋转。
- 电批仍是速度模式；当前上层流程使用扭矩阈值作为停止条件，但没有扭矩误差调节、力控、堵转检测或批头/螺丝啮合检测。
- `--torque-stop-nm` 达到阈值时会同时停止当前 `moveL` 和电批；到达最大安全行程仍未达到阈值会报错，不会判定为拧紧成功。
- 扭矩判定默认使用 5 点中值滤波、连续 3 次确认和 0.2 秒启动屏蔽，避免单点尖峰误判。
- OpenArm 后端会记录有效 CAN 反馈计数和反馈年龄；超过 `--feedback-timeout` 未收到有效电机状态帧会进入故障并停止接受运动命令。
- 电批服务的 `estop` 是软件状态和控制逻辑中的急停，不应替代机器人控制柜或硬件急停。
- 生产性安全策略、错误恢复和真实工件参数仍需现场验证。
- LeRobot 数据中的相机图像和状态质量依赖时间同步、TCP 标定、力传感器标定及人工操作。

后续闭环建议按以下顺序推进：CAN/电批独立测试 → 批头和螺丝啮合检测 → 预插接触检测与力控 → 相机标定和自动对孔 → 扭矩/电流拧紧判定 → LeRobot 多模态策略训练与部署。

更完整的环境、测试和实验说明见 [项目说明.md](项目说明.md)。
