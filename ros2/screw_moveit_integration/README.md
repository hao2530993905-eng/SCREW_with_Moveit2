# UR3 MoveIt planning with original RTDE insertion

这个包只把“孔前接近”接入 MoveIt。人工对孔和旋入阶段仍完整使用项目原来的
`URDriver`、`moveL/servoL/stopL` 和电批扭矩控制。

控制边界如下：

```text
孔前接近:
MoveIt 碰撞规划 -> plan_only 关节轨迹 -> RTDE servoJ 跟踪

人工对孔:
原 URDriver.moveL

旋入:
原动态 servoL + 电批转速/扭矩反馈
-> 达到阈值
-> 原 stopL -> hold() 同步停止
```

纯规划 launch 不启动 `ur_robot_driver`、ros2_control 或 MoveIt Servo，因此不会
和 `ur_rtde` 争用实体 UR 控制权。

## 构建

```bash
cd ~/桌面/ROS2/moveit
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select ur3_workcell_scene screw_moveit_integration
source install/setup.bash
```

## UR3 标定

MoveIt 规划必须使用当前实体 UR3 的标定文件：

```bash
mkdir -p ~/桌面/ROS2/moveit/config
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.5 \
  target_filename:=$HOME/桌面/ROS2/moveit/config/ur3_calibration.yaml
```

这里不需要运行示教器 External Control 节点。实体运动仍由原项目的
`ur_rtde` 控制，机械臂需处于允许远程脚本控制的模式。

## TCP 目标语义

项目中的笛卡尔目标统一表示示教器当前激活 TCP 的目标位姿，不表示
`tool0` 或法兰原点的目标位姿。

MoveIt 后端在每次 IK 前通过 `RTDEReceiveInterface.getTCPOffset()` 从实体 UR
实时读取当前活动 TCP 相对输出法兰的六维变换，然后计算：

```text
T_base_tool0 = T_base_tcp * inverse(T_tool0_tcp)
```

转换后的 `tool0` 目标只在 MoveIt 内部用于 IK；对外输入的
`--approach-pose`、RTDE `getActualTCPPose()`、`moveL` 和 `servoL`
始终使用同一个活动 TCP 语义。

代码不包含固定的工具长度，也不会调用 `setTcp()` 覆盖示教器设置。若在 IK
完成后、关节运动开始前修改活动 TCP，程序会拒绝该次运动，必须使用新 TCP
重新求解 IK。

活动 TCP 只定义“哪个工具点到达目标”。电批的实体碰撞几何仍由
`ur3_workcell_scene` 中的附着碰撞体定义；修改 TCP 或更换工具后，还必须确认
碰撞模型的尺寸、安装方向与实体一致。

## 一键整体流程

先启动原电批服务，然后运行：

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

`--start-moveit-stack` 自动启动：

- MoveGroup 纯规划节点；
- `robot_state_publisher`；
- RViz；
- 电批附着碰撞体；
- 保存的 `Box_0` 到 `Box_4` 障碍物。

它还会向内层记录流程注入 `--motion-backend moveit`，并在流程退出时回收这些
ROS 节点。MoveIt 日志保存在本次实验目录的 `moveit_launch.log`。

## 分开启动

终端 1：

```bash
ros2 launch screw_moveit_integration moveit_rtde_planning.launch.py \
  ur_type:=ur3 \
  kinematics_params_file:=$HOME/桌面/ROS2/moveit/config/ur3_calibration.yaml \
  start_rviz:=true
```

终端 2：

```bash
python3 lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py \
  --camera-ip 192.168.1.66 \
  -- \
  --enable-robot \
  --motion-backend moveit \
  --robot-host 192.168.1.5
```

不要同时运行旧的 `real_screw_moveit.launch.py`。那个 launch 会启动
`ur_robot_driver` 和 ros2_control，适用于纯 ROS 控制验证，不适用于当前要求的
原始 RTDE `moveL/servoL` 插入流程。
