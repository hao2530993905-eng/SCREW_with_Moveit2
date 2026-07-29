# ROS1 Noetic MoveIt 使用说明

本适配专门用于 `/home/sjh/SCREW_with_Moveit2`：

- `/usr/bin/python3`（Python 3.8）运行 ROS Noetic、MoveIt 和 RViz。
- `lerobot_eye`（Python 3.12）运行相机、数据采集、原有电批力控和 RTDE。
- 两个进程通过本机 `127.0.0.1:5061` 通信。
- 只有远距离 `moveJ` 改为 MoveIt 规划后通过 RTDE `servoJ` 执行。
- 原有 `moveL`、`servoL`、插孔和电批力控没有改为 MoveIt。
- 目标位姿始终指示示教器当前激活的 TCP。每次 IK 前通过
  `getTCPOffset()` 读取，不使用固定 24 cm。

## 首次编译

```bash
cd /home/sjh/SCREW_with_Moveit2
source /opt/ros/noetic/setup.bash
catkin_make -C ros1_ws
source ros1_ws/devel/setup.bash
```

## 一次性启动

先在同一个终端加载 ROS1 和项目工作空间，然后明确使用
`lerobot_eye` 的 Python：

```bash
cd /home/sjh/SCREW_with_Moveit2
source /opt/ros/noetic/setup.bash
source ros1_ws/devel/setup.bash

/home/sjh/anaconda3/envs/lerobot_eye/bin/python \
  lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py \
  --start-moveit-stack
```

原脚本所需的其他相机、机器人、数据集参数继续接在命令末尾。默认会自动
打开 RViz；加 `--no-moveit-rviz` 可关闭。

## 实体机几何标定

如已有实体 UR3 对应的 `kinematics.yaml`，请把它放在本项目目录内，例如：

```text
/home/sjh/SCREW_with_Moveit2/config/ur3_calibration.yaml
```

启动时追加：

```bash
--moveit-kinematics-params-file \
  /home/sjh/SCREW_with_Moveit2/config/ur3_calibration.yaml
```

不提供该参数时使用 ROS Noetic `ur_description` 自带的 UR3 标称几何参数。
这份几何标定和示教器 TCP 是两件事：前者修正机械臂各关节/连杆模型，后者
描述法兰到当前工具目标点的变换。

## 安全边界

MoveIt 只会对规划场景中已有的碰撞几何负责。启动文件会加载五个 Box 和
固定尺寸的电批碰撞模型；示教器 TCP 的变化只会更新目标点变换，不会自动
推断工具的真实外形。更换长短或外形明显不同的工具时，也应同步修改碰撞
模型。首次联调请使用低速、保护停止和现场监护。
