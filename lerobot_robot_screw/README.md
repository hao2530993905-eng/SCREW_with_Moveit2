# lerobot_robot_screw

本目录是 SCREW07091444 项目的 LeRobot 数据采集子项目。真实运行入口位于
`scripts/recording/`，数据输出统一写入项目根目录的 `artifacts/datasets/`。

真实机器人采集入口：

```text
lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py
lerobot_robot_screw/scripts/recording/record_camera_insert_dataset.py
lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py
```

LeRobot 第三方机器人包，按 LeRobot 0.5 的推荐自动发现前缀命名：

- distribution name: `lerobot_robot_screw`
- Python package: `lerobot_robot_screw`
- robot type: `screw_robot`

安装到 `lerobot` 环境后，LeRobot 会在启动时自动发现 `lerobot_robot_*` 包。也可以显式使用：

```bash
conda run -n lerobot python -m pip install -e ./lerobot_robot_screw

conda run -n lerobot python -m pytest lerobot_robot_screw/tests -v
```

生成一份本地 LeRobotDataset：

```bash
conda run -n lerobot python lerobot_robot_screw/scripts/tools/generate_lerobot_dataset.py \
  --root artifacts/datasets/screw_lerobot_dataset \
  --repo-id local/screw_robot_smoke \
  --frames 12 \
  --use-videos
```

随机抽样检查 state/action/image：

```bash
conda run -n lerobot python lerobot_robot_screw/scripts/tools/sample_lerobot_dataset.py \
  --root artifacts/datasets/screw_lerobot_dataset \
  --repo-id local/screw_robot_smoke
```

保存官方 visualizer 的 Rerun 文件：

```bash
conda run -n lerobot python /home/sjh/lerobot/src/lerobot/scripts/lerobot_dataset_viz.py \
  --repo-id local/screw_robot_smoke \
  --root artifacts/datasets/screw_lerobot_dataset \
  --episode-index 0 \
  --mode local \
  --save 1 \
  --output-dir artifacts/datasets/visualizer \
  --num-workers 0
```
