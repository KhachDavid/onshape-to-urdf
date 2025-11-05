# Minimal setup and usage

## 1) Requirements
- Python 3.10+
- Onshape API keys
- (Optional) ROS 2 for visualization (rviz2, robot_state_publisher, joint_state_publisher_gui)

## 2) Install
```bash
# from repo root
bash setup.sh
```

## 3) Configure Onshape
Edit `config.json` and set:
- `documentId`: your Onshape document UUID
- `workspaceId` (or `versionId`): your workspace UUID
- `elementId`: the assembly tab UUID

Set API keys (terminal):
```bash
export ONSHAPE_ACCESS_KEY=your_key
export ONSHAPE_SECRET_KEY=your_secret
```

.env (copied from the template using setup.sh script)
```
ONSHAPE_ACCESS_KEY=your_key
ONSHAPE_SECRET_KEY=your_secret
```

## 4) Generate URDF
```bash
bash run_conversion.sh
```
Outputs are written to `output/<robot>_<timestamp>/`:
- `robot.urdf` (original, package:// URIs)
- `robot_standalone.urdf` (file:// absolute mesh paths)
- `assets/` (meshes)

## 5) (Optional) Visualize in ROS 2
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat output/<robot>_<timestamp>/robot_standalone.urdf)" &
rviz2
```

## 6) Quick RViz demo script
From the repo root:
```bash
bash rviz_demo.sh                 # uses newest directory under output/
bash rviz_demo.sh output/<robot>_<timestamp>  # or specify a directory
```
The script will:
- start `joint_state_publisher_gui` (or fallback to `joint_state_publisher`),
- start `robot_state_publisher` with the chosen URDF,
- launch RViz2 with the bundled config at `rviz/demo.rviz` (Grid, TF, RobotModel).

## Notes
- Customize by editing only `config.json` IDs.
- Use `robot_standalone.urdf` when loading in tools that expect absolute mesh paths.