# xnero_mujoco_sim

xnero（Nero 双臂 + XHand 双手）的 **MuJoCo 硬件驱动替身**：让 Quest3 遥操链路
**脱离真机**跑通，并把 MuJoCo 里真实的**逐点触觉力**注入 `xhand_state` —— 于是
`nero_dual_data_collect` 一行不改就能录到含触觉的仿真数据。

参考实现：`astral_ws/src/astral_mujoco_sim`（astral 的同类包；差别见文末「与 astral 的差异」）。

## 1. 它替掉了什么

| 真机环节 | 谁来做 | 怎么接 |
|---|---|---|
| XHand USB 串口驱动（`xhand_control_ros2`，C++） | **本包 sim 节点** | 订 `/{side}_hand/xhand_command`，发 `/{side}_hand/xhand_state`（同契约） |
| Nero 臂 CAN 下发（`pyAgxArm`） | **teleop 的 sim_mode**（`sim_arm_stub.py`） | 不连 CAN，命令发 `~/joint_commands` |

**真机路径零改动**：`sim_mode` 默认 `false`，`xhand_control_ros2` 与 `pyAgxArm` 一行未动。
teleop 的 diff 只有「新增一个参数 + 把 `AgxArmFactory.create_arm(cfg)` 包进 if/else」。

## 2. 物理模型：手物理 + 臂运动学

- **手**：真物理。MJCF 里 24 个 `<position>` 执行器 + `mj_step`，接触真实、触觉力真实。
- **臂**：运动学覆盖。每步把 14 个臂关节 `qpos` 写成命令值、`qvel` 清零，语义是
  **「无限刚 Kinematic 体」——能推动物体，自己不被推动**（这正是采集操作数据想要的）。
- 触觉：指尖 120 个探针球（沿 `docs/xhand1-tactile-sim-params.md` 的标定），
  指尖还有 10 个 `touch` 传感器给「单指合力」。

## 3. 数据流

```
Quest3 ─→ quest3_udp_mocap ─┬→ xhand_dex_retargeting ─→ /{L,R}_hand/xhand_command ─┐
                            │                                                      │
                            └→ nero_teleop_{L,R} (sim_mode:=true)                 │
                                  └→ /nero_teleop_{L,R}/joint_commands ───────────┤
                                                                                  ▼
                                                     xnero_mujoco_sim_node (viewer)
                                                       ├ 手：位置执行器 → mj_step
                                                       ├ 臂：每步写 qpos+qvel=0
                                                       ├ 发 /{L,R}_hand/xhand_state
                                                       │    （sensor_states = 逐点触觉力）
                                                       └ 发 /nero_teleop_{L,R}/joint_states
                                                                                  ▼
                                                     nero_dual_data_collect（零改动）
```

## 4. 构建

> ⚠ **必须清掉 conda 的 PATH**。否则 CMake 会抓到 `~/miniconda3/bin/python3`（3.13）
> 去跑 ROS 的 `rosidl_adapter`，构建必失败；且失败的 `CMakeCache.txt` 会残留，
> 之后不清 `build/` 就一直失败（踩过）。

```bash
cd /home/robot/loopkok/sdk/xnero_ws
rm -rf build install log                      # 只在遇到残留状态时
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin
          source /opt/ros/humble/setup.bash
          cd /home/robot/loopkok/sdk/xnero_ws && colcon build --symlink-install'
```

依赖：
- **mujoco**（pip，装到 `/usr/bin/python3` —— 它同时有 rclpy）：`/usr/bin/python3 -m pip install --user mujoco`
- **torch**（`xhand_dex_retargeting_node` 经 `dex_retargeting` 需要）：
  `/usr/bin/python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cpu`

MJCF 改了**不用重建**：`--symlink-install` 下 `install/.../xnero_dual.xml` 一路符号链接回
源码树（实测过）。

## 5. 用法

```bash
source /opt/ros/humble/setup.bash && source /home/robot/loopkok/sdk/xnero_ws/install/setup.bash

# 只起仿真（自己往话题里发命令）
ros2 launch xnero_mujoco_sim xnero_mujoco_sim.launch.py

# 全链路（无真机）：quest3 → retargeting → teleop(sim) → sim
ros2 launch xnero_mujoco_sim xnero_sim_pipeline.launch.py
ros2 launch xnero_mujoco_sim xnero_sim_pipeline.launch.py \
    protocol:=udp enable_viewer:=false publish_tactile:=false
```

没有 Quest3 时，用键盘模拟器喂 VR 数据：
`ros2 run nero_quest_teleop keyboard_vr_sim --ros-args -p arm_side:=left`

**重新生成 MJCF**（改了 URDF 之后）：

```bash
cd src/xhand_nero_description
python3 scripts/make_mjcf.py                       # → assets/mjcf/xnero_dual.xml
python3 scripts/make_mjcf.py --kp 8 --force 0.8    # 手部增益/力矩上限
```

## 6. 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订 | `/{left,right}_hand/xhand_command` | `XHandCommand` | 12 关节位控（mode=3）；按 `name` 匹配，无名则按顺序 |
| 订 | `/nero_teleop_{left,right}/joint_commands` | `JointState` | 臂命令（teleop sim_mode 发）；两种命名都认 |
| 发 | `/{left,right}_hand/xhand_state` | `XHandStateArray` | `hand_states[0].position`（12）+ `sensor_states`（触觉） |
| 发 | `/nero_teleop_{left,right}/joint_states` | `JointState` | 臂实际 qpos（名字用真机的 `joint1..7`） |

## 7. 触觉数据怎么读

`xhand_state.sensor_states[0].finger_sensor_states[j]`（j 顺序 = `thumb, index, middle, ring, pinky`）：

| 字段 | 内容 |
|---|---|
| `location` | `"thumb"/"index"/"middle"/"ring"/"pinky"`（与真机驱动一致，中指是 **middle**） |
| `raw_force[120]` | **逐点三轴力**，`raw_force[k]` ↔ SDK 的 `raw_force[k]` ↔ 测点 `p{k+1:03d}` |
| `calc_force` | 该指 120 点的矢量和 |
| `raw_temperature[20]` / `calc_temperature` | 仿真里恒 0 |

**坐标系说明**：仿真给的 `raw_force[k]` 是**该指尖 link2 坐标系**下的三分量
（120 个点在同一坐标系里，可直接求和/看方向）。真机传感器给的是**传感器自身三轴**的
分量 —— 幅值可比，分量方向需按安装姿态换算。

**带宽**：2 手 × 600 点 × 3 float @50Hz ≈ 1.4 MB/s。嫌重就 `publish_tactile:=false`
（只发关节位置，采集链路照常工作）；或用 `tactile_rate` 单独降频。

## 8. 参数（`config/xnero_mujoco_sim.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `mjcf_path` | `""` | 空 = 用包内 `assets/mjcf/xnero_dual.xml` |
| `enable_viewer` | `true` | 开 MuJoCo 被动 viewer |
| `realtime` | `true` | 按 `timestep` 节流 |
| `state_rate` | `50.0` | `xhand_state` / 臂 `joint_states` 频率 |
| `hand_kp` / `hand_kv` | `5.0` / `0.1` | 手部执行器增益。**抓不稳就加大**（见下） |
| `hand_torque_limit` | `0.5` | 执行器力矩上限 N·m |
| `publish_tactile` | `true` | 是否在 `xhand_state` 里发触觉 |
| `tactile_rate` | `50.0` | 触觉发布频率（与 `state_rate` 独立） |
| `arm_command_timeout` | `0.5` | 臂命令超时（超时保持最后值，不归零） |

**手部增益为什么要单独给**：`XHandCommand` 里的 `kp=80 / ki=0 / kd=0` 是 XHand 的
**内部单位**，不能 1:1 映到 MuJoCo 的 `kp`。默认 5.0/0.1 能跟住空载命令；抓取打滑就
加大 `hand_kp` 与 `hand_torque_limit`，重跑 `make_mjcf.py --kp ... --force ...` 或改
`xnero_mujoco_sim.yaml`。

## 9. 测试

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/xnero_mujoco_sim
/usr/bin/python3 -m pytest test/ -q -p no:anyio     # 31 例
```

覆盖：手部命令打包/跟踪、meshdir 解析、MJCF 产物结构、**触觉逐点力与 touch 传感器一致**、
臂运动学覆盖精确性、`sim_mode` 防回归（真机路径未被替换）。

## 10. 已知限制

1. **臂是运动学体**：能推物体，自己不会被推动、不会因碰撞停下。这是选「臂运动学」的
   必然结果；需要臂也参与接触就把它改成物理驱动（加执行器 + 不再每步覆盖 qpos）。
2. **指尖末端的触觉点会被指尖 mesh 遮挡**：`*_rota_link2` body 里融着 fixed 子链
   （`*_rota_tip`、`*_rotaback_link2`）的碰撞 mesh（`fusestatic` 的结果），
   最靠近指尖的那 1~2 排测点（z≈40-42mm）压上去时可能先碰到它们。
   需要干净的整片 pad → 把 `make_mjcf.py` 里的 `PAD_LINK2_RE` 正则扩大到包含 tip。
3. **没有相机渲染**：图像仍来自真机/录好的数据。仿真出图是另一个量级的工作。
4. **关节命名两套并存**：URDF/MJCF 里是 `{side}_joint1..7`；`/nero_teleop_*/joint_states`
   沿用真机 teleop 的 `joint1..7`（消费端只看 `position[:7]`，无影响）。

## 11. 与 astral_mujoco_sim 的差异

| | astral | xnero |
|---|---|---|
| 物理 | 纯运动学（写 qpos + `mj_forward`，geom 全 `contype=0`） | **手物理 + 臂运动学**（`mj_step`） |
| 触觉 | 无 | **1200 点逐点接触力**注入 `xhand_state` |
| 臂命令来源 | teleop 本来就发 `/{side}_arm/joint_commands` | teleop 原本直连 CAN，**新增 `sim_mode`** 才发出命令 |
| MJCF 生成 | 仓库内手工维护的 MJCF | **`make_mjcf.py` 从 URDF 生成**（可重跑） |
