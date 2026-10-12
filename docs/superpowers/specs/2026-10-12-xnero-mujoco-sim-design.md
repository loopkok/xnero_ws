# xnero MuJoCo 仿真包 · 设计

日期：2026-10-12 ｜ 状态：已评审（设计已与用户逐节确认）

## 1. 目标

给 xnero（Nero 双臂 + XHand 双手）做一个 MuJoCo 仿真包，作为**硬件驱动替身**：
让「Quest3 遥操 → 重定向/IK → 驱动」整条链路**脱离真机**跑通，并且**产出可采集的
数据**——包括 MuJoCo 里真实的逐点触觉力。

参考实现：`astral_ws/src/astral_mujoco_sim`（astral 的同类包，但其为纯运动学替身）。

### 用户已确认的两个关键选择

| 决策 | 选择 | 含义 |
|---|---|---|
| 替掉哪个环节 | **手 + 臂都进仿真** | 手顶替 `xhand_control_ros2` 驱动；臂需给 teleop 加一条命令话题 |
| 物理程度 | **混合：手物理 + 臂运动学** | 手跑真物理、出接触力与触觉；臂每步写 qpos 精确跟随，不参与接触 |

### 非目标（明确不做）

- 不碰真机驱动路径（`xhand_control_ros2` C++ 驱动、`pyAgxArm` CAN SDK）
- 不重新实现 IK / 重定向
- **不做相机渲染**（采集用真相机；仿真出图是另一个量级的工作，需另立项目）

## 2. 现状：为什么要这么设计

### 2.1 链路上的接口（已核实）

| 环节 | 话题 | 谁发布 / 谁消费 |
|---|---|---|
| 手命令 | `/left_hand/xhand_command`、`/right_hand/xhand_command` | `xhand_dex_retargeting_node` 发 → `xhand_control_ros2` C++ 驱动收 |
| 手状态 | `/left_hand/xhand_state`、`/right_hand/xhand_state` | C++ 驱动发 → `nero_dual_data_collect` 收 |
| 臂命令 | **不存在** | `nero_teleop_node` 经 `AgxArmFactory` 直接写 CAN（`move_js`） |
| 臂状态 | `/nero_teleop_{left,right}/joint_states` | `nero_teleop_node` 反馈线程 20Hz **从 CAN 读回**后发 |

结论：**手的接口干净，可整块顶替；臂没有 ROS 命令话题，必须小改 teleop。**

### 2.2 teleop 里已有的东西（决定了改动可以很小）

- 已有 `dry_run` 参数，但它**只挡了 `move_js`**（`nero_teleop_node.py:605`）
- 绕过它的地方：CAN 连接块（`connect/enable/set_motion_mode/set_tcp_offset`，
  `:110-135`）、`_move_to_init_pose()`（`:615`，直接 `move_j`）、
  `emergency_stop()`（`electronic_emergency_stop()`）
- `pyAgxArm` 里**没有** mock/sim 后端

### 2.3 手部契约（已核实，sim 必须逐字对齐）

`XHandCommand`：`hand_id, name[12], position[12], kp, ki, kd, effort_limit, mode=3`

`XHAND_JOINT_NAMES`（无前缀，顺序即 position 数组顺序）：
```
thumb_bend_joint, thumb_rota_joint1, thumb_rota_joint2,
index_bend_joint, index_joint1, index_joint2,
mid_joint1, mid_joint2, ring_joint1, ring_joint2,
pinky_joint1, pinky_joint2
```
URDF/MJCF 里对应 `{left,right}_hand_<name>`，12 个 revolute 关节。

`XHandStateArray`：`hand_id, hand_name, hand_type, hand_states[],
sensor_states[{finger_sensor_states[{location, calc_force, raw_force[120],
calc_temperature, raw_temperature[20]}]}]`

`nero_dual_data_collect` 只读 `hand_states[0].position[:12]`（`data_collect_node.py:311`）。
即：**只要 sim 发的 `hand_states[0].position` 顺序对，采集链路零改动可用**；
再填 `sensor_states` 就是仿真触觉数据集。

## 3. 架构

### 3.1 产物布局

```
xnero_ws/src/
├── xhand_nero_description/
│   ├── urdf/xhand_nero_description.urdf      # 已有：含 1200 触觉点
│   └── scripts/make_mjcf.py                  # 新增：URDF → MJCF 生成器
└── xnero_mujoco_sim/                         # 新增包
    ├── package.xml / setup.py / setup.cfg / resource/xnero_mujoco_sim
    ├── xnero_mujoco_sim/
    │   ├── __init__.py
    │   ├── sim_node.py          # 主节点：ROS ↔ MuJoCo 桥
    │   ├── hand_map.py          # 12 关节映射 + 触觉 geom 名 ↔ 点号
    │   ├── fps_counter.py       # 移植自 astral_mujoco_sim
    │   └── latency_meter.py     # 移植自 astral_mujoco_sim
    ├── config/xnero_mujoco_sim.yaml
    ├── launch/
    │   ├── xnero_mujoco_sim.launch.py     # 只起 sim（另发命令）
    │   └── xnero_sim_pipeline.launch.py   # 全链路，无硬件
    └── assets/mjcf/xnero_dual.xml         # 生成产物
```

**mesh 不复制**：MJCF 里 `meshdir` 写成相对路径指回 `xhand_nero_description/meshes`，
节点加载前把它解析成绝对路径再喂 MuJoCo（避免 28MB 重复 + 源码/安装两种布局都成立）。

### 3.2 数据流

```
Quest3 ─→ quest3_udp_mocap ─┬→ xhand_dex_retargeting ─→ /{L,R}_hand/xhand_command ─┐
                            │                                                      │
                            └→ nero_teleop_node (sim_mode:=true)                   │
                                  ├─ 跳过 CAN，发 /nero_teleop_{L,R}/joint_commands ┤
                                  └─ 不发 joint_states（无 CAN，反馈线程自然静默） │
                                                                                   ▼
                                                        xnero_mujoco_sim_node (viewer)
                                                          ├ 手：位置执行器 → mj_step
                                                          ├ 臂：每步写 qpos+qvel=0
                                                          ├ 发 /{L,R}_hand/xhand_state
                                                          │    （含 sensor_states 触觉）
                                                          └ 发 /nero_teleop_{L,R}/joint_states
                                                                                   ▼
                                                        nero_dual_data_collect（零改动）
```

### 3.3 手物理 + 臂运动学的共存机制

- 手：`<position>` 执行器（kp/kv/forcerange 可配）+ `mj_step`，接触真实、触觉真实
- 臂：**每步在 step 之前**把 14 个臂关节 `qpos = 命令值`、`qvel = 0`，再 `mj_step`。
  语义是"**无限刚的 Kinematic 体**"：能推动物体，自己不被推动 —— 这正是采集操作数据
  想要的（操作物会被臂/手推动、可被抓握）。臂不主动产生额外接触；若要臂完全不参与
  接触（比如只想要手部接触），在 MJCF 里给臂 geom 设 `contype=0`，作为可选项。
- 实时节流由 `realtime` 参数控制（`time.sleep(timestep)`）

## 4. MJCF 生成（make_mjcf.py）

流水线：`prep_sim_urdf.py`（已有的：`package://` → 文件路径、34 个退化惯量修复、
关 5 个掌侧指尖 link2 的 mesh 碰撞）→ MuJoCo 编译 → `mj_saveLastXML` 落原生 MJCF
→ 后处理补：

1. `<compiler meshdir="../../../../xhand_nero_description/meshes">`（相对路径）
2. `<option timestep>`（默认 0.002）、`gravity`、`integrator`
3. `<visual>` / 光照 / 地面 plane（可 `--no-floor`）
4. **手部 24 个 `<position>` 执行器**：`name="act_{side}_hand_{joint}"`，
   `kp`/`kv`/`forcerange` 从参数给（命令里的 kp=80 是 XHand 内部单位，不能 1:1 映射）
5. **10 个触觉 `<site>` + `<touch>` 传感器**，site 位置/半径用已实测验证过的那组：
   - 四指（左右 index/mid/ring/pinky）：`pos="0.00487 0 0.03077" size="0.01782"`
   - 左手拇指：`pos="0 -0.03508 0.00269" size="0.02384"`
   - 右手拇指：`pos="0 0.03508 0.00347" size="0.02384"`
6. 探针 geom 的接触参数（`solref`/`solimp`/`friction`，按 `docs/xhand1-tactile-sim-params.md` §5.4）
7. 可选 `<keyframe>` 初始姿态

**幂等**：重复运行覆盖同一产物；产物带生成头注释（含生成命令与日期）。

## 5. sim 节点（sim_node.py）

### 订阅

| 话题 | 类型 | 处理 |
|---|---|---|
| `/{L,R}_hand/xhand_command` | `XHandCommand` | 按 `name` 映射到 12 个执行器 `ctrl`；`mode==3` 位控；`mode==0` 无力（kp=0）；其它模式记录一次 WARN 后按位控处理 |
| `/nero_teleop_{L,R}/joint_commands` | `JointState` | 存为臂目标 q（7 维）。优先按 `name` 匹配 URDF 关节名 `{left,right}_joint1..7`；`name` 为空或对不上时按顺序取前 7 个 |

### 发布

| 话题 | 类型 | 频率 | 内容 |
|---|---|---|---|
| `/{L,R}_hand/xhand_state` | `XHandStateArray` | `state_rate`（默认 50） | `hand_states[0]`：`name`=12 关节名、`position`=MJCF 实际 qpos、`effort`=执行器力、`temperature`=常数；`sensor_states[j].finger_sensor_states[k]`：`location`=5 指名、`calc_force`=该指合力、`raw_force[120]`=**逐点接触力**（按 geom 名映射）、`raw_temperature[20]`=0 |
| `/nero_teleop_{L,R}/joint_states` | `JointState` | `state_rate` | 臂 7 关节实际 qpos |

- `publish_tactile`（默认 true）可关；带宽估算：2 手 × 600 点 × 3 float @50Hz ≈ 1.4 MB/s
- 触觉取力：遍历 `data.contact`，`geom1/geom2` 里以 `tac_` 开头的那个 → 点号 →
  该指的 `raw_force[k]`；法向取 `|res[0]|`，切向取 `res[1:3]`

### 其它

- 线程模型沿用 astral：`_spin_drain` 非阻塞抽干回调 + 主循环物理步进（避免 rclpy 阻塞）
- viewer 可选（`enable_viewer`），相机初始位姿按模型尺度设置
- 仪表：命令 FPS、仿真循环 FPS、延迟（移植 astral 的 `fps_counter`/`latency_meter`）

## 6. teleop 改动（sim_mode）

`nero_quest_teleop/nero_teleop_node.py`，新增参数 `sim_mode`（默认 **false**）：

1. 连接块：`if not sim_mode:` 包住 `connect/enable/set_motion_mode/set_tcp_offset`
2. `_move_to_init_pose()`：sim_mode 下跳过（并记 INFO）
3. `emergency_stop()`：sim_mode 下跳过 `electronic_emergency_stop()`
4. 控制回路：`if sim_mode: 发布 ~/joint_commands (JointState, name=7 关节名)`；
   `elif not _dry_run: move_js(...)`（真机路径原样）

**默认 false ⇒ 真机行为零变化。** 这是硬约束，实现后要用 diff 复核。

## 7. 验证计划（必须给出可复现证据）

1. **MJCF 加载**：`nq/ngeom` 合理、24 个手关节名、1200 个 `tac_*` geom 名、10 个 touch 传感器全在
2. **手部跟踪**：发"握拳"命令 → `hand_states[0].position` 与命令同向收敛（给曲线/数值）
3. **触觉**：手压方块 → `sensor_states[*].raw_force[k]` 非零，且与直接 `mj_contactForce`
   逐点一致（对照打印）
4. **指尖合力**：`calc_force` == 该指 120 点求和
5. **臂**：发 `joint_commands` → MJCF qpos 精确等于命令；`joint_states` 回显一致
6. **全链路**：`xnero_sim_pipeline.launch.py` 无硬件能起，各节点不报错
7. **加分**：`nero_dual_data_collect` 订 sim state 录出一段 h5 并读回验证

## 8. 风险与对策

| 风险 | 对策 |
|---|---|
| 手部增益不匹配（kp=80 是内部单位） | yaml 可配 `hand_kp/hand_kv/torque_limit`，验证 2/3 步实测调出能抓稳的值并写进文档 |
| `xhand_state` 触觉带宽 ~1.4MB/s | `publish_tactile` 开关 + 独立 `tactile_rate` |
| 臂运动学覆盖 ⇒ 臂撞到东西不会停 | 选型的必然结果，文档写明；需要时给臂 geom `contype=0` |
| MJCF 体积（1282 body 融合后仍 ~1-2MB XML） | 一次性加载，不影响运行 |
| 改 teleop 影响真机 | 所有改动在 `sim_mode` 默认 false 之后；实现后 diff 复核真机路径 |

## 9. 交付物清单

1. `xhand_nero_description/scripts/make_mjcf.py` + `xnero_mujoco_sim/assets/mjcf/xnero_dual.xml`
2. `xnero_mujoco_sim/` 完整 ROS 包（节点 + 工具 + config + 两个 launch）
3. `nero_teleop_node.py` 的 `sim_mode`（默认关）
4. `xnero_mujoco_sim/README.md`（用法、话题表、验证方法）
5. 验证证据（第 7 节 1-6 项）
