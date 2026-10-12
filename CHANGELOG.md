# Changelog（xnero_ws）

## 2026-10-12

**XHAND1 触觉阵列内嵌 URDF、仿真预处理与参数文档**——`xhand_nero_description`。

**背景**：XHAND1 每个指尖有一片 120 点三轴触觉阵列（拇指 T30、其余四指 T16），厂商只给出三份
离散的点位 JSON（坐标已在各指 link2 系，单位 mm），既没进模型也没进文档；而这份厂商 URDF
**直接喂 MuJoCo 会失败**，触觉点与指尖网格共面也会让逐点力失真。

**做法**：① `scripts/generate_tactile_points.py` 把三份 JSON 转成 **1200 个 link**
（`<side>_hand_<finger>_tac_p###` + 同名 fixed joint + **命名** collision 球 + visual 球）
原地写回 `urdf/xhand_nero_description.urdf`，用 `<!-- TACTILE-BEGIN/END -->` 包裹，
幂等可重跑、`--remove-tactile` 可字节级还原厂商原版；半径/外凸/密度/visual 均有开关。
② `scripts/prep_sim_urdf.py` 修仿真器侧的三件事：`package://` 换文件系统路径、
**修 34 个退化惯量**（厂商把六个惯量分量含非对角全写成同一个 `1e-11`，是秩 1 矩阵，
MuJoCo 报 `inertia must have positive eigenvalues` 拒收整个模型；做法是保留对角量级、
非对角置 0）、可选关指定 link 的 collision。③ `docs/xhand1-tactile-sim-params.md` 给出
模组规格、Mooney-Rivlin 材料参数（C1 −0.03215 / C2 0.148075 / C3 0.002175 MPa，
ρ 1100 kg/m³ → 折算 μ=2(C1+C2)=0.232 MPa、E≈3μ≈0.696 MPa）、点间距统计、
MuJoCo/Isaac Lab 2.x 两端接法与已知坑。

**关键实测**：点位贴合网格（点到面距离中位数 0.32–0.37 mm，最大 1.20 mm，10 处全部通过）；
点间距由 JSON 复算与 PDF 5.1 节**逐位吻合**（T16 平均 2.203/最小 1.396/最大 2.585 mm，
T30 2.714/1.720/3.436 mm）；URDF 里 index 指尖 joint3 的 `origin z=0.0422482924089424`
与 PDF FAQ 引用值一致，确认同源。**触觉点与指尖 mesh 共面**：保留 mesh 碰撞时压入
0.8 mm，总法向力 9.79 N 中**有 3.86 N（39.4%）被 mesh 吃掉**且逐点分布失真
（p099=2.56/p100=3.37，正确值 0.40/2.72），故触觉模式必须关 5 个掌侧 link2 的 collision
——正则须精确到 `^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$`，
用 `.*link2$` 会误伤 22 个 link（实测，含 nero 臂与手背）。

**验证**：生成器连跑两次 md5 一致；`--remove-tactile` 与厂商原版逐字节相同；
1282 link/1281 joint 无重名无悬空；1200 点坐标逐点核对 0 处不符；`check_urdf` 解析通过；
MuJoCo 加载后 1200 个触觉 geom 名字全可查。`scripts/test_tactile_mujoco.py` 可复跑自检。

**xnero MuJoCo 仿真包（手物理 + 臂运动学，触觉注入 xhand_state）**——新增 `xnero_mujoco_sim`。

**背景**：遥操链路缺一个不依赖真机的执行端。参考 `astral_ws/src/astral_mujoco_sim`，
但 astral 那个是纯运动学替身（写 qpos + `mj_forward`、geom 全 `contype=0`），
没有接触也就没有触觉；而 xnero 这边刚做完触觉阵列，需要真实的逐点接触力。

**做法**：① `xhand_nero_description/scripts/make_mjcf.py` 从 URDF 生成 MuJoCo 原生 MJCF
（复用上面的预处理器 → MuJoCo 编译 → `mj_saveLastXML` → 后处理补 24 个手部位置执行器、
10 个指尖 `touch` 传感器、1200 个触觉 geom 的接触参数、地面光照 option，
meshdir 写相对路径、节点加载时解析成绝对路径）。② `xnero_mujoco_sim` 节点作为
**XHand 串口驱动的替身**：订 `/{left,right}_hand/xhand_command` 驱动 12 个手关节，
发 `/{left,right}_hand/xhand_state`——`hand_states[0].position` 与真机同契约，
`sensor_states[i].finger_sensor_states[j].raw_force[120]` 填 MuJoCo 的**逐点接触力**
（`raw_force[k]` ↔ 测点 `p{k+1}` ↔ SDK 的 `raw_force[k]`）。**手跑真物理**
（`<position>` 执行器 + `mj_step`），**臂走运动学覆盖**（每步写 qpos、qvel 清零，
语义是"无限刚 Kinematic 体"：能推物体、自己不被推动）。③ 两个 launch：
`xnero_mujoco_sim.launch.py`（只起仿真）与 `xnero_sim_pipeline.launch.py`
（quest3 → retargeting → teleop(sim) → 仿真，**不起** XHand 串口驱动）。

**关键实测**：MuJoCo 的 URDF 导入默认 `fusestatic=true`，1200 个点 body 会被融进对应指尖
body 但 **geom 全保留且名字还在**（geom 名取自 `<collision name>`）——所以逐点力靠 geom 名读、
DOF 不增加。`touch` 传感器语义是"**site 体积范围内**该 body 的接触法向力之和"，site 放错位置
读 0；用 120 点质心 + 覆盖半径标定的 site，实测与逐点求和**逐档完全一致（差 0.0000）**，
食指+中指同压时两指各读各的（2.6025 / 10.8178 N）**不串味**。**`fusestatic` 会把
fixed 子链（`*_rota_tip`、`*_rotaback_link2`）的碰撞 mesh 融进同一个 body**，
而 T16 是列优先网格、第 12 行（如 p060）的 z≈42.08 mm 正贴指尖关节（42.25 mm），
压上去先碰到那几片 mesh —— 测触觉要用 pad 中段的点（文档已记）。

**验证**：31 例测试全绿（手部命令打包与跟踪、meshdir 解析、MJCF 产物结构、触觉逐点力与
touch 传感器一致、臂运动学覆盖精确到 1e-9、sim_mode 防回归）；无硬件起全链路 launch
5 个进程全存活无 traceback；端到端 ROS 往返实测命令 1.2 → 关节 1.189/1.199、
食指 1 点出 1.798 N、touch 传感器 0.747 N 对得上。

**teleop 新增 `sim_mode`（默认关，真机路径零改动）**——`nero_quest_teleop`。

**背景**：`nero_teleop_node` 经 `AgxArmFactory` **直接写 CAN**，没有 ROS 命令话题，
且 teleop 里有 28 处 `self.robot.xxx()`（连接/使能/模式/下发/回读/急停/归零/断开），
逐处加 `if sim_mode` 既脏又易漏。

**做法**：新增 `nero_quest_teleop/sim_arm_stub.py`，把 pyAgxArm 的接口**全部实现成 no-op**，
只在 `move_js`/`move_j` 上把命令发到 `~/joint_commands`；`get_joint_angles()` 返回上一条命令
（把仿真臂当理想跟随器，使 `_move_to_init_pose()` 与归零流程立刻收敛而非空等 15 s 超时），
`get_flange_pose()` 返回 None（拿不到 FK，反馈线程据此不发 `joint_states`，臂状态改由仿真节点发）。
`nero_teleop_node.py` 的实际改动只有「新增 `sim_mode` 参数 + 把
`AgxArmFactory.create_arm(cfg)` 包进 if/else」共 **+10/−1 行**，28 处调用一行未动；
`sim_mode` 默认 `false`，真机路径不变（有 `test_sim_mode_stub.py` 静态守卫防回归）。

**验证**：`git diff` 逐行复核真机路径未被修改；全链路 launch 中两个臂节点均打印
`[SIM] sim_mode=true：不连接 CAN`；手动 `ros2 topic pub` 臂命令后 `joint_states` 回显
与命令**逐位相同**（`[0.0, -0.3, 0.0, 0.5, 0.0, 0.0, 0.0]`）。

**环境与工件**：本机补装 `torch 2.14.1+cpu`（`xhand_dex_retargeting` 经 `dex_retargeting`
需要，按用户要求装系统环境 `/usr/bin/python3`，未用 conda）；全工作空间 9 个包
`colcon build --symlink-install` 通过。**构建必须清 PATH**
（`export PATH=/usr/bin:/bin:/usr/sbin:/sbin`），否则 CMake 抓到 conda 的 python3.13 跑
`rosidl_adapter` 必失败，且 `CMakeCache.txt` 残留会让后续构建一直失败（已记入 README）。
设计与实施文档见 `docs/superpowers/specs/2026-10-12-xnero-mujoco-sim-design.md`
与 `docs/superpowers/plans/2026-10-12-xnero-mujoco-sim.md`。

**已知遗留**：SDK 的 `raw_force[k]` ↔ 测点 `p{k+1}` 序号对应厂商文档未明示，
首次上实机需单点按压验证一次；本包不含相机渲染；臂为运动学体（不会因碰撞停下）。
