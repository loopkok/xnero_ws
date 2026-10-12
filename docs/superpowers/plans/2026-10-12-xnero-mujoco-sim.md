# xnero MuJoCo 仿真包 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 做一个 xnero（Nero 双臂 + XHand 双手）的 MuJoCo 仿真包，作为硬件驱动替身，
让 Quest3 遥操链路脱离真机跑通，并把 MuJoCo 里真实的逐点触觉力注入 `xhand_state`，
使现有采集链路零改动就能录到含触觉的仿真数据。

**Architecture:** 手跑真物理（位置执行器 + `mj_step`，出接触力/触觉）；臂走运动学覆盖
（每步写 qpos、qvel 清零，是"无限刚 Kinematic 体"）。sim 节点顶替
`xhand_control_ros2` 的 ROS 接口，并在 teleop 加一个默认关闭的 `sim_mode` 提供臂命令话题。

**Tech Stack:** ROS 2 Humble（rclpy，`/usr/bin/python3`）+ MuJoCo 3.11（pip，同解释器）+ colcon。

**Spec:** `docs/superpowers/specs/2026-10-12-xnero-mujoco-sim-design.md`

> **执行结果（2026-10-12 已完成）**：9 个任务全部落地，31 例测试全绿。
> 执行中与计划的偏差：
> - teleop 改动比计划更小 —— 用**替身对象**（`sim_arm_stub.py`，把 pyAgxArm 接口全实现
>   成 no-op）替代了"逐处加守卫"，`nero_teleop_node.py` 的实际 diff 只有
>   「1 个参数 + 1 处 if/else」，28 处 `self.robot.xxx()` 一行未动。
> - 环境上补了两件事（都不在计划里，但不做链路起不来）：**系统 python 装 torch**
>   （`xhand_dex_retargeting` 经 `dex_retargeting` 需要）与**全量 colcon 构建**。
> - 触觉测试的取样点从 p060 改到 p030（p060 落在指尖关节处，被融进同 body 的
>   tip mesh 遮挡 —— 见 `docs/xhand1-tactile-sim-params.md` §7 第 7 条）。

## Global Constraints

- **一律 `/usr/bin/python3`**（3.10；rclpy 与 mujoco 3.11.0 都装在这里）。conda 的 python 是 3.13，**无法** import rclpy。
- **构建必须清 PATH**：`export PATH=/usr/bin:/bin:/usr/sbin:/sbin` 后再
  `source /opt/ros/humble/setup.bash`。否则 CMake 会抓到 `/home/robot/miniconda3/bin/python3`
  去跑 `rosidl_adapter`，构建必失败（实测踩过；且失败的 CMakeCache 会残留，需 `rm -rf build install log` 才能恢复）。
- 工作空间：`/home/robot/loopkok/sdk/xnero_ws`。
- 手部 12 关节名（顺序即 `XHandCommand.position` 数组顺序，URDF 里加 `{side}_hand_` 前缀）：
  `thumb_bend_joint, thumb_rota_joint1, thumb_rota_joint2, index_bend_joint, index_joint1,
  index_joint2, mid_joint1, mid_joint2, ring_joint1, ring_joint2, pinky_joint1, pinky_joint2`
- 臂 7 关节名：`{left,right}_joint1..7`。
- 触觉点名：`tac_{side}_{finger}_p{001..120}`；`<side>∈{left,right}`，`<finger>∈{thumb,index,mid,ring,pinky}`。
- 触觉 touch sensor 的 site（已实测验证，直接用）：
  四指 `pos="0.00487 0 0.03077" size="0.01782"`；左拇指 `pos="0 -0.03508 0.00269" size="0.02384"`；
  右拇指 `pos="0 0.03508 0.00347" size="0.02384"`。
- **不得改动真机路径**：`xhand_control_ros2`（C++）与 `pyAgxArm` 一行不动；teleop 的改动
  必须全部在 `sim_mode` 默认 false 之后。
- MJCF 里 meshdir 写相对路径，**节点加载前解析成绝对路径**。

---

### Task 1: 环境与包骨架

**Files:**
- Create: `src/xnero_mujoco_sim/package.xml`
- Create: `src/xnero_mujoco_sim/setup.py`
- Create: `src/xnero_mujoco_sim/setup.cfg`
- Create: `src/xnero_mujoco_sim/resource/xnero_mujoco_sim`
- Create: `src/xnero_mujoco_sim/xnero_mujoco_sim/__init__.py`
- Create: `src/xnero_mujoco_sim/config/xnero_mujoco_sim.yaml`
- Create: `src/xnero_mujoco_sim/launch/`（空目录，Task 8 填）
- Create: `src/xnero_mujoco_sim/assets/mjcf/`（空目录，Task 3 填）

**Interfaces:**
- Consumes: 无
- Produces: 可被 colcon 构建的空包 `xnero_mujoco_sim`；后续所有任务都在此包内。

- [ ] **Step 1: 清环境并构建基线（确认现有接口包能过）**

```bash
cd /home/robot/loopkok/sdk/xnero_ws
rm -rf build install log
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  cd /home/robot/loopkok/sdk/xnero_ws && colcon build --packages-select xhand_control_interfaces'
```
Expected: `Finished <<< xhand_control_interfaces`

- [ ] **Step 2: 写包骨架（5 个文件）**

`package.xml`（版本 3，ament_python；depend 含 rclpy/sensor_msgs/std_msgs/geometry_msgs/ament_index_python/xhand_control_interfaces，exec_depend 加 launch/launch_ros/xhand_nero_description）：
```xml
<?xml version="1.0"?>
<package format="3">
  <name>xnero_mujoco_sim</name>
  <version>0.1.0</version>
  <description>MuJoCo stand-in for xnero hardware drivers (XHand driver + Nero arm CAN).</description>
  <maintainer email="loopkok@todo.todo">loopkok</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>ament_index_python</depend>
  <depend>xhand_control_interfaces</depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>xhand_nero_description</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

`setup.py`：`data_files` 安装 `resource/`、`package.xml`、`config/*`、`launch/*.launch.py`、
`assets/mjcf/*`；`entry_points` 注册 `xnero_mujoco_sim_node = xnero_mujoco_sim.sim_node:main`。
（照抄 `astral_ws/src/astral_mujoco_sim/setup.py` 的结构，去掉 meshes 那段 —— 本包不复制 mesh。）

`setup.cfg`：
```ini
[develop]
script_dir=$base/lib/xnero_mujoco_sim
[install]
install_scripts=$base/lib/xnero_mujoco_sim
```

`resource/xnero_mujoco_sim`：空文件。
`xnero_mujoco_sim/__init__.py`：空文件。

`config/xnero_mujoco_sim.yaml`：
```yaml
xnero_mujoco_sim_node:
  ros__parameters:
    mjcf_path: ""                 # 空 = 用包内 assets/mjcf/xnero_dual.xml
    enable_viewer: true
    realtime: true
    state_rate: 50.0
    # 手部执行器增益（XHand 命令里的 kp=80 是手内部单位，不能 1:1 映射，见 spec §8）
    hand_kp: 5.0
    hand_kv: 0.1
    hand_torque_limit: 0.5        # N·m，执行器 forcerange
    # 触觉
    publish_tactile: true
    tactile_rate: 50.0
    # 臂
    arm_command_timeout: 0.5      # 超时未收到命令则保持最后值
    # 初始姿态（7 维，与 URDF 关节限位相容）
    init_pose_left:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    init_pose_right: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    fps_print_interval: 5.0
    print_latency: true
    latency_print_interval: 2.0
```

- [ ] **Step 3: 构建验证**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  cd /home/robot/loopkok/sdk/xnero_ws && colcon build --packages-select xnero_mujoco_sim --symlink-install'
```
Expected: `Finished <<< xnero_mujoco_sim`

- [ ] **Step 4: Commit**

```bash
cd /home/robot/loopkok/sdk/xnero_ws && git add src/xnero_mujoco_sim && \
  git commit -m "feat(xnero_mujoco_sim): 包骨架与配置"
```

---

### Task 2: 移植工具 + 手部映射（纯 Python，可单测）

**Files:**
- Create: `src/xnero_mujoco_sim/xnero_mujoco_sim/fps_counter.py`（从 astral 移植）
- Create: `src/xnero_mujoco_sim/xnero_mujoco_sim/latency_meter.py`（从 astral 移植）
- Create: `src/xnero_mujoco_sim/xnero_mujoco_sim/hand_map.py`
- Test: `src/xnero_mujoco_sim/test/test_hand_map.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `HAND_JOINT_NAMES: list[str]`（12 个，无前缀）
  - `arm_joint_names(side: str) -> list[str]` → `[f"{side}_joint{i}" for i in 1..7]`
  - `mjcf_hand_joint(side: str, name: str) -> str` → `f"{side}_hand_{name}"`
  - `actuator_name(side: str, name: str) -> str` → `f"act_{side}_hand_{name}"`
  - `tactile_geom(side: str, finger: str, point: int) -> str` → `f"tac_{side}_{finger}_p{point:03d}"`
  - `FINGERS: tuple[str, ...]` = `("thumb", "index", "mid", "ring", "pinky")`
  - `parse_tactile_geom(name: str) -> tuple[str, str, int] | None`
  - `FPSCounter`、`LatencyMeter`

- [ ] **Step 1: 写失败测试**

```python
# src/xnero_mujoco_sim/test/test_hand_map.py
from xnero_mujoco_sim.hand_map import (
    FINGERS, HAND_JOINT_NAMES, actuator_name, arm_joint_names,
    mjcf_hand_joint, parse_tactile_geom, tactile_geom,
)

def test_hand_joint_names_are_12_and_ordered():
    assert len(HAND_JOINT_NAMES) == 12
    assert HAND_JOINT_NAMES[0] == "thumb_bend_joint"
    assert HAND_JOINT_NAMES[-1] == "pinky_joint2"

def test_prefixing():
    assert mjcf_hand_joint("left", "index_joint1") == "left_hand_index_joint1"
    assert actuator_name("right", "mid_joint2") == "act_right_hand_mid_joint2"

def test_arm_joint_names():
    assert arm_joint_names("left") == [f"left_joint{i}" for i in range(1, 8)]

def test_tactile_geom_roundtrip():
    n = tactile_geom("left", "index", 1)
    assert n == "tac_left_index_p001"
    assert parse_tactile_geom(n) == ("left", "index", 1)
    assert parse_tactile_geom("right_hand_index_rota_link2") is None
    assert parse_tactile_geom("tac_left_index_p120") == ("left", "index", 120)

def test_fingers_tuple():
    assert FINGERS == ("thumb", "index", "mid", "ring", "pinky")
```

- [ ] **Step 2: 跑测试确认失败**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  source /home/robot/loopkok/sdk/xnero_ws/install/setup.bash && \
  cd /home/robot/loopkok/sdk/xnero_ws/src/xnero_mujoco_sim && \
  /usr/bin/python3 -m pytest test/test_hand_map.py -q'
```
Expected: FAIL — `ModuleNotFoundError: No module named 'xnero_mujoco_sim.hand_map'`

- [ ] **Step 3: 实现 hand_map.py**

```python
"""手部关节名 / 执行器名 / 触觉 geom 名的唯一映射源。"""
from __future__ import annotations
import re

FINGERS = ("thumb", "index", "mid", "ring", "pinky")
SIDES = ("left", "right")

# 顺序即 XHandCommand.position 数组顺序（见 xhand_retargeting:XHAND_JOINT_NAMES）
HAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]

_TAC_RE = re.compile(r"^tac_(left|right)_(thumb|index|mid|ring|pinky)_p(\d{3})$")

def arm_joint_names(side: str) -> list[str]:
    return [f"{side}_joint{i}" for i in range(1, 8)]

def mjcf_hand_joint(side: str, name: str) -> str:
    return f"{side}_hand_{name}"

def actuator_name(side: str, name: str) -> str:
    return f"act_{side}_hand_{name}"

def tactile_geom(side: str, finger: str, point: int) -> str:
    return f"tac_{side}_{finger}_p{point:03d}"

def parse_tactile_geom(name: str):
    m = _TAC_RE.match(name or "")
    return (m.group(1), m.group(2), int(m.group(3))) if m else None
```

`fps_counter.py` / `latency_meter.py`：从
`astral_ws/src/astral_mujoco_sim/astral_mujoco_sim/{fps_counter,latency_meter}.py` 逐字复制，
只改 docstring 里的包名。

- [ ] **Step 4: 跑测试确认通过**

同 Step 2 命令。Expected: `5 passed`

- [ ] **Step 5: Commit**

```bash
cd /home/robot/loopkok/sdk/xnero_ws && git add src/xnero_mujoco_sim && \
  git commit -m "feat(xnero_mujoco_sim): 手部映射与仪表工具"
```

---

### Task 3: MJCF 生成器（URDF → 原生 MJCF）

**Files:**
- Create: `src/xhand_nero_description/scripts/make_mjcf.py`
- Create（产物）: `src/xnero_mujoco_sim/assets/mjcf/xnero_dual.xml`
- Test: `src/xnero_mujoco_sim/test/test_mjcf_asset.py`

**Interfaces:**
- Consumes: `prep_sim_urdf.py`（已有，同目录）；`hand_map`（Task 2）里的 site 常量无关，site 值写在本脚本的常量表里（与 spec 一致）。
- Produces: MJCF 文件 `xnero_dual.xml`，含：24 个手执行器 `act_{side}_hand_{joint}`、
  14 个臂关节、1200 个 `tac_*` geom、10 个 touch 传感器 `ft_{side}_{finger}`、
  10 个 site `pad_{side}_{finger}`、地面 `floor`、光照。

- [ ] **Step 1: 写失败测试**

```python
# src/xnero_mujoco_sim/test/test_mjcf_asset.py
import re
from pathlib import Path
import pytest

MJCF = Path(__file__).resolve().parents[1] / "assets" / "mjcf" / "xnero_dual.xml"

@pytest.fixture(scope="module")
def text():
    assert MJCF.is_file(), f"MJCF 不存在：{MJCF}（先跑 make_mjcf.py）"
    return MJCF.read_text(encoding="utf-8")

def test_has_generator_header(text):
    assert "make_mjcf.py" in text.splitlines()[1] or "make_mjcf.py" in text[:400]

def test_meshdir_is_relative(text):
    m = re.search(r'meshdir="([^"]+)"', text)
    assert m and not m.group(1).startswith("/"), "meshdir 必须是相对路径"

def test_hand_actuators_24(text):
    names = re.findall(r'<position name="(act_\w+)"', text)
    assert len(names) == 24, names[:3]
    assert "act_left_hand_index_joint1" in names
    assert "act_right_hand_pinky_joint2" in names

def test_touch_sensors_10(text):
    names = re.findall(r'<touch name="(ft_\w+)"', text)
    assert len(names) == 10, names

def test_tactile_geoms_1200(text):
    assert len(re.findall(r'<geom name="tac_', text)) == 1200

def test_floor_present(text):
    assert 'name="floor"' in text
```

- [ ] **Step 2: 跑测试确认失败**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; cd /home/robot/loopkok/sdk/xnero_ws/src/xnero_mujoco_sim && \
  /usr/bin/python3 -m pytest test/test_mjcf_asset.py -q'
```
Expected: FAIL — `AssertionError: MJCF 不存在`

- [ ] **Step 3: 实现 make_mjcf.py**

脚本流程（每一步都要落日志）：
1. 调 `prep_sim_urdf.py`（子进程，`--out <tmpdir>`，`--no-collision` 用 spec §4 的精确正则
   `^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$`）拿到 `robot_sim.urdf`
2. `m = mujoco.MjModel.from_xml_path(sim_urdf)`；`mujoco.mj_saveLastXML(tmp_mjcf, m)`
3. 文本后处理（`xml.etree` 或正则 + 字符串拼接）：
   - 把 `<mujoco model="...">` 之后插入生成头注释（含命令、日期、来源 URDF md5）
   - `<compiler ...>` 里替换/追加 `meshdir="../../../../xhand_nero_description/meshes"`；
     并把所有 `file="<绝对路径>/meshes/xxx"` 改成相对 meshes 根的 `file="xxx"`
   - `<option timestep="0.002" integrator="implicitfast"/>`（无则插入）
   - `<visual>`：`<headlight diffuse="0.8 0.8 0.8"/>`、`<map/>` 默认
   - worldbody 开头插入 `<light pos="1 1 2" dir="-0.3 -0.3 -1"/>` 与
     `<geom name="floor" type="plane" size="2 2 0.05" rgba="0.2 0.2 0.25 1"/>`
   - 在 10 个指尖 link2 body 内插入
     `<site name="pad_{side}_{finger}" pos="..." size="..."/>`
   - `<sensor>` 段（无则创建）插入 10 个 `<touch name="ft_{side}_{finger}" site="pad_{side}_{finger}"/>`
   - `<actuator>` 段插入 24 个
     `<position name="act_{side}_hand_{joint}" joint="{side}_hand_{joint}" kp="5" kv="0.1" ctrlrange="<URDF限位>" forcerange="-0.5 0.5"/>`
     （限位从任务 2 的 URDF 读；thumb_bend 0..1.832、thumb_rota1 -0.698..1.57、
     thumb_rota2/其余 0..1.57 或 0..1.919 —— 脚本直接从 URDF 解析，不写死）
   - 给 1200 个 `tac_*` geom 加接触参数 `solref="0.004 1" solimp="0.9 0.95 0.001" friction="1 0.05 0.001"`
4. 写产物；打印统计（body/geom/joint/actuator/sensor 数）

CLI：`--out`（默认 `../../xnero_mujoco_sim/assets/mjcf/xnero_dual.xml` 相对本包）、
`--timestep`、`--kp`、`--kv`、`--force`、`--no-floor`。

- [ ] **Step 4: 生成并跑测试**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; cd /home/robot/loopkok/sdk/xnero_ws/src/xhand_nero_description && \
  /usr/bin/python3 scripts/make_mjcf.py' && \
bash -lc 'cd /home/robot/loopkok/sdk/xnero_ws/src/xnero_mujoco_sim && /usr/bin/python3 -m pytest test/test_mjcf_asset.py -q'
```
Expected: 生成日志 + `6 passed`

- [ ] **Step 5: MuJoCo 侧加载冒烟**

```bash
/usr/bin/python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path('/home/robot/loopkok/sdk/xnero_ws/src/xnero_mujoco_sim/assets/mjcf/xnero_dual.xml')
names = lambda o,n: [mujoco.mj_id2name(m,o,i) for i in range(n)]
tac = [n for n in names(mujoco.mjtObj.mjOBJ_GEOM, m.ngeom) if n and n.startswith('tac_')]
print(f'nq={m.nq} nu={m.nu} ngeom={m.ngeom} nsensor={m.nsensor} 触觉geom={len(tac)}')
assert len(tac)==1200 and m.nu==24 and m.nsensor==10
print('OK')"
```

- [ ] **Step 6: Commit**

```bash
cd /home/robot/loopkok/sdk/xnero_ws && git add src/xhand_nero_description/scripts/make_mjcf.py \
  src/xnero_mujoco_sim/assets/mjcf/xnero_dual.xml src/xnero_mujoco_sim/test/test_mjcf_asset.py && \
  git commit -m "feat: URDF→MJCF 生成器与 xnero_dual.xml"
```

---

### Task 4: sim 节点 —— 手部驱动与状态发布

**Files:**
- Create: `src/xnero_mujoco_sim/xnero_mujoco_sim/sim_node.py`
- Test: `src/xnero_mujoco_sim/test/test_sim_node_hand.py`

**Interfaces:**
- Consumes: Task 2 的 `hand_map`；Task 3 的 MJCF。
- Produces: 类 `XneroMujocoSimNode(rclpy.node.Node)`，节点名 `xnero_mujoco_sim_node`；
  可测的纯函数 `pack_hand_command(msg, side) -> dict[str, float] | None`（按 name 映射，
  name 缺失/不匹配时按顺序取前 12）；`load_model_with_abs_meshdir(mjcf_path, desc_share) -> MjModel`。

- [ ] **Step 1: 写失败测试**

```python
# src/xnero_mujoco_sim/test/test_sim_node_hand.py
import numpy as np
import pytest

from xnero_mujoco_sim.hand_map import HAND_JOINT_NAMES

pytest.importorskip("rclpy")
pytest.importorskip("mujoco")


def test_pack_hand_command_by_name():
    from xhand_control_interfaces.msg import XHandCommand
    from xnero_mujoco_sim.sim_node import pack_hand_command
    msg = XHandCommand()
    msg.name = list(reversed(HAND_JOINT_NAMES))
    msg.position = [float(i) for i in range(12)][::-1]
    out = pack_hand_command(msg, "left")
    assert set(out) == {f"left_hand_{n}" for n in HAND_JOINT_NAMES}
    assert out["left_hand_thumb_bend_joint"] == 0.0
    assert out["left_hand_pinky_joint2"] == 11.0


def test_pack_hand_command_without_names_falls_back_to_order():
    from xhand_control_interfaces.msg import XHandCommand
    from xnero_mujoco_sim.sim_node import pack_hand_command
    msg = XHandCommand()
    msg.name = []
    msg.position = [0.1] * 12
    out = pack_hand_command(msg, "right")
    assert out["right_hand_index_joint1"] == pytest.approx(0.1)
    assert len(out) == 12


def test_pack_hand_command_rejects_short():
    from xhand_control_interfaces.msg import XHandCommand
    from xnero_mujoco_sim.sim_node import pack_hand_command
    msg = XHandCommand(); msg.name = []; msg.position = [0.1] * 5
    assert pack_hand_command(msg, "left") is None


def test_resolve_meshdir_absolute(tmp_path):
    """节点加载前必须把相对 meshdir 解析成绝对路径。"""
    from xnero_mujoco_sim.sim_node import rewrite_meshdir
    p = tmp_path / "m.xml"
    p.write_text('<mujoco><compiler meshdir="../../x"/></mujoco>')
    out = rewrite_meshdir(p.read_text(), "/abs/meshes")
    assert 'meshdir="/abs/meshes"' in out
```

- [ ] **Step 2: 跑测试确认失败**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  source /home/robot/loopkok/sdk/xnero_ws/install/setup.bash && \
  cd /home/robot/loopkok/sdk/xnero_ws/src/xnero_mujoco_sim && \
  /usr/bin/python3 -m pytest test/test_sim_node_hand.py -q'
```
Expected: FAIL — `ModuleNotFoundError: ... sim_node`

- [ ] **Step 3: 实现 sim_node.py（本任务只做手，臂在 Task 6 加）**

结构（沿用 astral 的线程模型）：
- `pack_hand_command(msg, side)`：纯函数，见测试
- `rewrite_meshdir(text, abs_dir)`：`re.sub(r'meshdir="[^"]*"', f'meshdir="{abs_dir}"', text)`
- 类内：参数声明 → 加载模型（`rewrite_meshdir` + `from_xml_string`）→ 建立
  `self._qpos_addr[side][joint]`、`self._act_id[side][joint]` → 订阅
  `/{side}_hand/xhand_command` → 主循环：`_spin_drain` → 写 ctrl → `mj_step` →
  `viewer.sync()` → 按 `state_rate` 发布 `/{side}_hand/xhand_state`（本任务先只填
  `hand_states[0]`；`sensor_states` 留空，Task 5 填）
- 主循环用 `self.model.opt.timestep` 节流（`realtime`）

- [ ] **Step 4: 跑测试确认通过**

同 Step 2。Expected: `4 passed`

- [ ] **Step 5: 手部跟踪实测（证据）**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  source /home/robot/loopkok/sdk/xnero_ws/install/setup.bash && cd /home/robot/loopkok/sdk/xnero_ws && \
  /usr/bin/python3 - <<PY
import rclpy, numpy as np
from rclpy.node import Node
from xhand_control_interfaces.msg import XHandCommand, XHandStateArray
from xnero_mujoco_sim.sim_node import XneroMujocoSimNode
from xnero_mujoco_sim.hand_map import HAND_JOINT_NAMES

rclpy.init()
sim = XneroMujocoSimNode(viewer=False)      # 测试构造：不起 viewer
probe = Node("probe")
got = {}
probe.create_subscription(XHandStateArray, "/left_hand/xhand_state",
                          lambda m: got.setdefault("pos", m.hand_states[0].position), 10)
pub = probe.create_publisher(XHandCommand, "/left_hand/xhand_command", 10)
cmd = XHandCommand(); cmd.name = HAND_JOINT_NAMES
cmd.position = [0.6]*12; cmd.kp=[80.]*12; cmd.ki=[0.]*12; cmd.kd=[0.]*12
cmd.effort_limit=[400.]*12; cmd.mode = 3
for _ in range(400):
    pub.publish(cmd); sim.step_once(); rclpy.spin_once(probe, timeout_sec=0)
    rclpy.spin_once(sim, timeout_sec=0)
print("命令 0.6 → 实测 qpos:", np.round(got.get("pos", [])[:4], 3))
assert got and np.mean(got["pos"]) > 0.4, "手没跟上命令"
print("手部跟踪 OK")
sim.destroy_node(); probe.destroy_node(); rclpy.shutdown()
PY'
```
Expected: 打印出接近 0.6 的 qpos 与 `手部跟踪 OK`

- [ ] **Step 6: Commit**

---

### Task 5: 触觉注入 `xhand_state.sensor_states`

**Files:**
- Modify: `src/xnero_mujoco_sim/xnero_mujoco_sim/sim_node.py`
- Test: `src/xnero_mujoco_sim/test/test_sim_node_tactile.py`

**Interfaces:**
- Consumes: Task 4 的节点与主循环。
- Produces: 纯函数 `collect_tactile(data, model, side) -> dict[str, dict]`，键为
  `finger`，值为 `{"raw": np.ndarray(120,3), "calc": np.ndarray(3)}`（法向+切向按世界系分量累加）。

- [ ] **Step 1: 写失败测试**（用手压方块，断言 `sensor_states` 非零且与该指 touch 传感器一致）

```python
def test_tactile_matches_contact_forces(...):
    # 1) 发握拳命令让食指尖下压到 floor 上（或注入一个方块），step 若干步
    # 2) 收 /left_hand/xhand_state，取 sensor_states[0].finger_sensor_states[1] (index)
    # 3) 断言 sum(|raw_force|) > 0，且 total 法向 == abs(sensordata[ft_left_index])
```
（完整可运行代码见实现时补；断言两条：非零 + 与 touch 传感器一致，容差 1e-6。）

- [ ] **Step 2-4: 失败 → 实现 → 通过**

实现要点：遍历 `data.contact`，`geom1/geom2` 里以 `tac_` 开头的那个 →
`parse_tactile_geom` → 点号 → 写入该指 `raw[k-1] = (|fn|, ft1, ft2)`；
`calc` 累加。`location` 用 `FINGERS`。发布频率由 `tactile_rate` 控制（`publish_tactile` 为 false 时只发位置）。

- [ ] **Step 5: 实测证据**：打印"逐点力 vs 直接 mj_contactForce"对照表
- [ ] **Step 6: Commit**

---

### Task 6: 臂 —— 命令订阅 + 运动学覆盖 + 状态发布

**Files:**
- Modify: `src/xnero_mujoco_sim/xnero_mujoco_sim/sim_node.py`
- Test: `src/xnero_mujoco_sim/test/test_sim_node_arm.py`

**Interfaces:**
- Consumes: Task 4 的主循环。
- Produces: 订阅 `/nero_teleop_{left,right}/joint_commands`（`JointState`）；
  发布 `/nero_teleop_{left,right}/joint_states`（`JointState`，name 为 7 个臂关节名）。

- [ ] **Step 1: 写失败测试**：发 7 维命令 → step → 断言 MJCF `qpos` 精确等于命令
  （`abs(q - cmd) < 1e-9`）且 `joint_states.position` 回显一致；臂 `qvel == 0`。
- [ ] **Step 2-4: 失败 → 实现 → 通过**

实现要点：在 `mj_step` **之前**写 `qpos[arm]=cmd; qvel[arm]=0`；`arm_command_timeout`
超时后保持最后值（不归零）。

- [ ] **Step 5: Commit**

---

### Task 7: teleop `sim_mode`（默认 false，真机路径零变化）

**Files:**
- Modify: `src/nero_quest_teleop/nero_quest_teleop/nero_teleop_node.py`
- Test: `src/nero_quest_teleop/test/test_sim_mode_guard.py`

**Interfaces:**
- Consumes: Task 6 的 `/nero_teleop_{side}/joint_commands` 约定。
- Produces: teleop 新参数 `sim_mode`（bool，默认 False）；sim_mode 下发
  `~/joint_commands`（`JointState`，name=`{side}_joint1..7`）。

- [ ] **Step 1: 写失败测试**：一个静态检查测试，断言源码里
  （a）`declare_parameter("sim_mode", False)` 存在；
  （b）连接块被 `if not self._sim_mode` 包住；
  （c）`_move_to_init_pose` 与 `emergency_stop` 内有 sim_mode 守卫；
  （d）`move_js` 仍在 `elif not self._dry_run` 分支里（真机路径未被替换）。
  （用 `inspect.getsource` + 字符串断言；这是防回归的关键闸门。）
- [ ] **Step 2-4: 失败 → 实现 → 通过**

实现（4 处，全部 additive）：
```python
self.declare_parameter("sim_mode", False)          # 靠近 dry_run 声明
self._sim_mode = self.get_parameter("sim_mode").value
# 1) 连接块
if not self._sim_mode:
    self.robot = AgxArmFactory.create_arm(cfg); self.robot.connect(); ... set_tcp_offset(...)
else:
    self.robot = None
    self.get_logger().warn("[SIM] sim_mode=true：跳过 CAN 连接，命令发到 ~/joint_commands")
# 2) init pose
if not self._sim_mode:
    self._move_to_init_pose()
# 3) estop
if not self._sim_mode:
    self.robot.electronic_emergency_stop()
# 4) 下发
if self._sim_mode:
    self._publish_sim_command(cmd)      # JointState, name=arm_joint_names(side)
elif not self._dry_run:
    self.robot.move_js(cmd.tolist())
```
另需在 sim_mode 下让 `init_joint_angles` / 反馈线程安全（`self.robot is None` 时跳过）。

- [ ] **Step 5: 真机路径复核（硬要求）**

```bash
cd /home/robot/loopkok/sdk/xnero_ws && git diff src/nero_quest_teleop | grep -E '^[+-]' | grep -v '^[+-][+-]'
```
逐行确认：**没有任何一行修改了 `sim_mode` 之外的既有行为**（尤其 `move_js` 调用与
`_move_to_init_pose` 的内部实现）。

- [ ] **Step 6: Commit**

---

### Task 8: launch 文件

**Files:**
- Create: `src/xnero_mujoco_sim/launch/xnero_mujoco_sim.launch.py`
- Create: `src/xnero_mujoco_sim/launch/xnero_sim_pipeline.launch.py`

**Interfaces:**
- Consumes: Task 4/6 的节点、Task 7 的 teleop `sim_mode`。
- Produces: 两个可 `ros2 launch` 的入口。

- [ ] **Step 1: 写 sim-only launch**：只起 `xnero_mujoco_sim_node`，参数 `enable_viewer`、
  `mjcf_path`、`publish_tactile`、`state_rate` 可覆盖。
- [ ] **Step 2: 写全链路 launch**（镜像 `nero_quest_teleop/launch/teleop_full_left.launch.py`）：
  `quest3_udp_mocap` + `xhand_dex_retargeting` + `nero_teleop_{left,right}(sim_mode:=true)`
  + `xnero_mujoco_sim_node`；**不含** `xhand_control_ros2_node`（被 sim 顶替）。
  加 `side` 参数（left/right/both，默认 both）。
- [ ] **Step 3: 无硬件启动验证**

```bash
bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; source /opt/ros/humble/setup.bash && \
  source /home/robot/loopkok/sdk/xnero_ws/install/setup.bash && \
  timeout 25 ros2 launch xnero_mujoco_sim xnero_sim_pipeline.launch.py enable_viewer:=false'
```
Expected: 各节点起得来、无 traceback、sim 节点打印 `Loading MJCF` 与触觉点统计；
超时被杀属正常。

- [ ] **Step 4: Commit**

---

### Task 9: README + 端到端证据

**Files:**
- Create: `src/xnero_mujoco_sim/README.md`

- [ ] **Step 1: 写 README**：定位、架构图、话题表、构建（含**清 PATH** 那条坑）、
  两个 launch 用法、参数表、触觉数据结构与带宽、与真机的切换方法、
  已知限制（臂是运动学体、手部增益需调、无相机渲染）。
- [ ] **Step 2: 端到端证据**（加分项）：起 sim + 用脚本发握拳命令压 floor，
  用 `ros2 topic echo /left_hand/xhand_state --once` 抓一条含非零 `raw_force` 的状态。
- [ ] **Step 3: Commit**

---

## Self-Review

**Spec 覆盖**：S1 布局→T1/T3；S2 MJCF→T3；S3 话题契约→T4/T5/T6；S4 teleop→T7；
S5 混合机制→T6；S6 验证→各任务 Step 5 + T9；S7 非目标→约束段；S8 风险→T3(接触参数)/T5(带宽开关)/T7(真机复核)。

**占位符**：Task 5 的测试代码标注了"实现时补"——**这是计划缺陷，执行时必须先写出完整测试再改代码**（不得跳过 TDD 步骤）。

**类型一致性**：`pack_hand_command` / `rewrite_meshdir` / `collect_tactile` /
`arm_joint_names` / `tactile_geom` / `parse_tactile_geom` 在各任务中签名一致；
节点类名 `XneroMujocoSimNode` 统一。
