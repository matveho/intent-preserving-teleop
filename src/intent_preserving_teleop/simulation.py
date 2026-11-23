"""MuJoCo peg-in-hole environment with a 6-DoF Cartesian pose interface.

The public action is a bounded Cartesian position increment. A damped least-
squares inverse-kinematics step maps it to the seven Gen3 joints, so callers
(including a six-DoF PHANToM) do not need to command joint seven directly.
"""


"""
Needs to be verified that it works...


MuJoCo Kinova Gen3 7-DoF Peg-in-Hole Simulation Environment.

Implements ``RobotBackend`` from ``schema.py`` for Step 1 (Monday handoff to
Graeme for PHANToM teleoperation and Matvey for logging/ACT).

Key design points
-----------------
1. **Joint-1 Movement & Stability Fixes**:
   - Explicitly indexes the 7 robot joints (``joint_1`` .. ``joint_7``) by name
     using ``jnt_qposadr``, ``jnt_dofadr``, and actuator ``trnid`` rather than
     hardcoded ``[:7]`` slices (prevents off-by-one or extra-body indexing bugs).
   - Adds base/shoulder contact exclusion and elevates the robot mount above
     the floor/table so ``base_link`` / ``shoulder_link`` never pin Joint 1.
   - Applies ``armature >= 0.1`` and ``damping >= 1.0`` and ensures continuous
     joints (``joint_1``, ``joint_3``, ``joint_5``, ``joint_7``) have usable
     position actuator bounds (addressing ``mujoco_menagerie`` Issue #232).
2. **6-DoF Cartesian -> 7-DoF Kinova Gen3 Control**:
   - Maps bounded 6-DoF Cartesian twists or pose increments (from PHANToM or
     learned policy) to 7-DoF joint targets using Damped Least-Squares (DLS)
     Jacobian IK with nullspace posture regularization toward ``q_home``.
   - Runs at the external control rate (default 50 Hz) while stepping MuJoCo
     physics at 500 Hz (0.002 s), with velocity/acceleration/workspace limits,
     command latency queue, and optional low-pass filtering.
3. **Contact-Rich Peg-in-Hole & Domain Randomization**:
   - Peg rigidly attached to the Gen3 tool flange + wrist-mounted camera
     (``wrist_cam``) with per-episode lighting randomization.
   - Octagonal primitive-box hole assembly with configurable clearance (starts
     forgiving at 3 mm radial clearance, tightenable for validation).
   - Deterministic ``reset(seed=...)``, randomized hole pose/friction/lighting,
     optional Kinova-like encoder quantization, and automated success/failure
     detection (insertion, collision, out-of-bounds, jamming, timeout).


"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import mujoco  # type: ignore
import numpy as np

from schema import (
    CartesianCommand,
    ContactState,
    DEFAULT_SIM_TIMESTEP,
    EpisodeConfig,
    EpisodeStatus,
    NUM_ARM_JOINTS,
    Observation,
    RobotBackend,
    StepResult,
)

# Conservative Kinova Gen3 7-DoF home posture (arm forward, tool pointing down)
DEFAULT_HOME_QPOS = np.array(
    [0.0, 0.35, 0.0, 1.55, 0.0, 1.24, 0.0], dtype=np.float64
)

# Physical Kinova Gen3 joint limits (continuous joints bounded to +/- 2*pi for position ctrl)
KINOVA_JOINT_LIMITS = np.array(
    [
        [-2.0 * np.pi, 2.0 * np.pi],   # joint_1 (continuous)
        [-2.24, 2.24],                 # joint_2 (+/- 128.9 deg)
        [-2.0 * np.pi, 2.0 * np.pi],   # joint_3 (continuous)
        [-2.57, 2.57],                 # joint_4 (+/- 147.8 deg)
        [-2.0 * np.pi, 2.0 * np.pi],   # joint_5 (continuous)
        [-2.09, 2.09],                 # joint_6 (+/- 120.3 deg)
        [-2.0 * np.pi, 2.0 * np.pi],   # joint_7 (continuous)
    ],
    dtype=np.float64,
)


def _quat_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _find_tool_body(root: ET.Element) -> ET.Element:
    """Locate the terminal link body (child of joint_7) in gen3.xml."""
    for body in root.iter("body"):
        for jnt in body.findall("joint"):
            if jnt.get("name") in ("joint_7", "actuator7"):
                return body
    # Fallback: last nested body in worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("gen3.xml is missing <worldbody>.")
    bodies = list(worldbody.iter("body"))
    if not bodies:
        raise ValueError("No <body> elements found in gen3.xml.")
    return bodies[-1]


def build_peg_in_hole_xml(
    kinova_dir: Path,
    peg_radius: float = 0.012,
    peg_length: float = 0.080,
    clearance_m: float = 0.003,
    hole_depth: float = 0.045,
    nominal_hole_pos: Tuple[float, float, float] = (0.48, 0.0, 0.05),
) -> str:
    """Build a self-contained MJCF XML string combining Kinova Gen3 + peg + hole.

    Resolves all mesh paths in ``gen3.xml`` to absolute paths so the model can
    be loaded from any working directory without file-path errors.
    """
    gen3_xml_path = (kinova_dir / "gen3.xml").resolve()
    if not gen3_xml_path.exists():
        raise FileNotFoundError(
            f"Kinova Gen3 XML not found at {gen3_xml_path}. "
            "Run `python3 setup_mujoco.py` first."
        )

    tree = ET.parse(gen3_xml_path)
    root = tree.getroot()
    root.set("model", "kinova_gen3_peg_in_hole")

    # 1. Configure compiler & physics options (resolve meshdir to absolute path)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    meshdir = compiler.get("meshdir", "assets")
    abs_meshdir = (kinova_dir / meshdir).resolve()
    compiler.set("meshdir", str(abs_meshdir))

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", f"{DEFAULT_SIM_TIMESTEP}")
    option.set("integrator", "implicitfast")
    option.set("cone", "elliptic")
    option.set("impratio", "10")

    # 2. Patch joint regularization & actuator bounds (fixes Joint 1 & low-inertia instability)
    for jnt in root.iter("joint"):
        jname = jnt.get("name", "")
        if jname.startswith("joint_"):
            if jnt.get("armature") is None:
                jnt.set("armature", "0.1")
            if jnt.get("damping") is None:
                jnt.set("damping", "1.0")

    actuator_elem = root.find("actuator")
    if actuator_elem is not None:
        for act in list(actuator_elem):
            jname = act.get("joint", "")
            if jname in ("joint_1", "joint_3", "joint_5", "joint_7"):
                # Ensure continuous joints have explicit wide ctrlrange so both
                # viewer sliders and position controllers move Joint 1 smoothly.
                act.set("ctrllimited", "true")
                act.set("ctrlrange", "-6.283185 6.283185")

    # 3. Ensure base_link <-> shoulder_link contact exclusion (prevents Joint 1 pinning)
    contact_elem = root.find("contact")
    if contact_elem is None:
        contact_elem = ET.SubElement(root, "contact")
    ET.SubElement(
        contact_elem,
        "exclude",
        {"body1": "base_link", "body2": "shoulder_link"},
    )

    # 4. Add visual assets (floor checker & materials)
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        {
            "name": "grid_tex",
            "type": "2d",
            "builtin": "checker",
            "rgb1": "0.88 0.89 0.91",
            "rgb2": "0.78 0.80 0.83",
            "width": "512",
            "height": "512",
        },
    )
    ET.SubElement(
        asset,
        "material",
        {"name": "grid_mat", "texture": "grid_tex", "texrepeat": "6 6", "reflectance": "0.1"},
    )
    ET.SubElement(
        asset,
        "material",
        {"name": "peg_mat", "rgba": "0.85 0.25 0.18 1.0", "specular": "0.4", "shininess": "0.3"},
    )
    ET.SubElement(
        asset,
        "material",
        {"name": "hole_mat", "rgba": "0.20 0.45 0.72 1.0", "specular": "0.3", "shininess": "0.2"},
    )
    ET.SubElement(
        asset,
        "material",
        {"name": "table_mat", "rgba": "0.62 0.56 0.48 1.0", "specular": "0.1"},
    )

    # 5. Attach peg + peg_tip site + wrist_cam to the terminal Kinova link
    tool_body = _find_tool_body(root)
    peg_mount = ET.SubElement(tool_body, "body", {"name": "peg_body", "pos": "0 0 0.06"})
    ET.SubElement(
        peg_mount,
        "geom",
        {
            "name": "peg_geom",
            "type": "cylinder",
            "size": f"{peg_radius:.5f} {peg_length / 2.0:.5f}",
            "pos": f"0 0 {peg_length / 2.0:.5f}",
            "material": "peg_mat",
            "mass": "0.12",
            "friction": "0.4 0.005 0.0001",
            "solref": "0.005 1.0",
            "solimp": "0.95 0.99 0.001",
            "condim": "4",
        },
    )
    ET.SubElement(
        peg_mount,
        "site",
        {
            "name": "peg_tip",
            "pos": f"0 0 {peg_length:.5f}",
            "size": "0.004",
            "rgba": "1 0.9 0 0.8",
        },
    )
    # Wrist-mounted webcam angled toward the peg tip and hole entry
    ET.SubElement(
        tool_body,
        "camera",
        {
            "name": "wrist_cam",
            "pos": "0.06 0.0 0.02",
            "xyaxes": "0 1 0 -0.25 0 0.97",
            "fovy": "58",
        },
    )

    # 6. Add worldbody environment: lights, floor, table, and octagonal hole block
    worldbody = root.find("worldbody")
    assert worldbody is not None

    # Main + fill lights (randomized per episode for visual robustness)
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "key_light",
            "pos": "0.6 -0.5 1.4",
            "dir": "-0.2 0.2 -1.0",
            "diffuse": "0.85 0.85 0.82",
            "specular": "0.3 0.3 0.3",
            "castshadow": "true",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "fill_light",
            "pos": "-0.4 0.6 1.2",
            "dir": "0.3 -0.3 -1.0",
            "diffuse": "0.35 0.35 0.40",
        },
    )

    # Floor plane slightly below z=0 so base_link sits cleanly on mount
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": "0 0 -0.02",
            "size": "2 2 0.05",
            "material": "grid_mat",
            "condim": "3",
        },
    )

    # Task table in front of the robot (kept clear of robot base_link at origin)
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "work_table",
            "type": "box",
            "pos": "0.52 0.0 0.025",
            "size": "0.26 0.36 0.025",
            "material": "table_mat",
            "condim": "3",
        },
    )

    # Hole assembly built from 8 primitive wall segments (fast, exact cylinder-like clearance)
    hx, hy, hz = nominal_hole_pos
    hole_body = ET.SubElement(
        worldbody,
        "body",
        {"name": "hole_body", "pos": f"{hx:.4f} {hy:.4f} {hz:.4f}"},
    )
    inner_r = peg_radius + clearance_m
    wall_thickness = 0.025
    wall_half_h = hole_depth / 2.0
    n_sides = 8
    # Side length for circumscribed regular octagon around inner_r
    half_seg_len = inner_r * np.tan(np.pi / n_sides) + wall_thickness * 0.45

    for i in range(n_sides):
        theta = i * (2.0 * np.pi / n_sides)
        r_mid = inner_r + wall_thickness / 2.0
        wx = r_mid * np.cos(theta)
        wy = r_mid * np.sin(theta)
        ET.SubElement(
            hole_body,
            "geom",
            {
                "name": f"hole_wall_{i}",
                "type": "box",
                "pos": f"{wx:.5f} {wy:.5f} {wall_half_h:.5f}",
                "euler": f"0 0 {theta:.5f}",
                "size": f"{wall_thickness / 2.0:.5f} {half_seg_len:.5f} {wall_half_h:.5f}",
                "material": "hole_mat",
                "friction": "0.4 0.005 0.0001",
                "solref": "0.005 1.0",
                "solimp": "0.95 0.99 0.001",
                "condim": "4",
            },
        )

    # Bottom plate of the hole
    ET.SubElement(
        hole_body,
        "geom",
        {
            "name": "hole_bottom_plate",
            "type": "cylinder",
            "pos": "0 0 0.004",
            "size": f"{inner_r + wall_thickness:.5f} 0.004",
            "material": "hole_mat",
            "condim": "3",
        },
    )
    ET.SubElement(
        hole_body,
        "site",
        {
            "name": "hole_entry",
            "pos": f"0 0 {hole_depth:.5f}",
            "size": "0.004",
            "rgba": "0.1 0.9 0.2 0.8",
        },
    )
    ET.SubElement(
        hole_body,
        "site",
        {
            "name": "hole_bottom",
            "pos": "0 0 0.008",
            "size": "0.004",
            "rgba": "0.1 0.5 1.0 0.8",
        },
    )

    # 7. Add force/torque sensor at peg_tip for contact/jamming detection
    sensor_elem = root.find("sensor")
    if sensor_elem is None:
        sensor_elem = ET.SubElement(root, "sensor")
    ET.SubElement(sensor_elem, "force", {"name": "peg_force", "site": "peg_tip"})
    ET.SubElement(sensor_elem, "torque", {"name": "peg_torque", "site": "peg_tip"})

    return ET.tostring(root, encoding="unicode")


class KinovaPegInHoleEnv(RobotBackend):
    """MuJoCo Kinova Gen3 7-DoF Peg-in-Hole environment."""

    def __init__(
        self,
        kinova_dir: Path | str | None = None,
        config: Optional[EpisodeConfig] = None,
        peg_radius: float = 0.012,
        peg_length: float = 0.080,
        hole_depth: float = 0.045,
        dls_damping: float = 0.04,
        nullspace_gain: float = 0.5,
    ) -> None:
        if kinova_dir is None:
            kinova_dir = Path(__file__).resolve().parent / "assets" / "mujoco_menagerie" / "kinova_gen3"
        self.kinova_dir = Path(kinova_dir)
        self.config = config or EpisodeConfig()
        self.peg_radius = peg_radius
        self.peg_length = peg_length
        self.hole_depth = hole_depth
        self.dls_damping = dls_damping
        self.nullspace_gain = nullspace_gain

        self._nominal_hole_pos = np.array([0.48, 0.0, 0.05], dtype=np.float64)
        self._rng = np.random.default_rng(self.config.seed)
        self._renderer: Any = None

        self._build_model(clearance_m=self.config.domain_randomization.clearance_mm_range[0] / 1000.0)
        self._init_episode_buffers()

    def _build_model(self, clearance_m: float) -> None:
        xml_str = build_peg_in_hole_xml(
            kinova_dir=self.kinova_dir,
            peg_radius=self.peg_radius,
            peg_length=self.peg_length,
            clearance_m=clearance_m,
            hole_depth=self.hole_depth,
            nominal_hole_pos=tuple(self._nominal_hole_pos),  # type: ignore[arg-type]
        )
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)

        # Resolve exact indices by name (never assume [:7]!)
        self.joint_names = [f"joint_{i}" for i in range(1, NUM_ARM_JOINTS + 1)]
        self.joint_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names],
            dtype=np.int32,
        )
        if np.any(self.joint_ids < 0):
            raise RuntimeError(f"Could not find all 7 Kinova joints: {self.joint_names}")

        self.qpos_ids = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids], dtype=np.int32)
        self.dof_ids = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids], dtype=np.int32)

        # Map each joint_1..joint_7 to its corresponding actuator index
        self.actuator_ids = np.full(NUM_ARM_JOINTS, -1, dtype=np.int32)
        for act_id in range(self.model.nu):
            trnid = self.model.actuator_trnid[act_id, 0]
            matches = np.where(self.joint_ids == trnid)[0]
            if len(matches) == 1:
                self.actuator_ids[matches[0]] = act_id
        if np.any(self.actuator_ids < 0):
            raise RuntimeError(f"Missing actuator mapping for joints: {self.actuator_ids}")

        # Site, body, geom, and sensor IDs
        self.peg_tip_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "peg_tip")
        self.hole_entry_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "hole_entry")
        self.hole_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hole_body")
        self.peg_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
        self.table_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "work_table")
        self.hole_geom_ids = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"hole_wall_{i}")
            for i in range(8)
        }
        self.hole_geom_ids.add(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "hole_bottom_plate")
        )
        self.force_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_force")
        self.torque_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_torque")
        self.key_light_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_LIGHT, "key_light")

        # Scratch arrays for Jacobian
        self._jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacr = np.zeros((3, self.model.nv), dtype=np.float64)

    def _init_episode_buffers(self) -> None:
        self._step_index = 0
        self._hold_steps = 0
        self._jam_steps = 0
        self._prev_twist = np.zeros(6, dtype=np.float64)
        self._filtered_twist = np.zeros(6, dtype=np.float64)
        self._cmd_queue: Deque[np.ndarray] = deque()
        self._q_target = DEFAULT_HOME_QPOS.copy()
        self.episode_params: Dict[str, Any] = {}

    def reset(
        self,
        seed: Optional[int] = None,
        config: Optional[EpisodeConfig] = None,
    ) -> Observation:
        if config is not None:
            self.config = config
        active_seed = self.config.seed if seed is None else seed
        self._rng = np.random.default_rng(active_seed)
        dr = self.config.domain_randomization

        # Rebuild model only if clearance is randomized across a non-degenerate range
        c_lo, c_hi = dr.clearance_mm_range
        clearance_m = (
            float(self._rng.uniform(c_lo, c_hi)) / 1000.0 if (dr.enabled and c_hi > c_lo) else c_lo / 1000.0
        )
        if dr.enabled and c_hi > c_lo:
            self._build_model(clearance_m=clearance_m)

        mujoco.mj_resetData(self.model, self.data)
        self._init_episode_buffers()

        # 1. Randomize hole pose on the work table
        hole_pos = self._nominal_hole_pos.copy()
        hole_yaw = 0.0
        if dr.enabled:
            hole_pos[0] += float(self._rng.uniform(-dr.hole_xy_range_m[0], dr.hole_xy_range_m[0]))
            hole_pos[1] += float(self._rng.uniform(-dr.hole_xy_range_m[1], dr.hole_xy_range_m[1]))
            hole_yaw = float(self._rng.uniform(-dr.hole_yaw_range_rad, dr.hole_yaw_range_rad))

        self.model.body_pos[self.hole_body_id] = hole_pos
        self.model.body_quat[self.hole_body_id] = np.array(
            [np.cos(hole_yaw / 2.0), 0.0, 0.0, np.sin(hole_yaw / 2.0)], dtype=np.float64
        )

        # 2. Randomize peg/hole contact friction
        friction = 0.4
        if dr.enabled:
            friction = float(self._rng.uniform(dr.friction_range[0], dr.friction_range[1]))
        self.model.geom_friction[self.peg_geom_id, 0] = friction
        for gid in self.hole_geom_ids:
            if gid >= 0:
                self.model.geom_friction[gid, 0] = friction

        # 3. Randomize key light position and intensity for wrist_cam robustness
        light_pos = np.array([0.6, -0.5, 1.4], dtype=np.float64)
        light_diffuse = np.array([0.85, 0.85, 0.82], dtype=np.float64)
        if dr.enabled and dr.randomize_lighting and self.key_light_id >= 0:
            light_pos += self._rng.uniform(-0.35, 0.35, size=3)
            light_pos[2] = max(0.8, light_pos[2])
            intensity = float(self._rng.uniform(0.55, 1.15))
            tint = self._rng.uniform(-0.08, 0.08, size=3)
            light_diffuse = np.clip(np.array([0.85, 0.85, 0.82]) * intensity + tint, 0.2, 1.2)
            self.model.light_pos[self.key_light_id] = light_pos
            self.model.light_diffuse[self.key_light_id] = light_diffuse

        # 4. Initialize arm near home and move peg tip to randomized start above hole
        self.data.qpos[self.qpos_ids] = DEFAULT_HOME_QPOS.copy()
        self.data.ctrl[self.actuator_ids] = DEFAULT_HOME_QPOS.copy()
        mujoco.mj_forward(self.model, self.data)

        start_target = hole_pos.copy()
        start_target[2] += self.hole_depth + float(
            self._rng.uniform(dr.peg_start_z_offset_m[0], dr.peg_start_z_offset_m[1])
            if dr.enabled
            else np.mean(dr.peg_start_z_offset_m)
        )
        if dr.enabled:
            start_target[0] += float(
                self._rng.uniform(-dr.peg_start_xy_offset_m[0], dr.peg_start_xy_offset_m[0])
            )
            start_target[1] += float(
                self._rng.uniform(-dr.peg_start_xy_offset_m[1], dr.peg_start_xy_offset_m[1])
            )

        # Solve a few damped IK iterations to place the peg at the randomized start
        self._settle_to_start_position(start_target)

        # 5. Latency queue initialization
        lat_lo, lat_hi = dr.latency_steps_range
        latency_steps = (
            int(self._rng.integers(lat_lo, lat_hi + 1))
            if (dr.enabled and lat_hi > lat_lo)
            else self.config.control_limits.latency_steps
        )
        self._cmd_queue = deque([np.zeros(6, dtype=np.float64)] * latency_steps)

        self.episode_params = {
            "seed": active_seed,
            "hole_pos": hole_pos.tolist(),
            "hole_yaw_rad": hole_yaw,
            "start_target_pos": start_target.tolist(),
            "friction": friction,
            "clearance_m": clearance_m,
            "latency_steps": latency_steps,
            "light_pos": light_pos.tolist(),
            "light_diffuse": light_diffuse.tolist(),
        }
        return self.get_observation()

    def _settle_to_start_position(self, target_pos: np.ndarray, iters: int = 40) -> None:
        """Use kinematic DLS IK to initialize the peg tip cleanly at start_target."""
        for _ in range(iters):
            mujoco.mj_forward(self.model, self.data)
            cur_pos = self.data.site_xpos[self.peg_tip_site_id].copy()
            pos_err = target_pos - cur_pos
            if np.linalg.norm(pos_err) < 1e-3:
                break
            mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
            Jp = self._jacp[:, self.dof_ids]
            jjt = Jp @ Jp.T + (self.dls_damping**2) * np.eye(3)
            dq = Jp.T @ np.linalg.solve(jjt, 0.5 * pos_err)
            q_new = np.clip(
                self.data.qpos[self.qpos_ids] + dq,
                KINOVA_JOINT_LIMITS[:, 0],
                KINOVA_JOINT_LIMITS[:, 1],
            )
            self.data.qpos[self.qpos_ids] = q_new
            self.data.qvel[self.dof_ids] = 0.0
        self._q_target = self.data.qpos[self.qpos_ids].copy()
        self.data.ctrl[self.actuator_ids] = self._q_target.copy()
        mujoco.mj_forward(self.model, self.data)

    def _apply_control_constraints(self, command: CartesianCommand) -> np.ndarray:
        """Convert command to bounded, filtered, delayed 6-DoF twist."""
        limits = self.config.control_limits
        dt = limits.control_dt

        if not command.clutch_engaged:
            raw_twist = np.zeros(6, dtype=np.float64)
        elif command.mode == "delta_pose":
            raw_twist = command.as_vector() / dt
        else:
            raw_twist = command.as_vector().copy()

        # Deadband
        if limits.deadband_linear > 0.0 and np.linalg.norm(raw_twist[:3]) < limits.deadband_linear:
            raw_twist[:3] = 0.0
        if limits.deadband_angular > 0.0 and np.linalg.norm(raw_twist[3:]) < limits.deadband_angular:
            raw_twist[3:] = 0.0

        # Command latency buffer
        if len(self._cmd_queue) > 0:
            self._cmd_queue.append(raw_twist)
            delayed_twist = self._cmd_queue.popleft()
        else:
            delayed_twist = raw_twist

        # Exponential low-pass filter
        alpha = float(np.clip(limits.lpf_alpha, 0.01, 1.0))
        self._filtered_twist = alpha * delayed_twist + (1.0 - alpha) * self._filtered_twist
        twist = self._filtered_twist.copy()

        # Velocity norm bounds
        lin_norm = float(np.linalg.norm(twist[:3]))
        if lin_norm > limits.max_linear_vel:
            twist[:3] *= limits.max_linear_vel / lin_norm
        ang_norm = float(np.linalg.norm(twist[3:]))
        if ang_norm > limits.max_angular_vel:
            twist[3:] *= limits.max_angular_vel / ang_norm

        # Acceleration bounds
        d_lin = twist[:3] - self._prev_twist[:3]
        max_d_lin = limits.max_linear_acc * dt
        d_lin_norm = float(np.linalg.norm(d_lin))
        if d_lin_norm > max_d_lin:
            twist[:3] = self._prev_twist[:3] + d_lin * (max_d_lin / d_lin_norm)

        d_ang = twist[3:] - self._prev_twist[3:]
        max_d_ang = limits.max_angular_acc * dt
        d_ang_norm = float(np.linalg.norm(d_ang))
        if d_ang_norm > max_d_ang:
            twist[3:] = self._prev_twist[3:] + d_ang * (max_d_ang / d_ang_norm)

        # Workspace soft-wall clamp (prevent pushing further outside workspace box)
        ee_pos = self.data.site_xpos[self.peg_tip_site_id]
        ws_min = np.asarray(limits.workspace_min, dtype=np.float64)
        ws_max = np.asarray(limits.workspace_max, dtype=np.float64)
        for axis in range(3):
            if ee_pos[axis] <= ws_min[axis] and twist[axis] < 0.0:
                twist[axis] = 0.0
            if ee_pos[axis] >= ws_max[axis] and twist[axis] > 0.0:
                twist[axis] = 0.0

        self._prev_twist = twist.copy()
        return twist

    def _solve_dls_ik_velocity(self, twist: np.ndarray) -> np.ndarray:
        """Map 6-DoF Cartesian twist to 7-DoF joint velocities via DLS + nullspace."""
        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
        J = np.vstack([self._jacp[:, self.dof_ids], self._jacr[:, self.dof_ids]])  # (6, 7)

        jjt = J @ J.T + (self.dls_damping**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(jjt)  # (7, 6)

        q_cur = self.data.qpos[self.qpos_ids]
        null_proj = np.eye(NUM_ARM_JOINTS) - J_pinv @ J
        q_dot_null = self.nullspace_gain * (DEFAULT_HOME_QPOS - q_cur)

        q_dot = J_pinv @ twist + null_proj @ q_dot_null
        max_jv = self.config.control_limits.max_joint_vel
        return np.clip(q_dot, -max_jv, max_jv)

    def step(self, command: CartesianCommand) -> StepResult:
        limits = self.config.control_limits
        dt = limits.control_dt
        n_substeps = max(1, int(round(dt / self.model.opt.timestep)))

        executed_twist = self._apply_control_constraints(command)
        q_dot = self._solve_dls_ik_velocity(executed_twist)

        self._q_target = np.clip(
            self._q_target + q_dot * dt,
            KINOVA_JOINT_LIMITS[:, 0],
            KINOVA_JOINT_LIMITS[:, 1],
        )
        self.data.ctrl[self.actuator_ids] = self._q_target

        for _ in range(n_substeps):
            mujoco.mj_step(self.model, self.data)

        self._step_index += 1
        obs = self.get_observation()
        status, eval_info = self._evaluate_status(obs)
        done = status != EpisodeStatus.RUNNING
        eval_info["episode_params"] = self.episode_params

        return StepResult(
            observation=obs,
            executed_twist=executed_twist,
            status=status,
            done=done,
            info=eval_info,
        )

    def _read_contact_state(self) -> ContactState:
        num_contacts = int(self.data.ncon)
        peg_hole_contact = False
        table_collision = False
        total_force = 0.0
        c_force = np.zeros(6, dtype=np.float64)

        for i in range(num_contacts):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            pair = {g1, g2}
            if self.peg_geom_id in pair and (g1 in self.hole_geom_ids or g2 in self.hole_geom_ids):
                peg_hole_contact = True
            if self.table_geom_id in pair and self.peg_geom_id in pair:
                table_collision = True
            mujoco.mj_contactForce(self.model, self.data, i, c_force)
            total_force += float(np.linalg.norm(c_force[:3]))

        # Read wrist/peg tip wrench sensor
        wrench = np.zeros(6, dtype=np.float64)
        if self.force_sensor_id >= 0:
            adr = self.model.sensor_adr[self.force_sensor_id]
            wrench[:3] = self.data.sensordata[adr : adr + 3]
        if self.torque_sensor_id >= 0:
            adr = self.model.sensor_adr[self.torque_sensor_id]
            wrench[3:] = self.data.sensordata[adr : adr + 3]

        crit = self.config.success_criteria
        ee_speed = float(np.linalg.norm(self._prev_twist[:3]))
        if peg_hole_contact and total_force >= crit.jam_force_threshold_n and ee_speed < 0.005:
            self._jam_steps += 1
        else:
            self._jam_steps = max(0, self._jam_steps - 1)

        return ContactState(
            in_contact=num_contacts > 0,
            num_contacts=num_contacts,
            peg_hole_contact=peg_hole_contact,
            table_collision=table_collision,
            total_contact_force_n=total_force,
            wrench_ee=wrench,
            is_jammed=self._jam_steps >= crit.jam_steps_threshold,
        )

    def _apply_sensor_realism(
        self, joint_pos: np.ndarray, ee_pos: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        sr = self.config.sensor_realism
        q_out = joint_pos.copy()
        p_out = ee_pos.copy()

        if sr.joint_pos_noise_std_rad > 0.0:
            q_out += self._rng.normal(0.0, sr.joint_pos_noise_std_rad, size=q_out.shape)
        if sr.encoder_quantization_rad is not None and sr.encoder_quantization_rad > 0.0:
            step = sr.encoder_quantization_rad
            q_out = np.round(q_out / step) * step

        if sr.ee_pos_noise_std_m > 0.0:
            p_out += self._rng.normal(0.0, sr.ee_pos_noise_std_m, size=p_out.shape)
        if sr.cartesian_round_decimals is not None:
            p_out = np.round(p_out, sr.cartesian_round_decimals)

        return q_out, p_out

    def render_wrist_camera(self) -> np.ndarray:
        """Render RGB frame from the wrist-mounted simulated webcam."""
        sr = self.config.sensor_realism
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=sr.camera_height, width=sr.camera_width
            )
        self._renderer.update_scene(self.data, camera="wrist_cam")
        return self._renderer.render()

    def get_observation(self) -> Observation:
        mujoco.mj_forward(self.model, self.data)

        raw_qpos = self.data.qpos[self.qpos_ids].copy()
        raw_qvel = self.data.qvel[self.dof_ids].copy()
        raw_qtrq = self.data.qfrc_actuator[self.dof_ids].copy()

        raw_ee_pos = self.data.site_xpos[self.peg_tip_site_id].copy()
        ee_xmat = self.data.site_xmat[self.peg_tip_site_id].reshape(3, 3)
        ee_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(ee_quat, ee_xmat.flatten())

        # End-effector spatial velocity via Jacobian * qvel
        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
        ee_v_lin = self._jacp[:, self.dof_ids] @ raw_qvel
        ee_v_ang = self._jacr[:, self.dof_ids] @ raw_qvel

        hole_pos = self.data.site_xpos[self.hole_entry_site_id].copy()
        hole_xmat = self.data.site_xmat[self.hole_entry_site_id].reshape(3, 3)
        hole_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(hole_quat, hole_xmat.flatten())

        # Peg tip expressed in hole_entry frame: +z is above hole, negative z is inserted
        rel_hole = hole_xmat.T @ (raw_ee_pos - hole_pos)
        peg_to_hole_local = np.array([rel_hole[0], rel_hole[1], -rel_hole[2]], dtype=np.float64)

        obs_qpos, obs_ee_pos = self._apply_sensor_realism(raw_qpos, raw_ee_pos)
        contact = self._read_contact_state()

        wrist_rgb = (
            self.render_wrist_camera() if self.config.sensor_realism.render_wrist_camera else None
        )

        return Observation(
            timestamp=float(self.data.time),
            step_index=self._step_index,
            ee_pos=obs_ee_pos,
            ee_quat=ee_quat,
            ee_vel_linear=ee_v_lin,
            ee_vel_angular=ee_v_ang,
            joint_pos=obs_qpos,
            joint_vel=raw_qvel,
            joint_torque=raw_qtrq,
            peg_pos=obs_ee_pos.copy(),
            peg_quat=ee_quat.copy(),
            hole_pos=hole_pos,
            hole_quat=hole_quat,
            peg_to_hole_local=peg_to_hole_local,
            contact=contact,
            wrist_rgb=wrist_rgb,
        )

    def _evaluate_status(self, obs: Observation) -> Tuple[EpisodeStatus, Dict[str, Any]]:
        crit = self.config.success_criteria
        lateral_err = float(np.linalg.norm(obs.peg_to_hole_local[:2]))
        insertion_depth = float(obs.peg_to_hole_local[2])

        # Tilt angle between peg Z-axis and downward hole axis (-Z in hole frame)
        peg_z = _quat_to_rotmat(obs.peg_quat)[:, 2]
        hole_z = _quat_to_rotmat(obs.hole_quat)[:, 2]
        cos_tilt = float(np.clip(abs(np.dot(peg_z, hole_z)), -1.0, 1.0))
        tilt_err = float(np.arccos(cos_tilt))

        info = {
            "lateral_error_m": lateral_err,
            "insertion_depth_m": insertion_depth,
            "tilt_error_rad": tilt_err,
            "hold_steps": self._hold_steps,
        }

        # Success check
        if (
            insertion_depth >= crit.insertion_depth_m
            and lateral_err <= crit.max_lateral_error_m
            and tilt_err <= crit.max_tilt_error_rad
        ):
            self._hold_steps += 1
            if self._hold_steps >= crit.hold_steps_required:
                return EpisodeStatus.SUCCESS, info
        else:
            self._hold_steps = 0

        # Failure checks
        if obs.contact.total_contact_force_n > crit.max_contact_force_n:
            return EpisodeStatus.FAILURE_COLLISION, info
        if obs.contact.is_jammed:
            return EpisodeStatus.FAILURE_JAMMED, info

        ws_min = np.asarray(self.config.control_limits.workspace_min) - 0.05
        ws_max = np.asarray(self.config.control_limits.workspace_max) + 0.05
        if np.any(obs.ee_pos < ws_min) or np.any(obs.ee_pos > ws_max):
            return EpisodeStatus.FAILURE_OUT_OF_BOUNDS, info

        if self._step_index >= crit.max_episode_steps:
            return EpisodeStatus.FAILURE_TIMEOUT, info

        return EpisodeStatus.RUNNING, info

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None