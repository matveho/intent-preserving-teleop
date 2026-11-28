"""MuJoCo Kinova Gen3 7-DoF Peg-in-Hole Simulation Environment.

Implements ``RobotBackend`` from ``schema.py`` for Step 1 (handoff to
Graeme for PHANToM teleoperation and Matvey for logging/ACT).

Key design points
-----------------
1. **Joint-1 Movement & Stability Fixes**:
   - Explicitly indexes the 7 robot joints (``joint_1`` .. ``joint_7``) by name
     using ``jnt_qposadr``, ``jnt_dofadr``, and actuator ``trnid`` rather than
     hardcoded ``[:7]`` slices.
   - Excludes ``base_link``/``shoulder_link`` and ``peg_body``/wrist-link
     self-contacts.
   - Applies ``armature >= 0.1`` and ``damping >= 1.0`` and sets explicit
     ``ctrlrange`` bounds on continuous joints (``joint_1``, ``joint_3``,
     ``joint_5``, ``joint_7``).
2. **Peg mounted on the rotating end link (joint_7)**:
   - ``peg_body`` is a child of the body that owns ``joint_7`` (the bracelet),
     so every one of the 7 joints moves the peg. The peg axis is built
     explicitly (outward along the flange axis) and verified at model build
     time; a RuntimeError is raised if it is parented wrongly or points back
     into the wrist.
3. **6-DoF Cartesian -> 7-DoF Kinova Gen3 Control**:
   - Damped Least-Squares (DLS) Jacobian IK with nullspace posture
     regularization toward ``DEFAULT_HOME_QPOS`` (this is where the 7th DoF
     is used).
   - Runs at the external control rate while stepping MuJoCo physics at 500 Hz.
4. **Single hole by default, optional second hole**:
   - Default: one vertical hole in the table (``"top"``); no extra geometry.
   - ``enable_second_hole=True`` adds a second target, ``second_hole_layout``:
     ``"table"`` (same orientation, 15 cm to the right; decoy for
     shared-autonomy demos) or ``"wall"`` (horizontal hole in a vertical wall,
     perpendicular to the top hole).
   - Select with ``reset(hole="top" | "side" | "random")``.
5. **Domain Randomization**: hole pose/friction/lighting/clearance, latency,
   optional encoder quantization, automated success/failure detection.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import mujoco  # type: ignore
import numpy as np

try:
    from intent_preserving_teleop.schema import (
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
    from intent_preserving_teleop.domain_randomization import resolve_insertion_layout
except ImportError:
    from schema import (  # type: ignore
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
    from domain_randomization import resolve_insertion_layout  # type: ignore

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

# Nominal hole placements. Each hole's local +Z is its opening/entry axis.
# Top hole: opening faces +Z (up). Peg approaches pointing -Z.
TOP_HOLE_NOMINAL_POS = (0.48, 0.0, 0.05)
TOP_HOLE_BASE_QUAT = (1.0, 0.0, 0.0, 0.0)  # [w, x, y, z]

# Side hole: sits on the +Y face of a vertical wall at y ~ -0.29 (robot's right
# when facing +X). Rotating -90 deg about X sends local +Z -> world +Y, so the
# opening faces the robot and the peg approaches pointing -Y.
SIDE_WALL_THICKNESS_HALF = 0.010
SIDE_HOLE_NOMINAL_POS = (0.50, -0.28, 0.20)
SIDE_HOLE_BASE_QUAT = (float(np.sqrt(0.5)), -float(np.sqrt(0.5)), 0.0, 0.0)
SIDE_WALL_HALF_EXTENTS = (0.16, SIDE_WALL_THICKNESS_HALF, 0.15)

# Optional second hole (OFF by default; enable with enable_second_hole=True).
#   "table": same vertical orientation as the top hole, 15 cm to its right (-Y).
#            A same-orientation decoy target: no tool re-orientation needed, so
#            the only difference is *which hole the operator intends*. Best for
#            shared-autonomy demos.
#   "wall":  horizontal hole in a vertical wall (perpendicular to the top hole).
SECOND_HOLE_LAYOUTS = {
    "table": {"pos": (0.48, -0.15, 0.05), "quat": TOP_HOLE_BASE_QUAT, "wall": False},
    "wall": {"pos": SIDE_HOLE_NOMINAL_POS, "quat": SIDE_HOLE_BASE_QUAT, "wall": True},
}

HOLE_KEYS = ("top", "side")


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


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a * b for [w, x, y, z] quaternions."""
    out = np.zeros(4, dtype=np.float64)
    mujoco.mju_mulQuat(out, np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64))
    return out


def _hole_quat(base_quat: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Base hole orientation followed by a yaw about the hole's own entry axis."""
    q_yaw = np.array([np.cos(yaw_rad / 2.0), 0.0, 0.0, np.sin(yaw_rad / 2.0)], dtype=np.float64)
    return _quat_mul(base_quat, q_yaw)


def _find_tool_body(root: ET.Element) -> ET.Element:
    """Locate the terminal link body (the body that owns joint_7) in gen3.xml."""
    for body in root.iter("body"):
        for jnt in body.findall("joint"):
            if jnt.get("name") in ("joint_7", "actuator7"):
                return body
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("gen3.xml is missing <worldbody>.")
    bodies = list(worldbody.iter("body"))
    if not bodies:
        raise ValueError("No <body> elements found in gen3.xml.")
    return bodies[-1]


def _detect_flange_transform(tool_body: ET.Element) -> Tuple[str, float]:
    """Return (flange position string, outward-axis sign along the body's Z).

    The site's own ``quat`` is deliberately ignored: applying it to the peg
    mount *and* flipping the peg along Z can double-flip the peg so it points
    back into the wrist. Instead the peg axis is built explicitly in
    ``build_peg_in_hole_xml`` and verified in ``_build_model``.
    """
    for site in tool_body.findall("site"):
        pos_str = site.get("pos", "0 0 -0.0615")
        vals = [float(v) for v in pos_str.split()]
        z_sign = -1.0 if len(vals) >= 3 and vals[2] < 0 else 1.0
        return pos_str, z_sign
    # Default for Kinova Gen3 bracelet_link (-Z outward flange)
    return "0 0 -0.0615", -1.0


def _add_hole_assembly(
    parent: ET.Element,
    prefix: str,
    pos: Tuple[float, float, float],
    quat_wxyz: Tuple[float, float, float, float],
    peg_radius: float,
    clearance_m: float,
    hole_depth: float,
) -> None:
    """Add an octagonal primitive-box hole (body + walls + bottom + sites).

    Names produced: ``{prefix}_body``, ``{prefix}_wall_{i}``,
    ``{prefix}_bottom_plate``, ``{prefix}_entry``, ``{prefix}_bottom``.
    The hole's local +Z is the entry axis.
    """
    hx, hy, hz = pos
    qw, qx, qy, qz = quat_wxyz
    hole_body = ET.SubElement(
        parent,
        "body",
        {
            "name": f"{prefix}_body",
            "pos": f"{hx:.4f} {hy:.4f} {hz:.4f}",
            "quat": f"{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}",
        },
    )
    inner_r = peg_radius + clearance_m
    wall_thickness = 0.025
    wall_half_h = hole_depth / 2.0
    n_sides = 8
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
                "name": f"{prefix}_wall_{i}",
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

    ET.SubElement(
        hole_body,
        "geom",
        {
            "name": f"{prefix}_bottom_plate",
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
            "name": f"{prefix}_entry",
            "pos": f"0 0 {hole_depth:.5f}",
            "size": "0.004",
            "rgba": "0.1 0.9 0.2 0.8",
        },
    )
    ET.SubElement(
        hole_body,
        "site",
        {
            "name": f"{prefix}_bottom",
            "pos": "0 0 0.008",
            "size": "0.004",
            "rgba": "0.1 0.5 1.0 0.8",
        },
    )


def build_peg_in_hole_xml(
    kinova_dir: Path,
    peg_radius: float = 0.012,
    peg_length: float = 0.080,
    clearance_m: float = 0.003,
    hole_depth: float = 0.045,
    nominal_hole_pos: Tuple[float, float, float] = TOP_HOLE_NOMINAL_POS,
    second_hole_layout: Optional[str] = None,
    side_hole_pos: Optional[Tuple[float, float, float]] = None,
) -> str:
    """Build a self-contained MJCF XML string: Kinova Gen3 + peg + top hole.

    ``second_hole_layout`` (None | "table" | "wall") optionally adds a second
    hole. None (default) leaves the scene with a single hole and no extra geometry.
    """
    gen3_xml_path = (kinova_dir / "gen3.xml").resolve()
    if not gen3_xml_path.exists():
        raise FileNotFoundError(
            f"Kinova Gen3 XML not found at {gen3_xml_path}. "
            "Run `python scripts/setup_mujoco.py` first."
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

    # 2. Patch joint regularization & actuator bounds
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
                act.set("ctrllimited", "true")
                act.set("ctrlrange", "-6.283185 6.283185")

    # 3. Collect existing robot body names to exclude peg <-> wrist self-contacts
    existing_bodies = {b.get("name", "") for b in root.iter("body")}
    contact_elem = root.find("contact")
    if contact_elem is None:
        contact_elem = ET.SubElement(root, "contact")

    for b1, b2 in [
        ("base_link", "shoulder_link"),
        ("peg_body", "bracelet_link"),
        ("peg_body", "spherical_wrist_2_link"),
        ("peg_body", "spherical_wrist_1_link"),
        ("peg_body", "forearm_link"),
    ]:
        if (b1 == "peg_body" or b1 in existing_bodies) and (
            b2 == "peg_body" or b2 in existing_bodies
        ):
            ET.SubElement(contact_elem, "exclude", {"body1": b1, "body2": b2})

    # 4. Add visual assets
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
    ET.SubElement(
        asset,
        "material",
        {"name": "wall_mat", "rgba": "0.55 0.58 0.62 1.0", "specular": "0.1"},
    )

    # 5. Mount the peg on the body that owns joint_7 (the rotating end link),
    #    extending along the flange's outward axis.
    #
    #    peg_mount's local +Z is the peg's outward axis. If the flange points
    #    along the tool body's -Z, rotate the mount 180 deg about X so local
    #    +Z == body -Z. Everything inside peg_mount then extends along +Z, and
    #    the ``peg_tip`` site frame's Z axis points out of the peg tip.
    tool_body = _find_tool_body(root)
    flange_pos, z_sign = _detect_flange_transform(tool_body)
    peg_attrs: Dict[str, str] = {"name": "peg_body", "pos": flange_pos}
    if z_sign < 0:
        peg_attrs["quat"] = "0 1 0 0"
    peg_mount = ET.SubElement(tool_body, "body", peg_attrs)

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
    ET.SubElement(
        tool_body,
        "camera",
        {
            "name": "wrist_cam",
            "pos": f"0.06 0.0 {z_sign * 0.02:.4f}",
            "xyaxes": "0 1 0 -0.25 0 0.97" if z_sign < 0 else "0 1 0 0.25 0 -0.97",
            "fovy": "58",
        },
    )

    # 6. Worldbody environment: lights, floor, table
    worldbody = root.find("worldbody")
    assert worldbody is not None

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

    # 7. Top hole (vertical, in the table)
    _add_hole_assembly(
        worldbody,
        prefix="hole",
        pos=nominal_hole_pos,
        quat_wxyz=TOP_HOLE_BASE_QUAT,
        peg_radius=peg_radius,
        clearance_m=clearance_m,
        hole_depth=hole_depth,
    )

    # 8. Optional second hole (off by default).
    if second_hole_layout is not None:
        if second_hole_layout not in SECOND_HOLE_LAYOUTS:
            raise ValueError(
                f"second_hole_layout must be one of {tuple(SECOND_HOLE_LAYOUTS)}, "
                f"got {second_hole_layout!r}"
            )
        layout = SECOND_HOLE_LAYOUTS[second_hole_layout]
        sx, sy, sz = side_hole_pos if side_hole_pos is not None else layout["pos"]
        if layout["wall"]:
            # Vertical plate standing on the table; its +Y face coincides with
            # the hole body origin plane.
            wall_hx, wall_hy, wall_hz = SIDE_WALL_HALF_EXTENTS
            ET.SubElement(
                worldbody,
                "geom",
                {
                    "name": "side_wall",
                    "type": "box",
                    "pos": f"{sx:.4f} {sy - wall_hy:.4f} {sz:.4f}",
                    "size": f"{wall_hx:.4f} {wall_hy:.4f} {wall_hz:.4f}",
                    "material": "wall_mat",
                    "condim": "3",
                },
            )
        _add_hole_assembly(
            worldbody,
            prefix="side_hole",
            pos=(sx, sy, sz),
            quat_wxyz=layout["quat"],
            peg_radius=peg_radius,
            clearance_m=clearance_m,
            hole_depth=hole_depth,
        )

    sensor_elem = root.find("sensor")
    if sensor_elem is None:
        sensor_elem = ET.SubElement(root, "sensor")
    ET.SubElement(sensor_elem, "force", {"name": "peg_force", "site": "peg_tip"})
    ET.SubElement(sensor_elem, "torque", {"name": "peg_torque", "site": "peg_tip"})

    return ET.tostring(root, encoding="unicode")


class KinovaPegInHoleEnv(RobotBackend):
    """MuJoCo Kinova Gen3 7-DoF Peg-in-Hole environment (top + side holes)."""

    def __init__(
        self,
        kinova_dir: Path | str | None = None,
        config: Optional[EpisodeConfig] = None,
        peg_radius: float = 0.012,
        peg_length: float = 0.080,
        hole_depth: float = 0.045,
        dls_damping: float = 0.04,
        nullspace_gain: float = 0.5,
        hole: Optional[str] = None,
        enable_second_hole: bool = False,
        second_hole_layout: str = "table",
    ) -> None:
        self.config = config or EpisodeConfig()
        configured_layout = self.config.domain_randomization.insertion_layout
        if configured_layout not in ("top", "horizontal", "random"):
            raise ValueError(
                "domain_randomization.insertion_layout must be 'top', "
                f"'horizontal', or 'random'; got {configured_layout!r}"
            )
        requested_layout = hole if hole in ("top", "horizontal", "random") else configured_layout
        # Horizontal/random layouts need the existing vertical-wall fixture.
        if requested_layout in ("horizontal", "random"):
            enable_second_hole = True
            if second_hole_layout == "table":
                second_hole_layout = "wall"
        if second_hole_layout not in SECOND_HOLE_LAYOUTS:
            raise ValueError(
                f"second_hole_layout must be one of {tuple(SECOND_HOLE_LAYOUTS)}, "
                f"got {second_hole_layout!r}"
            )
        valid_holes = HOLE_KEYS + ("horizontal", "random") if enable_second_hole else ("top",)
        if hole is not None and hole not in valid_holes:
            raise ValueError(
                f"hole must be one of {valid_holes}, got {hole!r} "
                f"(enable_second_hole={enable_second_hole})"
            )
        self.enable_second_hole = enable_second_hole
        self.second_hole_layout = second_hole_layout

        if kinova_dir is None:
            # Prefer the shared repository asset location. This is independent
            # of the shell's working directory (scripts/, repo root, etc.).
            here = Path(__file__).resolve()
            candidates = [
                here.parents[2] / "assets" / "mujoco_menagerie" / "kinova_gen3",
                here.parents[2] / "scripts" / "assets" / "mujoco_menagerie" / "kinova_gen3",
            ]
            kinova_dir = next((p for p in candidates if (p / "gen3.xml").exists()), candidates[0])
        self.kinova_dir = Path(kinova_dir)
        self.peg_radius = peg_radius
        self.peg_length = peg_length
        self.hole_depth = hole_depth
        self.dls_damping = dls_damping
        self.nullspace_gain = nullspace_gain

        self._default_hole = hole if hole is not None else requested_layout
        self.active_hole = "top" if self._default_hole in ("random", "horizontal") else self._default_hole

        self._nominal_hole_pos = np.array(TOP_HOLE_NOMINAL_POS, dtype=np.float64)
        self._nominal_side_hole_pos = np.array(
            SECOND_HOLE_LAYOUTS[second_hole_layout]["pos"], dtype=np.float64
        )
        self._rng = np.random.default_rng(self.config.seed)
        self._renderer: Any = None
        self.xml_str: str = ""

        self._build_model(clearance_m=self.config.domain_randomization.clearance_mm_range[0] / 1000.0)
        self._init_episode_buffers()

    def _build_model(self, clearance_m: float) -> None:
        self.xml_str = build_peg_in_hole_xml(
            kinova_dir=self.kinova_dir,
            peg_radius=self.peg_radius,
            peg_length=self.peg_length,
            clearance_m=clearance_m,
            hole_depth=self.hole_depth,
            nominal_hole_pos=tuple(self._nominal_hole_pos),  # type: ignore[arg-type]
            second_hole_layout=self.second_hole_layout if self.enable_second_hole else None,
            side_hole_pos=tuple(self._nominal_side_hole_pos),  # type: ignore[arg-type]
        )
        self.model = mujoco.MjModel.from_xml_string(self.xml_str)
        self.data = mujoco.MjData(self.model)

        self.joint_names = [f"joint_{i}" for i in range(1, NUM_ARM_JOINTS + 1)]
        self.joint_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names],
            dtype=np.int32,
        )
        if np.any(self.joint_ids < 0):
            raise RuntimeError(f"Could not find all 7 Kinova joints: {self.joint_names}")

        self.qpos_ids = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids], dtype=np.int32)
        self.dof_ids = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids], dtype=np.int32)

        self.actuator_ids = np.full(NUM_ARM_JOINTS, -1, dtype=np.int32)
        for act_id in range(self.model.nu):
            trnid = self.model.actuator_trnid[act_id, 0]
            matches = np.where(self.joint_ids == trnid)[0]
            if len(matches) == 1:
                self.actuator_ids[matches[0]] = act_id
        if np.any(self.actuator_ids < 0):
            raise RuntimeError(f"Missing actuator mapping for joints: {self.actuator_ids}")

        self.peg_tip_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "peg_tip")
        self.peg_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
        self.table_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "work_table")
        self.side_wall_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "side_wall")
        self.obstacle_geom_ids = {g for g in (self.table_geom_id, self.side_wall_geom_id) if g >= 0}

        # Per-hole bookkeeping (top: prefix "hole", side: prefix "side_hole")
        specs = {
            "top": ("hole", self._nominal_hole_pos, np.array(TOP_HOLE_BASE_QUAT, dtype=np.float64)),
        }
        if self.enable_second_hole:
            specs["side"] = (
                "side_hole",
                self._nominal_side_hole_pos,
                np.array(SECOND_HOLE_LAYOUTS[self.second_hole_layout]["quat"], dtype=np.float64),
            )
        self._holes: Dict[str, Dict[str, Any]] = {}
        self.hole_geom_ids: set = set()
        for key, (prefix, nominal_pos, base_quat) in specs.items():
            gids = {
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{prefix}_wall_{i}")
                for i in range(8)
            }
            gids.add(
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{prefix}_bottom_plate")
            )
            self._holes[key] = {
                "body_id": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_body"),
                "entry_site_id": mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}_entry"
                ),
                "geom_ids": gids,
                "nominal_pos": nominal_pos.copy(),
                "base_quat": base_quat,
            }
            self.hole_geom_ids |= gids
        self._select_hole(self.active_hole)

        self.force_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_force")
        self.torque_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_torque")
        self.key_light_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_LIGHT, "key_light")

        self._jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacr = np.zeros((3, self.model.nv), dtype=np.float64)

        self._verify_peg_mount()

    def _select_hole(self, key: str) -> None:
        """Make ``key`` ('top' or 'side') the hole used for observations/success."""
        if key not in self._holes:
            raise ValueError(f"Unknown hole {key!r}; expected one of {tuple(self._holes)}")
        self.active_hole = key
        self.hole_body_id = self._holes[key]["body_id"]
        self.hole_entry_site_id = self._holes[key]["entry_site_id"]

    def _verify_peg_mount(self) -> None:
        """Check the peg rides on joint_7's body and points outward, not into the wrist."""
        peg_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "peg_body")
        tool_body_id = int(self.model.jnt_bodyid[self.joint_ids[-1]])
        if int(self.model.body_parentid[peg_body_id]) != tool_body_id:
            raise RuntimeError(
                "peg_body is not a child of the body that owns joint_7; "
                "the peg would not use all 7 DoF."
            )

        # Outward check at the home posture: peg tip must lie beyond the tool
        # body origin along the direction from the previous link to the tool body.
        self.data.qpos[self.qpos_ids] = DEFAULT_HOME_QPOS
        mujoco.mj_forward(self.model, self.data)
        parent_id = int(self.model.body_parentid[tool_body_id])
        tool_pos = self.data.xpos[tool_body_id]
        axis_out = tool_pos - self.data.xpos[parent_id]
        tip_vec = self.data.site_xpos[self.peg_tip_site_id] - tool_pos
        if float(np.dot(axis_out, tip_vec)) <= 0.0:
            raise RuntimeError(
                "Peg points back into the wrist. Flip the outward-axis sign in "
                "_detect_flange_transform (z_sign) for your gen3.xml."
            )

    def _init_episode_buffers(self) -> None:
        self._step_index = 0
        self._hold_steps = 0
        self._jam_steps = 0
        self._prev_twist = np.zeros(6, dtype=np.float64)
        self._filtered_twist = np.zeros(6, dtype=np.float64)
        self._cmd_queue: Deque[np.ndarray] = deque()
        self._q_target = DEFAULT_HOME_QPOS.copy()
        self.episode_params: Dict[str, Any] = {}

    def get_active_contacts(self) -> List[Tuple[str, str, float]]:
        """Return list of (geom1_name, geom2_name, force_N) for debugging."""
        contacts: List[Tuple[str, str, float]] = []
        c_force = np.zeros(6, dtype=np.float64)
        for i in range(int(self.data.ncon)):
            con = self.data.contact[i]
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom1)) or f"geom_{con.geom1}"
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom2)) or f"geom_{con.geom2}"
            mujoco.mj_contactForce(self.model, self.data, i, c_force)
            contacts.append((g1, g2, float(np.linalg.norm(c_force[:3]))))
        return contacts

    def reset(
        self,
        seed: Optional[int] = None,
        config: Optional[EpisodeConfig] = None,
        hole: Optional[str] = None,
    ) -> Observation:
        """Reset the episode.

        ``hole``: ``"top"``, ``"side"`` or ``"random"``; defaults to the value
        given at construction.
        """
        if config is not None:
            self.config = config
            layout = self.config.domain_randomization.insertion_layout
            if layout not in ("top", "horizontal", "random"):
                raise ValueError(
                    "domain_randomization.insertion_layout must be 'top', "
                    f"'horizontal', or 'random'; got {layout!r}"
                )
            if self.config.domain_randomization.insertion_layout in ("horizontal", "random"):
                if not self.enable_second_hole or self.second_hole_layout != "wall":
                    self.enable_second_hole = True
                    self.second_hole_layout = "wall"
                    self._nominal_side_hole_pos = np.array(
                        SECOND_HOLE_LAYOUTS["wall"]["pos"], dtype=np.float64
                    )
                    self._build_model(
                        clearance_m=self.config.domain_randomization.clearance_mm_range[0] / 1000.0
                    )
        active_seed = self.config.seed if seed is None else seed
        self._rng = np.random.default_rng(active_seed)
        dr = self.config.domain_randomization

        choice = hole if hole is not None else (
            self.config.domain_randomization.insertion_layout
            if config is not None
            else self._default_hole
        )
        if choice in ("horizontal", "random") and (hole is None or choice == "horizontal"):
            choice = resolve_insertion_layout(choice, self._rng)
        available = tuple(self._holes)
        valid = available + ("horizontal", "random") if len(available) > 1 else available
        if choice not in valid:
            raise ValueError(f"hole must be one of {valid}, got {choice!r}")
        if choice == "random":
            choice = available[int(self._rng.integers(0, len(available)))]

        c_lo, c_hi = dr.clearance_mm_range
        clearance_m = (
            float(self._rng.uniform(c_lo, c_hi)) / 1000.0 if (dr.enabled and c_hi > c_lo) else c_lo / 1000.0
        )
        if dr.enabled and c_hi > c_lo:
            self._build_model(clearance_m=clearance_m)

        mujoco.mj_resetData(self.model, self.data)
        self._init_episode_buffers()
        self._select_hole(choice)

        # Put both holes back to nominal, then randomize only the active one.
        for spec in self._holes.values():
            self.model.body_pos[spec["body_id"]] = spec["nominal_pos"]
            self.model.body_quat[spec["body_id"]] = spec["base_quat"]

        spec = self._holes[choice]
        hole_pos = spec["nominal_pos"].copy()
        hole_yaw = 0.0
        if dr.enabled:
            if choice == "top" or not SECOND_HOLE_LAYOUTS[self.second_hole_layout]["wall"]:
                # Slide on the table plane (X, Y)
                hole_pos[0] += float(self._rng.uniform(-dr.hole_xy_range_m[0], dr.hole_xy_range_m[0]))
                hole_pos[1] += float(self._rng.uniform(-dr.hole_xy_range_m[1], dr.hole_xy_range_m[1]))
            else:
                # Slide on the wall plane (X, Z)
                hole_pos[0] += float(self._rng.uniform(-dr.hole_xy_range_m[0], dr.hole_xy_range_m[0]))
                hole_pos[2] += float(self._rng.uniform(-dr.hole_xy_range_m[1], dr.hole_xy_range_m[1]))
            hole_yaw = float(self._rng.uniform(-dr.hole_yaw_range_rad, dr.hole_yaw_range_rad))

        hole_quat = _hole_quat(spec["base_quat"], hole_yaw)
        self.model.body_pos[self.hole_body_id] = hole_pos
        self.model.body_quat[self.hole_body_id] = hole_quat

        friction = 0.4
        if dr.enabled:
            friction = float(self._rng.uniform(dr.friction_range[0], dr.friction_range[1]))
        self.model.geom_friction[self.peg_geom_id, 0] = friction
        for gid in self.hole_geom_ids:
            if gid >= 0:
                self.model.geom_friction[gid, 0] = friction

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

        self.data.qpos[self.qpos_ids] = DEFAULT_HOME_QPOS.copy()
        self.data.ctrl[self.actuator_ids] = DEFAULT_HOME_QPOS.copy()
        mujoco.mj_forward(self.model, self.data)

        # Start pose: above the hole entry along the hole axis, peg pointing into the hole.
        hole_R = _quat_to_rotmat(hole_quat)
        hole_x, hole_y, hole_axis = hole_R[:, 0], hole_R[:, 1], hole_R[:, 2]
        standoff = self.hole_depth + float(
            self._rng.uniform(dr.peg_start_z_offset_m[0], dr.peg_start_z_offset_m[1])
            if dr.enabled
            else np.mean(dr.peg_start_z_offset_m)
        )
        start_target = hole_pos + hole_axis * standoff
        if dr.enabled:
            start_target += hole_x * float(
                self._rng.uniform(-dr.peg_start_xy_offset_m[0], dr.peg_start_xy_offset_m[0])
            )
            start_target += hole_y * float(
                self._rng.uniform(-dr.peg_start_xy_offset_m[1], dr.peg_start_xy_offset_m[1])
            )

        self._settle_to_start_pose(start_target, target_axis=-hole_axis)

        lat_lo, lat_hi = dr.latency_steps_range
        latency_steps = (
            int(self._rng.integers(lat_lo, lat_hi + 1))
            if (dr.enabled and lat_hi > lat_lo)
            else self.config.control_limits.latency_steps
        )
        self._cmd_queue = deque([np.zeros(6, dtype=np.float64)] * latency_steps)

        self.episode_params = {
            "seed": active_seed,
            "active_hole": choice,
            "insertion_layout": "horizontal" if choice == "side" else "top",
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

    def _settle_to_start_pose(
        self,
        target_pos: np.ndarray,
        target_axis: np.ndarray,
        iters: int = 300,
        pos_tol: float = 1e-3,
        ang_tol: float = 1e-2,
    ) -> None:
        """Kinematically move the peg tip to ``target_pos`` with its axis along ``target_axis``.

        Uses the full 6x7 Jacobian: position + peg-axis alignment (roll about
        the peg axis is free) with a nullspace pull toward DEFAULT_HOME_QPOS.
        This keeps the 7-DoF arm in a sane configuration instead of letting a
        position-only solve wander (e.g. joint_3 spinning to ~4.9 rad).
        """
        target_axis = np.asarray(target_axis, dtype=np.float64)
        target_axis = target_axis / np.linalg.norm(target_axis)
        lam2 = self.dls_damping**2

        for _ in range(iters):
            mujoco.mj_forward(self.model, self.data)
            cur_pos = self.data.site_xpos[self.peg_tip_site_id].copy()
            cur_axis = self.data.site_xmat[self.peg_tip_site_id].reshape(3, 3)[:, 2].copy()

            pos_err = target_pos - cur_pos
            rot_err = np.cross(cur_axis, target_axis)  # world-frame rotation vector (axis only)
            if np.linalg.norm(pos_err) < pos_tol and np.linalg.norm(rot_err) < ang_tol:
                break

            mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
            J = np.vstack([self._jacp[:, self.dof_ids], self._jacr[:, self.dof_ids]])
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(6))

            q_cur = self.data.qpos[self.qpos_ids].copy()
            null_proj = np.eye(NUM_ARM_JOINTS) - J_pinv @ J
            err = np.concatenate([pos_err, rot_err])
            dq = J_pinv @ (0.5 * err) + null_proj @ (0.1 * (DEFAULT_HOME_QPOS - q_cur))
            dq = np.clip(dq, -0.1, 0.1)

            self.data.qpos[self.qpos_ids] = np.clip(
                q_cur + dq, KINOVA_JOINT_LIMITS[:, 0], KINOVA_JOINT_LIMITS[:, 1]
            )
            self.data.qvel[self.dof_ids] = 0.0

        self._q_target = self.data.qpos[self.qpos_ids].copy()
        self.data.ctrl[self.actuator_ids] = self._q_target.copy()
        mujoco.mj_forward(self.model, self.data)

    def _apply_control_constraints(self, command: CartesianCommand) -> np.ndarray:
        limits = self.config.control_limits
        dt = limits.control_dt

        if not command.clutch_engaged:
            raw_twist = np.zeros(6, dtype=np.float64)
        elif command.mode == "delta_pose":
            raw_twist = command.as_vector() / dt
        else:
            raw_twist = command.as_vector().copy()

        if limits.deadband_linear > 0.0 and np.linalg.norm(raw_twist[:3]) < limits.deadband_linear:
            raw_twist[:3] = 0.0
        if limits.deadband_angular > 0.0 and np.linalg.norm(raw_twist[3:]) < limits.deadband_angular:
            raw_twist[3:] = 0.0

        if len(self._cmd_queue) > 0:
            self._cmd_queue.append(raw_twist)
            delayed_twist = self._cmd_queue.popleft()
        else:
            delayed_twist = raw_twist

        alpha = float(np.clip(limits.lpf_alpha, 0.01, 1.0))
        self._filtered_twist = alpha * delayed_twist + (1.0 - alpha) * self._filtered_twist
        twist = self._filtered_twist.copy()

        lin_norm = float(np.linalg.norm(twist[:3]))
        if lin_norm > limits.max_linear_vel:
            twist[:3] *= limits.max_linear_vel / lin_norm
        ang_norm = float(np.linalg.norm(twist[3:]))
        if ang_norm > limits.max_angular_vel:
            twist[3:] *= limits.max_angular_vel / ang_norm

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
        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
        J = np.vstack([self._jacp[:, self.dof_ids], self._jacr[:, self.dof_ids]])

        jjt = J @ J.T + (self.dls_damping**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(jjt)

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
        num_peg_contacts = 0
        peg_hole_contact = False
        table_collision = False
        peg_contact_force = 0.0
        c_force = np.zeros(6, dtype=np.float64)

        for i in range(int(self.data.ncon)):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            pair = {g1, g2}
            if self.peg_geom_id in pair:
                num_peg_contacts += 1
                mujoco.mj_contactForce(self.model, self.data, i, c_force)
                peg_contact_force += float(np.linalg.norm(c_force[:3]))
                if g1 in self.hole_geom_ids or g2 in self.hole_geom_ids:
                    peg_hole_contact = True
                # Table or side wall counts as an obstacle collision
                if pair & self.obstacle_geom_ids:
                    table_collision = True

        wrench = np.zeros(6, dtype=np.float64)
        if self.force_sensor_id >= 0:
            adr = self.model.sensor_adr[self.force_sensor_id]
            wrench[:3] = self.data.sensordata[adr : adr + 3]
        if self.torque_sensor_id >= 0:
            adr = self.model.sensor_adr[self.torque_sensor_id]
            wrench[3:] = self.data.sensordata[adr : adr + 3]

        crit = self.config.success_criteria
        ee_speed = float(np.linalg.norm(self._prev_twist[:3]))
        if peg_hole_contact and peg_contact_force >= crit.jam_force_threshold_n and ee_speed < 0.005:
            self._jam_steps += 1
        else:
            self._jam_steps = max(0, self._jam_steps - 1)

        return ContactState(
            in_contact=num_peg_contacts > 0,
            num_contacts=num_peg_contacts,
            peg_hole_contact=peg_hole_contact,
            table_collision=table_collision,
            total_contact_force_n=peg_contact_force,
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

        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.peg_tip_site_id)
        ee_v_lin = self._jacp[:, self.dof_ids] @ raw_qvel
        ee_v_ang = self._jacr[:, self.dof_ids] @ raw_qvel

        # Active hole's entry frame (local +Z = entry axis, for either hole)
        hole_pos = self.data.site_xpos[self.hole_entry_site_id].copy()
        hole_xmat = self.data.site_xmat[self.hole_entry_site_id].reshape(3, 3)
        hole_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(hole_quat, hole_xmat.flatten())

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

        peg_z = _quat_to_rotmat(obs.peg_quat)[:, 2]
        hole_z = _quat_to_rotmat(obs.hole_quat)[:, 2]
        cos_tilt = float(np.clip(abs(np.dot(peg_z, hole_z)), -1.0, 1.0))
        tilt_err = float(np.arccos(cos_tilt))

        info = {
            "lateral_error_m": lateral_err,
            "insertion_depth_m": insertion_depth,
            "tilt_error_rad": tilt_err,
            "hold_steps": self._hold_steps,
            "active_hole": self.active_hole,
        }

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
