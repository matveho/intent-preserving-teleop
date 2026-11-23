"""Shared robot, observation, action, and episode schema.

Step 0 (Frozen Interface Contract):
Simulation (MuJoCo), PHANToM teleoperation, demonstration logging, ACT policy
inference, shared-autonomy arbitration, and the physical Kinova Gen3 Kortex
backend must all adhere to this schema so swapping sim <-> real only replaces
the RobotBackend implementation.

Conventions
-----------
- Reference frame : ``base_link`` (robot base, right-handed, +Z up)
- Linear units    : meters (m), meters/second (m/s), meters/second^2 (m/s^2)
- Angular units   : radians (rad), radians/second (rad/s), radians/second^2 (rad/s^2)
- Quaternions     : ``[w, x, y, z]`` unit quaternion (MuJoCo native order)
- Force / Torque  : Newtons (N), Newton-meters (N*m)
- Timestamps      : monotonic seconds (float64)
- Control rate    : 50 Hz default (dt = 0.02 s); MuJoCo physics at 500 Hz (0.002 s)


Needs to be verified that it works...
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Literal, Optional, Tuple

import numpy as np

SCHEMA_VERSION = "1.0.0"
BASE_FRAME = "base_link"
NUM_ARM_JOINTS = 7  # Kinova Gen3 7-DoF
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_SIM_TIMESTEP = 0.002


class EpisodeStatus(str, Enum):
    """Termination and progress states shared across sim, logging, and eval."""

    RUNNING = "running"
    SUCCESS = "success"
    FAILURE_TIMEOUT = "failure_timeout"
    FAILURE_COLLISION = "failure_collision"
    FAILURE_OUT_OF_BOUNDS = "failure_out_of_bounds"
    FAILURE_JAMMED = "failure_jammed"
    FAILURE_COMM_LOSS = "failure_comm_loss"


@dataclass
class ControlLimits:
    """Physical and safety bounds matched between MuJoCo and physical Gen3."""

    control_hz: float = DEFAULT_CONTROL_HZ
    max_linear_vel: float = 0.25       # m/s (conservative teleop cap)
    max_angular_vel: float = 0.80      # rad/s
    max_linear_acc: float = 1.00       # m/s^2
    max_angular_acc: float = 3.00      # rad/s^2
    max_joint_vel: float = 1.20        # rad/s
    # Workspace bounding box in base_link frame (meters)
    workspace_min: Tuple[float, float, float] = (0.20, -0.40, 0.02)
    workspace_max: Tuple[float, float, float] = (0.75, 0.40, 0.65)
    # Command transport realism
    latency_steps: int = 0             # integer control steps of delay
    lpf_alpha: float = 1.0             # 1.0 = no filtering; (0, 1) = exponential LPF
    deadband_linear: float = 0.0       # m/s deadband for teleop noise
    deadband_angular: float = 0.0      # rad/s deadband for teleop noise

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_hz


@dataclass
class SensorRealismConfig:
    """Configurable sensor quantization and noise for sim-to-real fidelity.

    Real Kinova Gen3 actuators report finite-resolution joint encoder positions
    rather than exact float64 simulation states. Enabling quantization and
    small observation noise prevents policies from overfitting to perfect
    simulator coordinates.
    """

    # 14-bit equivalent (~0.022 deg = 3.835e-4 rad) default; set None to disable
    encoder_quantization_rad: Optional[float] = 2.0 * np.pi / (2**14)
    cartesian_round_decimals: Optional[int] = 4  # 0.1 mm resolution
    joint_pos_noise_std_rad: float = 0.0
    ee_pos_noise_std_m: float = 0.0
    render_wrist_camera: bool = False
    camera_width: int = 320
    camera_height: int = 240


@dataclass
class DomainRandomizationConfig:
    """Per-episode randomization bounds (logged per episode for reproducibility)."""

    enabled: bool = True
    hole_xy_range_m: Tuple[float, float] = (0.04, 0.04)     # +/- half-range around nominal
    hole_yaw_range_rad: float = 0.25                        # +/- rad
    peg_start_xy_offset_m: Tuple[float, float] = (0.06, 0.06)
    peg_start_z_offset_m: Tuple[float, float] = (0.06, 0.12)
    friction_range: Tuple[float, float] = (0.25, 0.75)      # sliding friction coeff
    clearance_mm_range: Tuple[float, float] = (3.0, 3.0)    # forgiving default; tighten later
    randomize_lighting: bool = True
    latency_steps_range: Tuple[int, int] = (0, 0)


@dataclass
class SuccessCriteria:
    """Deterministic peg-in-hole insertion success and failure thresholds."""

    insertion_depth_m: float = 0.030       # peg tip >= 30 mm below hole entry
    max_lateral_error_m: float = 0.005     # <= 5 mm radial offset from hole axis
    max_tilt_error_rad: float = 0.12       # <= ~7 deg alignment error
    hold_steps_required: int = 10          # sustain inserted state for 0.2 s at 50 Hz
    max_episode_steps: int = 1500          # 30 s at 50 Hz
    max_contact_force_n: float = 40.0      # excessive collision threshold
    jam_force_threshold_n: float = 15.0    # sustained force with near-zero motion
    jam_steps_threshold: int = 50          # 1.0 s at 50 Hz


@dataclass
class EpisodeConfig:
    """Full configuration for one deterministic episode."""

    seed: int = 0
    control_limits: ControlLimits = field(default_factory=ControlLimits)
    sensor_realism: SensorRealismConfig = field(default_factory=SensorRealismConfig)
    domain_randomization: DomainRandomizationConfig = field(
        default_factory=DomainRandomizationConfig
    )
    success_criteria: SuccessCriteria = field(default_factory=SuccessCriteria)


@dataclass
class CartesianCommand:
    """Bounded 6-DoF Cartesian control command in ``base_link`` frame.

    Attributes
    ----------
    linear:
        3D linear velocity [vx, vy, vz] in m/s (if mode == "twist") or
        position increment [dx, dy, dz] in m (if mode == "delta_pose").
    angular:
        3D angular velocity [wx, wy, wz] in rad/s (if mode == "twist") or
        axis-angle rotation increment [rx, ry, rz] in rad (if mode == "delta_pose").
    mode:
        "twist" (recommended for shared autonomy & Kortex Gen3) or "delta_pose".
    clutch_engaged:
        When False, command is treated as zero motion (hold current pose).
    extend_workspace:
        Optional PHANToM pen-button mode: when the operator reaches the physical
        PHANToM boundary and presses the extension button, the controller keeps
        applying bounded motion along the commanded direction.
    timestamp:
        Command creation time in monotonic seconds.
    source:
        Origin of the command ("human", "policy", "shared", or "script").
    """

    linear: np.ndarray
    angular: np.ndarray
    mode: Literal["twist", "delta_pose"] = "twist"
    clutch_engaged: bool = True
    extend_workspace: bool = False
    timestamp: float = 0.0
    source: Literal["human", "policy", "shared", "script"] = "script"

    def __post_init__(self) -> None:
        self.linear = np.asarray(self.linear, dtype=np.float64).reshape(3)
        self.angular = np.asarray(self.angular, dtype=np.float64).reshape(3)

    @classmethod
    def zero(cls, timestamp: float = 0.0, source: str = "script") -> "CartesianCommand":
        return cls(
            linear=np.zeros(3, dtype=np.float64),
            angular=np.zeros(3, dtype=np.float64),
            timestamp=timestamp,
            source=source,  # type: ignore[arg-type]
        )

    def as_vector(self) -> np.ndarray:
        """Return 6-element vector [linear(3), angular(3)]."""
        return np.concatenate([self.linear, self.angular])


@dataclass
class ContactState:
    """Contact and force feedback summary for a single control step."""

    in_contact: bool = False
    num_contacts: int = 0
    peg_hole_contact: bool = False
    table_collision: bool = False
    total_contact_force_n: float = 0.0
    wrench_ee: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    is_jammed: bool = False


@dataclass
class Observation:
    """Synchronized robot, task, contact, and optional visual observation."""

    timestamp: float
    step_index: int
    # End-effector (peg tip / tool frame) in base_link
    ee_pos: np.ndarray             # shape (3,), meters
    ee_quat: np.ndarray            # shape (4,), [w, x, y, z]
    ee_vel_linear: np.ndarray      # shape (3,), m/s
    ee_vel_angular: np.ndarray     # shape (3,), rad/s
    # 7-DoF Kinova Gen3 joint state
    joint_pos: np.ndarray          # shape (7,), rad
    joint_vel: np.ndarray          # shape (7,), rad/s
    joint_torque: np.ndarray       # shape (7,), N*m
    # Task-specific peg & hole state in base_link
    peg_pos: np.ndarray            # shape (3,), meters (peg tip)
    peg_quat: np.ndarray           # shape (4,), [w, x, y, z]
    hole_pos: np.ndarray           # shape (3,), meters (hole entry center)
    hole_quat: np.ndarray          # shape (4,), [w, x, y, z]
    peg_to_hole_local: np.ndarray  # shape (3,), [dx, dy, insertion_depth] in hole frame
    # Contact & optional camera
    contact: ContactState = field(default_factory=ContactState)
    wrist_rgb: Optional[np.ndarray] = None  # shape (H, W, 3), uint8

    def to_flat_dict(self) -> Dict[str, Any]:
        """Serialize numeric fields for dataset logging (HDF5/NPZ/JSON)."""
        return {
            "timestamp": float(self.timestamp),
            "step_index": int(self.step_index),
            "ee_pos": self.ee_pos.tolist(),
            "ee_quat": self.ee_quat.tolist(),
            "ee_vel_linear": self.ee_vel_linear.tolist(),
            "ee_vel_angular": self.ee_vel_angular.tolist(),
            "joint_pos": self.joint_pos.tolist(),
            "joint_vel": self.joint_vel.tolist(),
            "joint_torque": self.joint_torque.tolist(),
            "peg_pos": self.peg_pos.tolist(),
            "peg_quat": self.peg_quat.tolist(),
            "hole_pos": self.hole_pos.tolist(),
            "hole_quat": self.hole_quat.tolist(),
            "peg_to_hole_local": self.peg_to_hole_local.tolist(),
            "in_contact": bool(self.contact.in_contact),
            "num_contacts": int(self.contact.num_contacts),
            "peg_hole_contact": bool(self.contact.peg_hole_contact),
            "table_collision": bool(self.contact.table_collision),
            "total_contact_force_n": float(self.contact.total_contact_force_n),
            "wrench_ee": self.contact.wrench_ee.tolist(),
            "is_jammed": bool(self.contact.is_jammed),
        }


@dataclass
class StepResult:
    """Return object for one control step."""

    observation: Observation
    executed_twist: np.ndarray  # shape (6,) actually applied after safety limits
    status: EpisodeStatus
    done: bool
    info: Dict[str, Any] = field(default_factory=dict)


class RobotBackend(ABC):
    """Unified interface implemented by both MuJoCo sim and physical Gen3 Kortex."""

    @abstractmethod
    def reset(
        self,
        seed: Optional[int] = None,
        config: Optional[EpisodeConfig] = None,
    ) -> Observation:
        """Reset episode deterministically from seed and return initial observation."""

    @abstractmethod
    def step(self, command: CartesianCommand) -> StepResult:
        """Execute one bounded 6-DoF Cartesian control step."""

    @abstractmethod
    def get_observation(self) -> Observation:
        """Read current synchronized observation without advancing physics."""

    @abstractmethod
    def close(self) -> None:
        """Release simulation or hardware resources cleanly."""