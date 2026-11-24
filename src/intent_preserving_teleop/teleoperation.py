"""Fixed PHANToM-to-robot pose mapping; no clutch or workspace shifting."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from .schema import CartesianCommand

# PHANToM right/up/towards operator -> robot forward/left/up.
PHANTOM_TO_BASE = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
# Driver tip offset is along local -Z; the simulated peg points along +Z.
STYLUS_TO_PEG = np.diag([1., -1., -1.])
# Full geometric bounds from the local Omni configuration, converted mm -> m.
PHANTOM_MIN = np.array([-.210, -.110, -.085])
PHANTOM_MAX = np.array([.210, .205, .130])


@dataclass
class PoseSample:
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    received_at: float


class PoseTeleop:
    timeout = 0.2

    def __init__(self, limits):
        self.center = (np.array(limits.workspace_min) + limits.workspace_max) / 2
        self.span = .95 * (np.array(limits.workspace_max) - limits.workspace_min)
        self.target_position = None
        self.target_rotation = None

    def command(self, sample, observation, now):
        if sample is None or not 0 <= now - sample.received_at <= self.timeout:
            return None
        if not np.isfinite(np.r_[sample.position, sample.quaternion_xyzw]).all() or np.linalg.norm(sample.quaternion_xyzw) < 1e-6:
            return None
        device = Rotation.from_quat(sample.quaternion_xyzw).as_matrix()
        current = Rotation.from_quat(observation.ee_quat[[1, 2, 3, 0]])
        normalized = (sample.position - (PHANTOM_MIN + PHANTOM_MAX) / 2) / (PHANTOM_MAX - PHANTOM_MIN)
        self.target_position = self.center + self.span * np.clip(PHANTOM_TO_BASE @ normalized, -.5, .5)
        self.target_rotation = Rotation.from_matrix(PHANTOM_TO_BASE @ device @ STYLUS_TO_PEG)
        return CartesianCommand(
            linear=4.0 * (self.target_position - observation.ee_pos),
            angular=4.0 * (self.target_rotation * current.inv()).as_rotvec(),
            timestamp=now, source='human')
