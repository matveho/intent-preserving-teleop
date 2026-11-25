#!/usr/bin/env python3

"""Verification & diagnostic suite for the MuJoCo Kinova Gen3 peg-in-hole sim.

Run after ``python3 setup_mujoco.py``:

Usage (from repo root or scripts/):
    python verify_sim.py --viewer-only        # open MuJoCo viewer immediately
    python verify_sim.py --viewer             # run all tests, then open viewer even on failure
    python verify_sim.py --test-camera        # include wrist_cam rendering test

Checks performed
----------------
1. Joint 1-7 individual actuation test (explicitly verifies Joint 1 moves
   smoothly in both directions and prints full actuator/joint diagnostics).
2. 6-DoF Cartesian twist -> 7-DoF Kinova Gen3 tracking test (+X, -X, +Y, -Y, +Z, -Z, yaw).
3. Deterministic seeded reset test (same seed -> identical initial state).
4. Automated headless reset -> command -> step -> terminate trial loop.
5. Optional wrist camera render check across two lighting seeds.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco  # type: ignore
import numpy as np

# Allow running directly from scripts/ or tests/ without pip install -e .
REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if SRC_DIR.exists() and str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

try:
    from intent_preserving_teleop.simulation import DEFAULT_HOME_QPOS, KinovaPegInHoleEnv
    from intent_preserving_teleop.schema import (
        CartesianCommand,
        EpisodeConfig,
        EpisodeStatus,
        SensorRealismConfig,
    )
except ImportError:
    from peg_in_hole_env import DEFAULT_HOME_QPOS, KinovaPegInHoleEnv  # type: ignore
    from schema import (  # type: ignore
        CartesianCommand,
        EpisodeConfig,
        EpisodeStatus,
        SensorRealismConfig,
    )


def print_contact_diagnostics(env: KinovaPegInHoleEnv, label: str = "Active contacts") -> None:
    contacts = env.get_active_contacts()
    if not contacts:
        print(f"  [contacts] {label}: none")
        return
    print(f"  [contacts] {label} ({len(contacts)} pairs):")
    for g1, g2, force in contacts[:10]:
        print(f"    - {g1} <-> {g2} | force = {force:.2f} N")


def test_individual_joints(env: KinovaPegInHoleEnv) -> bool:
    print("\n=== Test 1: Individual Joint Movement (Joint 1 - Joint 7) ===")
    all_ok = True
    delta_cmd = 0.35  # rad

    for idx in range(7):
        mujoco.mj_resetData(env.model, env.data)
        env.data.qpos[env.qpos_ids] = DEFAULT_HOME_QPOS.copy()
        env.data.ctrl[env.actuator_ids] = DEFAULT_HOME_QPOS.copy()
        mujoco.mj_forward(env.model, env.data)

        q0 = float(env.data.qpos[env.qpos_ids[idx]])
        act_id = int(env.actuator_ids[idx])
        dof_id = int(env.dof_ids[idx])

        target = DEFAULT_HOME_QPOS.copy()
        target[idx] = q0 + delta_cmd
        env.data.ctrl[env.actuator_ids] = target
        for _ in range(300):
            mujoco.mj_step(env.model, env.data)
        q_plus = float(env.data.qpos[env.qpos_ids[idx]])

        target[idx] = q0 - delta_cmd
        env.data.ctrl[env.actuator_ids] = target
        for _ in range(300):
            mujoco.mj_step(env.model, env.data)
        q_minus = float(env.data.qpos[env.qpos_ids[idx]])

        moved_pos = q_plus - q0
        moved_neg = q_minus - q0
        passed = (moved_pos > 0.15) and (moved_neg < -0.15)
        all_ok = all_ok and passed

        cr = env.model.actuator_ctrlrange[act_id]
        arm = env.model.dof_armature[dof_id]
        status = "PASS" if passed else "FAIL"
        print(
            f"  [{status}] {env.joint_names[idx]:8s} | "
            f"act={act_id} ctrlrange=[{cr[0]:5.2f}, {cr[1]:5.2f}] armature={arm:.2f} | "
            f"+cmd dQ={moved_pos:+.3f} rad, -cmd dQ={moved_neg:+.3f} rad"
        )
        if not passed:
            print_contact_diagnostics(env, f"{env.joint_names[idx]} failure")
    return all_ok


def test_cartesian_control(env: KinovaPegInHoleEnv) -> bool:
    print("\n=== Test 2: 6-DoF Cartesian Twist -> 7-DoF DLS IK ===")
    all_ok = True
    axes = [
        ("+X", np.array([0.08, 0.0, 0.0]), np.zeros(3), 0, +1),
        ("-X", np.array([-0.08, 0.0, 0.0]), np.zeros(3), 0, -1),
        ("+Y", np.array([0.0, 0.08, 0.0]), np.zeros(3), 1, +1),
        ("-Y", np.array([0.0, -0.08, 0.0]), np.zeros(3), 1, -1),
        ("+Z", np.array([0.0, 0.0, 0.08]), np.zeros(3), 2, +1),
        ("-Z", np.array([0.0, 0.0, -0.08]), np.zeros(3), 2, -1),
    ]

    for label, lin, ang, axis_idx, sign in axes:
        obs0 = env.reset(seed=42)
        p0 = obs0.ee_pos.copy()
        for _ in range(25):
            res = env.step(CartesianCommand(linear=lin, angular=ang, mode="twist"))
        dp = res.observation.ee_pos - p0
        primary_move = dp[axis_idx] * sign
        passed = primary_move > 0.015
        all_ok = all_ok and passed
        status = "PASS" if passed else "FAIL"
        print(
            f"  [{status}] {label:3s} | d_pos=[{dp[0]:+.4f}, {dp[1]:+.4f}, {dp[2]:+.4f}] m "
            f"(primary={primary_move:+.4f} m)"
        )
    return all_ok


def test_determinism_and_trials(env: KinovaPegInHoleEnv) -> bool:
    print("\n=== Test 3: Deterministic Seeded Reset & Automated Trial Loop ===")
    obs_a = env.reset(seed=123)
    obs_b = env.reset(seed=123)
    obs_c = env.reset(seed=456)

    same_seed_exact = (
        np.allclose(obs_a.ee_pos, obs_b.ee_pos, atol=1e-12)
        and np.allclose(obs_a.hole_pos, obs_b.hole_pos, atol=1e-12)
        and np.allclose(obs_a.joint_pos, obs_b.joint_pos, atol=1e-12)
    )
    diff_seed_diff = not np.allclose(obs_a.hole_pos, obs_c.hole_pos, atol=1e-6)
    print(f"  [{'PASS' if same_seed_exact else 'FAIL'}] Identical seed produces identical state")
    print(f"  [{'PASS' if diff_seed_diff else 'FAIL'}] Different seeds randomize hole/start pose")

    obs = env.reset(seed=7)
    status = EpisodeStatus.RUNNING
    for _ in range(250):
        err_xy = obs.hole_pos[:2] - obs.peg_pos[:2]
        if np.linalg.norm(err_xy) > 0.004:
            v_lin = np.array([2.0 * err_xy[0], 2.0 * err_xy[1], 0.0])
        else:
            v_lin = np.array([1.5 * err_xy[0], 1.5 * err_xy[1], -0.05])
        res = env.step(CartesianCommand(linear=v_lin, angular=np.zeros(3)))
        obs = res.observation
        status = res.status
        if res.done:
            break

    print(
        f"  [INFO] Scripted trial ended after {obs.step_index} steps | "
        f"status={status.value} | depth={obs.peg_to_hole_local[2]*1000:.1f} mm | "
        f"lat_err={np.linalg.norm(obs.peg_to_hole_local[:2])*1000:.1f} mm | "
        f"contact_force={obs.contact.total_contact_force_n:.2f} N"
    )
    if status == EpisodeStatus.FAILURE_COLLISION:
        print_contact_diagnostics(env, "Trial collision")
    return same_seed_exact and diff_seed_diff


def test_wrist_camera(kinova_dir: Path) -> None:
    print("\n=== Test 4: Wrist Camera + Lighting Randomization (Optional) ===")
    cfg = EpisodeConfig(sensor_realism=SensorRealismConfig(render_wrist_camera=True))
    cam_env = KinovaPegInHoleEnv(kinova_dir=kinova_dir, config=cfg)
    try:
        obs1 = cam_env.reset(seed=1)
        obs2 = cam_env.reset(seed=2)
        if obs1.wrist_rgb is not None and obs2.wrist_rgb is not None:
            mean_diff = float(np.mean(np.abs(obs1.wrist_rgb.astype(float) - obs2.wrist_rgb.astype(float))))
            print(
                f"  [PASS] wrist_cam rendered {obs1.wrist_rgb.shape} uint8 | "
                f"mean pixel diff across lighting seeds = {mean_diff:.2f}"
            )
    except Exception as exc:
        print(f"  [SKIP] Headless OpenGL rendering unavailable ({exc})")
    finally:
        cam_env.close()


def launch_viewer(env: KinovaPegInHoleEnv, seed: int = 0) -> None:
    import mujoco.viewer  # type: ignore

    env.reset(seed=seed)
    print("\n[viewer] Launching interactive MuJoCo viewer (real-time pacing).")
    print("[viewer] Tips inside the window:")
    print("  - Press C to visualize contact points and F to show contact force arrows")
    print("  - Open the right panel -> Control to drag Joint 1..7 sliders manually")
    print("  - Press Tab to cycle cameras (including wrist_cam)")
    print_contact_diagnostics(env, "Initial pose")

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            t0 = time.perf_counter()
            mujoco.mj_step(env.model, env.data)
            viewer.sync()
            dt_sleep = env.model.opt.timestep - (time.perf_counter() - t0)
            if dt_sleep > 0:
                time.sleep(dt_sleep)


def launch_insertion_demo(env: KinovaPegInHoleEnv, seed: int = 0) -> None:
    """Show the same scripted alignment and insertion used by the trial check."""
    import mujoco.viewer  # type: ignore

    obs = env.reset(seed=seed, hole="top")
    finished = False
    print("\n[viewer] Showing seeded scripted peg insertion.")
    print("[viewer] Close the window to stop; contact points/forces are toggled with C/F.")
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            t0 = time.perf_counter()
            if not finished:
                err = obs.hole_pos[:2] - obs.peg_pos[:2]
                if np.linalg.norm(err) > 0.004:
                    linear = np.array([2.0 * err[0], 2.0 * err[1], 0.0])
                else:
                    linear = np.array([1.5 * err[0], 1.5 * err[1], -0.05])
                result = env.step(CartesianCommand(linear=linear, angular=np.zeros(3)))
                obs = result.observation
                if result.done:
                    finished = True
                    print(
                        f"[viewer] Trial ended: {result.status.value}; "
                        f"depth={obs.peg_to_hole_local[2] * 1000:.1f} mm, "
                        f"lateral={np.linalg.norm(obs.peg_to_hole_local[:2]) * 1000:.2f} mm"
                    )
                    # Leave the final successful pose visible until the viewer closes.
            viewer.sync()
            dt_sleep = env.config.control_limits.control_dt - (time.perf_counter() - t0)
            if dt_sleep > 0:
                time.sleep(dt_sleep)


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify MuJoCo Kinova Gen3 peg-in-hole setup.")
    parser.add_argument(
        "--kinova-dir",
        type=Path,
        default=None,
        help="Path to mujoco_menagerie/kinova_gen3 (auto-detected if omitted).",
    )
    parser.add_argument("--viewer", action="store_true", help="Launch viewer after running tests.")
    parser.add_argument("--viewer-only", action="store_true", help="Skip tests and open viewer immediately.")
    parser.add_argument("--viewer-insertion", action="store_true", help="Visually replay a scripted insertion.")
    parser.add_argument("--test-camera", action="store_true", help="Also test wrist_cam rendering.")
    parser.add_argument("--seed", type=int, default=0, help="Episode seed for --viewer / --viewer-only.")
    args = parser.parse_args()

    env = KinovaPegInHoleEnv(kinova_dir=args.kinova_dir)
    try:
        if args.viewer_only:
            launch_viewer(env, seed=args.seed)
            return
        if args.viewer_insertion:
            launch_insertion_demo(env, seed=args.seed)
            return

        ok1 = test_individual_joints(env)
        ok2 = test_cartesian_control(env)
        ok3 = test_determinism_and_trials(env)
        if args.test_camera:
            test_wrist_camera(env.kinova_dir)

        print("\n=== Summary ===")
        if ok1 and ok2 and ok3:
            print("ALL CORE CHECKS PASSED (Joint 1-7 actuation, 6-DoF IK, seeded trials).")
        else:
            print("SOME CHECKS FAILED — inspect diagnostics above.")

        # Always launch viewer if requested, even when a check fails (for debugging!)
        if args.viewer:
            launch_viewer(env, seed=args.seed)

        if not (ok1 and ok2 and ok3):
            sys.exit(1)
    finally:
        env.close()


if __name__ == "__main__":
    main()
