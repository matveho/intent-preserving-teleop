#!/usr/bin/env python3
"""Exercise and report the MuJoCo backend's configured motion constraints."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from intent_preserving_teleop.schema import CartesianCommand  # noqa: E402
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1901)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "outputs" / "simulation_verification"
    )
    args = parser.parse_args()

    env = KinovaPegInHoleEnv()
    try:
        env.reset(seed=args.seed, hole="top")
        limits = env.config.control_limits
        rows: list[dict[str, float | int]] = []
        previous = np.zeros(6, dtype=np.float64)
        # Alternate over-limit commands on every axis to exercise vector bounds.
        requests = [
            np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 4.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 0.0, 4.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 4.0]),
        ]
        for i in range(240):
            requested = requests[(i // 30) % len(requests)]
            result = env.step(
                CartesianCommand(linear=requested[:3], angular=requested[3:])
            )
            actual = result.executed_twist
            rows.append(
                {
                    "step": i,
                    "timestamp_s": result.observation.timestamp,
                    "linear_speed_m_s": float(np.linalg.norm(actual[:3])),
                    "angular_speed_rad_s": float(np.linalg.norm(actual[3:])),
                    "linear_accel_m_s2": float(
                        np.linalg.norm(actual[:3] - previous[:3]) / limits.control_dt
                    ),
                    "angular_accel_rad_s2": float(
                        np.linalg.norm(actual[3:] - previous[3:]) / limits.control_dt
                    ),
                    "max_joint_speed_rad_s": float(
                        np.max(np.abs(env.data.qvel[env.dof_ids]))
                    ),
                }
            )
            previous = actual.copy()
            if result.done:
                break

        timestamps = np.asarray([r["timestamp_s"] for r in rows], dtype=float)
        intervals = np.diff(timestamps)
        summary = {
            "seed": args.seed,
            "configured_control_hz": limits.control_hz,
            "configured_control_dt_s": limits.control_dt,
            "physics_timestep_s": float(env.model.opt.timestep),
            "physics_substeps_per_control": int(round(limits.control_dt / env.model.opt.timestep)),
            "max_linear_speed_m_s": max(float(r["linear_speed_m_s"]) for r in rows),
            "linear_speed_limit_m_s": limits.max_linear_vel,
            "max_angular_speed_rad_s": max(float(r["angular_speed_rad_s"]) for r in rows),
            "angular_speed_limit_rad_s": limits.max_angular_vel,
            "max_linear_accel_m_s2": max(float(r["linear_accel_m_s2"]) for r in rows),
            "linear_accel_limit_m_s2": limits.max_linear_acc,
            "max_angular_accel_rad_s2": max(float(r["angular_accel_rad_s2"]) for r in rows),
            "angular_accel_limit_rad_s2": limits.max_angular_acc,
            "max_joint_speed_rad_s": max(float(r["max_joint_speed_rad_s"]) for r in rows),
            "joint_speed_limit_rad_s": limits.max_joint_vel,
            "max_observed_control_interval_s": float(np.max(intervals)) if len(intervals) else None,
            "configured_latency_steps": limits.latency_steps,
            "configured_lpf_alpha": limits.lpf_alpha,
            "configured_workspace_min_m": list(limits.workspace_min),
            "configured_workspace_max_m": list(limits.workspace_max),
            "steps_recorded": len(rows),
        }
        checks = {
            "linear_speed": summary["max_linear_speed_m_s"] <= limits.max_linear_vel + 1e-8,
            "angular_speed": summary["max_angular_speed_rad_s"] <= limits.max_angular_vel + 1e-8,
            "linear_acceleration": summary["max_linear_accel_m_s2"] <= limits.max_linear_acc + 1e-6,
            "angular_acceleration": summary["max_angular_accel_rad_s2"] <= limits.max_angular_acc + 1e-6,
            "joint_speed": summary["max_joint_speed_rad_s"] <= limits.max_joint_vel + 1e-6,
        }
        summary["checks"] = checks  # type: ignore[assignment]
        summary["passed"] = all(checks.values())  # type: ignore[assignment]

        args.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = args.output_dir / "control_constraints_steps.csv"
        json_path = args.output_dir / "control_constraints_summary.json"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        print(f"Wrote {csv_path} and {json_path}")
        return 0 if summary["passed"] else 1
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
