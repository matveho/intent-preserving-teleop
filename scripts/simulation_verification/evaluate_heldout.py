#!/usr/bin/env python3
"""Run the scripted alignment baseline on the reserved fixed seed set."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from intent_preserving_teleop.schema import CartesianCommand, EpisodeStatus  # noqa: E402
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv  # noqa: E402


def run_trial(env: KinovaPegInHoleEnv, seed: int, max_steps: int) -> dict[str, object]:
    obs = env.reset(seed=seed, hole="top")
    initial_pos = obs.ee_pos.copy()
    previous_pos = initial_pos.copy()
    path_length = 0.0
    collisions = 0
    contact_steps = 0
    status = EpisodeStatus.RUNNING
    steps = 0
    for steps in range(1, max_steps + 1):
        err_xy = obs.hole_pos[:2] - obs.peg_pos[:2]
        if np.linalg.norm(err_xy) > 0.004:
            linear = np.array([2.0 * err_xy[0], 2.0 * err_xy[1], 0.0])
        else:
            linear = np.array([1.5 * err_xy[0], 1.5 * err_xy[1], -0.05])
        result = env.step(CartesianCommand(linear=linear, angular=np.zeros(3)))
        obs, status = result.observation, result.status
        path_length += float(np.linalg.norm(obs.ee_pos - previous_pos))
        previous_pos = obs.ee_pos.copy()
        collisions += int(obs.contact.table_collision)
        contact_steps += int(obs.contact.peg_hole_contact)
        if result.done:
            break
    lateral_error = float(np.linalg.norm(obs.peg_to_hole_local[:2]))
    return {
        "seed": seed,
        "status": status.value,
        "success": status == EpisodeStatus.SUCCESS,
        "steps": steps,
        "completion_time_s": steps * env.config.control_limits.control_dt,
        "final_lateral_error_m": lateral_error,
        "final_depth_m": float(obs.peg_to_hole_local[2]),
        "path_length_m": path_length,
        "contact_steps": contact_steps,
        "table_collision_steps": collisions,
        "peak_contact_force_n": float(obs.contact.total_contact_force_n),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-file", type=Path, default=Path(__file__).with_name("heldout_v1.json"))
    parser.add_argument("--episodes", type=int, default=None, help="Prefix length of held-out seeds to run.")
    parser.add_argument("--max-steps", type=int, default=1500)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "outputs" / "simulation_verification"
    )
    args = parser.parse_args()
    manifest = json.loads(args.seed_file.read_text(encoding="utf-8"))
    seeds = manifest["seeds"]
    count = len(seeds) if args.episodes is None else args.episodes
    if count < 1 or count > len(seeds):
        parser.error(f"--episodes must be between 1 and {len(seeds)}")

    env = KinovaPegInHoleEnv()
    try:
        rows = [run_trial(env, int(seed), args.max_steps) for seed in seeds[:count]]
    finally:
        env.close()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "heldout_baseline_episodes.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    successful = sum(bool(row["success"]) for row in rows)
    summary = {
        "seed_manifest": str(args.seed_file.relative_to(ROOT)) if args.seed_file.is_relative_to(ROOT) else str(args.seed_file),
        "manifest_version": manifest["version"],
        "episodes": len(rows),
        "successes": successful,
        "success_rate": successful / len(rows),
        "mean_completion_time_s": float(np.mean([row["completion_time_s"] for row in rows])),
        "mean_final_lateral_error_m": float(np.mean([row["final_lateral_error_m"] for row in rows])),
        "mean_path_length_m": float(np.mean([row["path_length_m"] for row in rows])),
        "baseline": "scripted XY alignment then downward insertion; not an ACT policy",
    }
    json_path = args.output_dir / "heldout_baseline_summary.json"
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Wrote {csv_path} and {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
