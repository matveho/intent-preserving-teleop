#!/usr/bin/env python3
"""Run repeatable diagnostic probes for insertion contact behaviors.

The offset probes are deliberately not assumed to produce edge contact or a
jam. The per-step contact log is the evidence used to confirm what happened.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from intent_preserving_teleop.schema import CartesianCommand, EpisodeStatus  # noqa: E402
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv  # noqa: E402


CASES = {
    "aligned": (np.array([0.0, 0.0]), False),
    "edge_offset": (np.array([0.004, 0.0]), False),
    "lateral_offset": (np.array([0.010, 0.0]), False),
    "sustained_offset": (np.array([0.004, 0.0]), True),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=2201)
    parser.add_argument("--episodes", type=int, default=1, help="Repeat each case with incrementing seeds.")
    parser.add_argument("--max-steps", type=int, default=700)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "outputs" / "simulation_verification"
    )
    args = parser.parse_args()
    if args.episodes < 1 or args.max_steps < 1:
        parser.error("--episodes and --max-steps must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    step_path = args.output_dir / "contact_case_steps.csv"
    episode_path = args.output_dir / "contact_case_episodes.csv"
    step_fields = [
        "case", "seed", "step", "time_s", "status", "ee_x", "ee_y", "ee_z",
        "lateral_error_m", "insertion_depth_m", "contact", "peg_hole_contact",
        "table_collision", "contact_count", "force_n", "jammed", "command_vx",
        "command_vy", "command_vz",
    ]
    episode_fields = [
        "case", "seed", "status", "steps", "succeeded", "peg_hole_contact_seen",
        "table_collision_seen", "jam_seen", "max_contact_force_n",
        "final_lateral_error_m", "final_insertion_depth_m",
    ]
    with step_path.open("w", newline="", encoding="utf-8") as sf, episode_path.open(
        "w", newline="", encoding="utf-8"
    ) as ef:
        step_writer = csv.DictWriter(sf, fieldnames=step_fields)
        episode_writer = csv.DictWriter(ef, fieldnames=episode_fields)
        step_writer.writeheader()
        episode_writer.writeheader()
        env = KinovaPegInHoleEnv()
        try:
            for case_name, (offset_xy, hold_offset) in CASES.items():
                for episode in range(args.episodes):
                    seed = args.seed + episode
                    obs = env.reset(seed=seed, hole="top")
                    hole_target = obs.hole_pos.copy()
                    hole_target[:2] += offset_xy
                    contacts: list[bool] = []
                    collisions: list[bool] = []
                    jammed: list[bool] = []
                    forces: list[float] = []
                    status = EpisodeStatus.RUNNING
                    steps = 0
                    for steps in range(1, args.max_steps + 1):
                        delta_xy = hole_target[:2] - obs.peg_pos[:2]
                        if np.linalg.norm(delta_xy) > 0.002:
                            vx = float(np.clip(2.0 * delta_xy[0], -0.12, 0.12))
                            vy = float(np.clip(2.0 * delta_xy[1], -0.12, 0.12))
                            vz = 0.0
                        else:
                            # The sustained-offset case keeps pressing after reaching its offset target.
                            vx, vy, vz = 0.0, 0.0, -0.035
                        result = env.step(
                            CartesianCommand(
                                linear=np.array([vx, vy, vz]),
                                angular=np.zeros(3),
                                source="script",
                            )
                        )
                        obs, status = result.observation, result.status
                        c = obs.contact
                        contacts.append(c.peg_hole_contact)
                        collisions.append(c.table_collision)
                        jammed.append(c.is_jammed)
                        forces.append(c.total_contact_force_n)
                        step_writer.writerow(
                            {
                                "case": case_name, "seed": seed, "step": steps,
                                "time_s": obs.timestamp, "status": status.value,
                                "ee_x": obs.ee_pos[0], "ee_y": obs.ee_pos[1], "ee_z": obs.ee_pos[2],
                                "lateral_error_m": np.linalg.norm(obs.peg_to_hole_local[:2]),
                                "insertion_depth_m": obs.peg_to_hole_local[2],
                                "contact": c.in_contact, "peg_hole_contact": c.peg_hole_contact,
                                "table_collision": c.table_collision, "contact_count": c.num_contacts,
                                "force_n": c.total_contact_force_n, "jammed": c.is_jammed,
                                "command_vx": vx, "command_vy": vy, "command_vz": vz,
                            }
                        )
                        if result.done:
                            break
                    episode_writer.writerow(
                        {
                            "case": case_name, "seed": seed, "status": status.value,
                            "steps": steps, "succeeded": status == EpisodeStatus.SUCCESS,
                            "peg_hole_contact_seen": any(contacts),
                            "table_collision_seen": any(collisions), "jam_seen": any(jammed),
                            "max_contact_force_n": max(forces, default=0.0),
                            "final_lateral_error_m": float(np.linalg.norm(obs.peg_to_hole_local[:2])),
                            "final_insertion_depth_m": float(obs.peg_to_hole_local[2]),
                        }
                    )
                    print(
                        f"{case_name} seed={seed}: {status.value}, "
                        f"peg-hole contact={any(contacts)}, jam={any(jammed)}, "
                        f"peak force={max(forces, default=0.0):.2f} N"
                    )
        finally:
            env.close()
    print(f"Wrote {step_path} and {episode_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
