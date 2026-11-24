#!/usr/bin/env python3
"""Continuous ROS PHANToM pose tracking in MuJoCo (no button required).

Only subscribes to hardware topics; sends no commands/forces to the PHANToM.
"""
import argparse
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from intent_preserving_teleop.schema import EpisodeConfig
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv
from intent_preserving_teleop.teleoperation import PoseSample, PoseTeleop


class PhantomInput(Node):
    def __init__(self, arm):
        super().__init__('phantom_mujoco_teleop')
        self.lock = threading.Lock()
        self.sample = None
        self.stamp = -1
        self.pose_count = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, f'/{arm}/measured_cp', self.pose, qos)

    def pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        sample = PoseSample(np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]), time.monotonic())
        with self.lock:
            self.pose_count += 1
            if (stamp <= self.stamp or not np.isfinite(np.r_[sample.position, sample.quaternion_xyzw]).all()
                    or np.linalg.norm(sample.quaternion_xyzw) < 1e-6):
                self.sample = None
            else:
                self.sample = sample
            self.stamp = stamp

    def snapshot(self):
        with self.lock:
            return self.sample, self.pose_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', default='arm')
    parser.add_argument('--kinova-dir', type=Path)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0, help='Exit after this many seconds; 0 runs until closed.')
    parser.add_argument('--status-file', type=Path, help='Optional live diagnostic JSON, not a demonstration log.')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('Duration must be finite and nonnegative.')
    config = EpisodeConfig(seed=args.seed)
    config.domain_randomization.enabled = False
    config.success_criteria.max_episode_steps = 50 * 600
    env = KinovaPegInHoleEnv(kinova_dir=args.kinova_dir, config=config, nullspace_gain=0.0)
    obs = env.reset(seed=args.seed)
    mapper = PoseTeleop(config.control_limits)
    reset_requested = threading.Event()
    def key(keycode):
        if keycode in (82, 114):
            reset_requested.set()
    rclpy.init()
    node = PhantomInput(args.arm.strip('/'))
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    viewer = None
    started = time.monotonic()
    last_report = 0
    steps = 0
    moving_steps = 0
    episode_status = "running"
    mode = ''
    try:
        if not args.headless:
            viewer = mujoco.viewer.launch_passive(env.model, env.data, key_callback=key)
            viewer.cam.lookat[:] = [0.4, 0.0, 0.2]
            viewer.cam.distance = 1.4
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
        print('Continuous pose tracking: move/rotate stylus; no button needed. R resets the simulation. Close viewer to exit.', flush=True)
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            if reset_requested.is_set():
                reset_requested.clear()
                # Randomization disabled: reset preserves the model used by the viewer.
                if viewer:
                    with viewer.lock(): obs = env.reset(seed=args.seed)
                else: obs = env.reset(seed=args.seed)
                episode_status = "running"
            sample, count = node.snapshot()
            command = mapper.command(sample, obs, now)
            new_mode = 'tracking pose' if command is not None else 'waiting for valid ROS pose'
            if new_mode != mode:
                print(new_mode, flush=True)
                mode = new_mode
            if command is not None:
                if viewer:
                    with viewer.lock(): result = env.step(command)
                else: result = env.step(command)
                obs = result.observation
                steps += 1
                moving_steps += int(np.linalg.norm(result.executed_twist) > 1e-6)
                # Interactive preview continues after trial success/failure;
                # evaluation semantics remain unchanged in the backend.
                episode_status = result.status.value
            else:
                # Freeze physics and discard pending motion on invalid/stale input.
                env._prev_twist[:] = 0
                env._filtered_twist[:] = 0
                env._cmd_queue.clear()
                env.data.qvel[:] = 0
            if viewer: viewer.sync()
            if now - last_report >= 1:
                report = dict(mode=mode, episode_status=episode_status, poses_received=count, control_steps=steps,
                              moving_steps=moving_steps, ee_position_m=obs.ee_pos.tolist(),
                              target_position_m=None if mapper.target_position is None else mapper.target_position.tolist(),
                              target_quaternion_xyzw=None if mapper.target_rotation is None else mapper.target_rotation.as_quat().tolist(),
                              ee_quaternion_wxyz=obs.ee_quat.tolist(),
                              pose_age_s=None if sample is None else now-sample.received_at)
                if args.status_file:
                    args.status_file.parent.mkdir(parents=True, exist_ok=True)
                    temp = args.status_file.with_suffix('.tmp')
                    temp.write_text(json.dumps(report, indent=2)+'\n')
                    temp.replace(args.status_file)
                last_report = now
            time.sleep(max(0, config.control_limits.control_dt - (time.monotonic()-now)))
    except KeyboardInterrupt:
        pass
    finally:
        if viewer: viewer.close()
        rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        env.close()


if __name__ == '__main__':
    main()
