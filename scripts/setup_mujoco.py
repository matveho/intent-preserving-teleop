#!/usr/bin/env python3
"""Setup script for the MuJoCo + Kinova Gen3 (mujoco_menagerie) simulator.

Run this once inside the shared repository (e.g. ``python3 setup_mujoco.py``):
1. Installs ``mujoco``, ``numpy``, and ``scipy`` into the active Python env.
2. Sparse-clones ``kinova_gen3`` from ``google-deepmind/mujoco_menagerie`` into
   ``assets/mujoco_menagerie/kinova_gen3`` (fast, downloads only Kinova Gen3).
3. Verifies the downloaded ``gen3.xml`` compiles in MuJoCo and inspects every
   joint and actuator (explicitly flagging issues that can prevent Joint 1
   from moving, such as missing armature/damping or unset ctrlrange).
"""

from __future__ import annotations

import argparse
import importlib
import shutil
import subprocess
import sys
from pathlib import Path

MENAGERIE_REPO = "https://github.com/google-deepmind/mujoco_menagerie.git"
REQUIRED_PACKAGES = ["mujoco>=3.1.0", "numpy>=1.24.0", "scipy>=1.10.0"]


def run_cmd(cmd: list[str], cwd: Path | None = None) -> None:
    print(f"[setup] Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True)


def ensure_python_packages(upgrade: bool = False) -> None:
    missing: list[str] = []
    for spec in REQUIRED_PACKAGES:
        pkg_name = spec.split(">=")[0].split("==")[0]
        try:
            importlib.import_module(pkg_name)
        except ImportError:
            missing.append(spec)

    if not missing and not upgrade:
        print("[setup] Python dependencies already installed (mujoco, numpy, scipy).")
        return

    pkgs_to_install = REQUIRED_PACKAGES if upgrade else missing
    cmd = [sys.executable, "-m", "pip", "install"]
    if upgrade:
        cmd.append("--upgrade")
    cmd.extend(pkgs_to_install)
    run_cmd(cmd)


def fetch_kinova_gen3(assets_dir: Path, force: bool = False) -> Path:
    menagerie_dir = assets_dir / "mujoco_menagerie"
    kinova_dir = menagerie_dir / "kinova_gen3"
    gen3_xml = kinova_dir / "gen3.xml"

    if gen3_xml.exists() and not force:
        print(f"[setup] Kinova Gen3 model already present at: {gen3_xml}")
        return kinova_dir

    if menagerie_dir.exists() and force:
        print(f"[setup] Removing existing directory: {menagerie_dir}")
        shutil.rmtree(menagerie_dir)

    assets_dir.mkdir(parents=True, exist_ok=True)

    # Sparse checkout downloads only kinova_gen3 instead of the whole menagerie
    try:
        run_cmd(
            [
                "git",
                "clone",
                "--depth",
                "1",
                "--filter=blob:none",
                "--sparse",
                MENAGERIE_REPO,
                str(menagerie_dir),
            ]
        )
        run_cmd(["git", "sparse-checkout", "set", "kinova_gen3"], cwd=menagerie_dir)
    except subprocess.CalledProcessError:
        print("[setup] Sparse filter clone failed; falling back to shallow clone...")
        if menagerie_dir.exists():
            shutil.rmtree(menagerie_dir)
        run_cmd(["git", "clone", "--depth", "1", MENAGERIE_REPO, str(menagerie_dir)])

    if not gen3_xml.exists():
        raise FileNotFoundError(
            f"Expected {gen3_xml} after cloning mujoco_menagerie, but it was not found."
        )

    print(f"[setup] Successfully fetched Kinova Gen3 to: {kinova_dir}")
    return kinova_dir


def inspect_gen3_model(kinova_dir: Path) -> None:
    import mujoco  # type: ignore

    gen3_xml = kinova_dir / "gen3.xml"
    print(f"\n[verify] Loading {gen3_xml} with MuJoCo {mujoco.__version__}...")
    model = mujoco.MjModel.from_xml_path(str(gen3_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print(
        f"[verify] Loaded OK: nq={model.nq}, nv={model.nv}, "
        f"nu={model.nu}, nbody={model.nbody}, nsite={model.nsite}"
    )

    print("\n--- Joints ---")
    for j in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        qadr = model.jnt_qposadr[j]
        dadr = model.jnt_dofadr[j]
        limited = bool(model.jnt_limited[j])
        jrange = model.jnt_range[j]
        armature = model.dof_armature[dadr]
        damping = model.dof_damping[dadr]
        flag = "  <-- LOW ARMATURE (patched automatically in peg_in_hole_env.py)" if armature < 1e-3 else ""
        print(
            f"  [{j}] {jname:12s} qposadr={qadr} dofadr={dadr} "
            f"limited={str(limited):5s} range=[{jrange[0]:6.2f}, {jrange[1]:6.2f}] "
            f"armature={armature:.3f} damping={damping:.3f}{flag}"
        )

    print("\n--- Actuators ---")
    for a in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        jid = model.actuator_trnid[a, 0]
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        clim = bool(model.actuator_ctrllimited[a])
        crange = model.actuator_ctrlrange[a]
        kp = model.actuator_gainprm[a, 0]
        flag = ""
        if clim and abs(crange[1] - crange[0]) < 1e-6:
            flag = "  <-- ZERO CTRLRANGE (blocks joint movement!)"
        elif not clim:
            flag = "  (unlimited continuous joint; peg_in_hole_env.py sets safe viewer ctrlrange)"
        print(
            f"  [{a}] {aname:16s} -> joint={jname:10s} kp={kp:6.1f} "
            f"ctrllimited={str(clim):5s} ctrlrange=[{crange[0]:6.2f}, {crange[1]:6.2f}]{flag}"
        )

    print("\n[verify] Setup complete. Run `python3 verify_sim.py` next to test Joint 1-7 and Cartesian IK.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Install MuJoCo and fetch Kinova Gen3.")
    parser.add_argument(
        "--assets-dir",
        type=Path,
        # Keep downloaded models in the shared repository, not scripts/assets.
        # This makes setup independent of the shell's current directory and
        # matches KinovaPegInHoleEnv's default model search path.
        default=Path(__file__).resolve().parents[1] / "assets",
        help="Directory where mujoco_menagerie/kinova_gen3 will be stored.",
    )
    parser.add_argument(
        "--skip-pip",
        action="store_true",
        help="Skip pip install step (use if packages are already managed by conda/uv).",
    )
    parser.add_argument(
        "--upgrade",
        action="store_true",
        help="Upgrade mujoco/numpy/scipy even if already installed.",
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        help="Re-download mujoco_menagerie/kinova_gen3 from scratch.",
    )
    args = parser.parse_args()

    if not args.skip_pip:
        ensure_python_packages(upgrade=args.upgrade)
    kinova_dir = fetch_kinova_gen3(args.assets_dir, force=args.force_download)
    inspect_gen3_model(kinova_dir)


if __name__ == "__main__":
    main()
