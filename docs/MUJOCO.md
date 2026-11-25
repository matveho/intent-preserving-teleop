# MuJoCo Setup
For the MuJoCo environment, run `python scripts/setup_mujoco.py` from the
repository root and follow [MUJOCO.md](MUJOCO.md).

- Need to add more details below...

# MuJoCo Setup and Verification

Run commands from the repository root. 

If the dependencies or Kinova Gen3 Menagerie assets are not installed yet:

```powershell
python scripts\setup_mujoco.py
```

The model is stored in the repository's root `assets\mujoco_menagerie\kinova_gen3` folder. Setup resolves that path relative to its own location, so it also works if launched from another current directory.

## Watch the scripted insertion

To see the seeded peg-in-hole controller align and insert the peg, run:

```powershell
python scripts\verify_sim.py --viewer-insertion --seed 17
```

The window leaves the peg at its final pose after success. Press **C** to toggle contact points and **F** to toggle contact-force arrows. Close the viewer window to exit. Reuse the same seed to replay the same start and target configuration.

To inspect the initial pose without moving the robot:

```powershell
python scripts\verify_sim.py --viewer-only --seed 17
```

In this mode, use the viewer's **Control** panel to adjust the seven joint sliders. The scripted insertion mode is the better way to observe the automated insertion path.

## Run the full headless verification suite

```powershell
python scripts\verify_sim.py
```

The suite checks:

- Each of the seven joint actuators moves in both directions.
- Cartesian commands move the end effector along the six translation axes.
- Reset is deterministic for a repeated seed and varies for a different seed.
- A scripted reset-command-step trial reaches a terminal status.

For the optional wrist-camera render check (requires a working graphics context):

```powershell
python scripts\verify_sim.py --test-camera
```

To run the full checks and then open the interactive viewer:

```powershell
python scripts\verify_sim.py --viewer
```

`--viewer` shows the robot after the checks; use `--viewer-insertion` to watch the scripted peg insertion itself.

## Current scripted insertion result

The repeatable aligned controller reached `success` for seed 17 on two runs (36.9 mm depth, 0.70 mm lateral error) and for seed 18 (36.9 mm depth, 1.79 mm lateral error). These checks establish that the scripted controller can insert the peg. They do not establish that manual PHANToM control is easy or that all edge-contact and jamming cases are validated. Peak contact force exceeded the configured 40 N collision threshold in those successful runs; review the force threshold and success/failure ordering before treating contact validation as complete.

## Troubleshooting

- **Model not found:** run setup from the repo root; confirm `assets\mujoco_menagerie\kinova_gen3\gen3.xml` exists.
- **MuJoCo import blocked by Windows:** if Windows reports an Application Control signing-level error for a `.pyd` file, check the matching Code Integrity events. Reinstalling into another environment may not avoid the system policy.
- **Viewer fails to open:** try the headless suite first. Camera rendering and interactive viewing require a usable graphics context.
