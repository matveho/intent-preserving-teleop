# Local PHANToM → MuJoCo teleoperation

The bridge continuously reads `/arm/measured_cp` and tracks a position and orientation target through the simulator's Cartesian controller at 50 Hz. **No button is required.** It sends no commands to the physical PHANToM or a physical robot.

Position uses a fixed, absolute mapping: PHANToM right/up/towards the operator maps to robot right/up/backwards. The Omni configuration's full geometric bounds (X: −210 to 210 mm, Y: −110 to 205 mm, Z: −85 to 130 mm) map to 95% of the simulator's workspace box. The resulting gains are approximately 2.43× along robot X, 1.81× along Y, and 1.90× along Z. Targets outside that box are clamped. These configured bounds are an initial workspace estimate, not a measured calibration of the operator's comfortable range.

Rotation is **1:1**, using a proper rotation-matrix frame conversion rather than Euler-angle scaling or accumulated increments. The fixed tool-frame correction maps the driver's stylus tip axis (local −Z) onto the peg axis (local +Z). A vertical stylus therefore commands a vertical peg regardless of the startup pose. Returning to the same stylus pose produces the same target. The arm tracks that target subject to its existing velocity, acceleration, joint, contact, and workspace constraints; some position/orientation combinations are unreachable.

Press **R** in MuJoCo to reset the episode. Stale/invalid ROS data freezes simulation. This interactive preview continues through contact and trial success/failure so the operator can withdraw the peg; trial status remains in diagnostics. The backend retains its original termination criteria for evaluation. Closing the viewer stops the bridge. Workspace shifting and clutch behavior are intentionally absent for this local prototype.

On `csc-305-w06`, the environment and model are already prepared. In one desktop terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/mokoneshnikov/environments/phantom-ros-build-check/install/setup.bash
export PHANTOM_CONFIG_FILES=/home/mokoneshnikov/phantom-config
ros2 run sensable_phantom sensable_phantom -j /home/mokoneshnikov/phantom-config/sawSensablePhantom.json
```

Do not start a second device node if this one is already running. In a second desktop terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/mokoneshnikov/environments/phantom-ros-build-check/install/setup.bash
/home/mokoneshnikov/environments/phantom-mujoco/bin/python /home/mokoneshnikov/projects/intent-preserving-teleop/scripts/run_teleop.py --kinova-dir /home/mokoneshnikov/environments/phantom-mujoco/assets/mujoco_menagerie/kinova_gen3
```

On another prepared ROS machine, use a Python 3.12 virtual environment created with `--system-site-packages`, install `mujoco==3.3.7` and `scipy` there, and use `scripts/setup_mujoco.py` to download the Kinova model. Source Jazzy and your built ROS workspace before running `scripts/run_teleop.py`. Use your own device configuration and paths.
