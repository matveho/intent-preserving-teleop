#!/usr/bin/env python3

# Author: Graeme Aniskowicz
# Date: 2026-11-21

# ===================== Modules ===================== #
import sys
import time
import argparse
from pathlib import Path

import crtk
import arm
import mujoco
import mujoco.viewer

# ===================== Module ====================== #
class example:
    """Connects a PHANToM Omni to a MuJoCo simulation."""

    # Configuration
    def __init__(self, ral, robot_name, expected_interval):
        # --------------------------------------------------
        # PHANToM configuration
        # --------------------------------------------------

        # Create a Python Interface for the PHANToM OMNI
        self.arm = arm.arm(
            ral = ral,
            arm_name = robot_name,
            expected_interval = expected_interval
        )

        # --------------------------------------------------
        # MuJoCo configuration
        # --------------------------------------------------

        # Load robot model
        # project_root = Path(__file__).resolve().parents[1]
        # model_path = project_root / "assets" / "scenes" / "main_scene.xml"
        # self.model = mujoco.MjModel.from_xml_path(str(model_path))

        # Create simulation state
        # self.data = mujoco.MjData(self.model)

        # Find the mocap body used to visualize the PHANToM position.
        # self.marker_id = self.model.body("phantom_target").mocapid

        # if self.marker_id == -1:
        #     raise RuntimeError(
        #         "The MuJoCo model must contain a mocap body named "
        #         "'phantom_target'."
        #     )
        
        # Launch viewer
        # self.viewer = mujoco.viewer.launch(self.model, self.data)

        print("PHANToM + MuJoCo initialized.")
        print("Move the PHANToM to move the marker in MuJoCo.")
        print("Close the MuJoCo viewer to exit.")

        return

    def update_simulation(self):
        """Read the PHANToM pose and update the MuJoCo simulation."""

        # Get the measured Cartesian pose from the PHANToM.
        pose = self.arm.measured_cp()

        # Extract its translation.
        # PyKDL Frame translation is stored in pose.p.
        phantom_position = [
            pose.p[0],
            pose.p[1],
            pose.p[2]
        ]

        # Apply coordinate transformation and scaling here.
        # mujoco_position = phantom_position

        # Update the mocap body's position.
        # self.data.mocap_pos[self.marker_id] = mujoco_position

        # Advance the simulation.
        # mujoco.mj_step(self.model, self.data)

        # Synchronize the viewer with the simulation.
        self.viewer.sync()

        # print(phantom_position)

    # Main Method
    def run(self):
        """Main application loop."""
        try:
            # while self.viewer.is_running():
            while True:
                self.update_simulation()
                # time.sleep(self.model.opt.timestep)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping simulation...")
        # finally:
            # self.viewer.close()

if __name__ == "__main__":
    # Get ROS Arguments
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--arm", type=str, required=True,
                        help = "arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace")
    parser.add_argument("-i", "--interval", type=float, default=0.01,
                        help = "expected interval in seconds between messages sent by the device")
    args = parser.parse_args(argv)

    # Run simulation
    ral = crtk.ral("sensible_phantom_client")
    example = example(ral, args.arm, args.interval)
    ral.spin_and_execute(example.run)