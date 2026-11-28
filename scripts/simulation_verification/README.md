# Dominik's simulation validation tools

Run from the repository root after installing the project dependencies and making sure the Kinova Gen3 MuJoCo model exists under `assets/mujoco_menagerie/kinova_gen3`.

```powershell
python scripts\simulation_verification\check_control_constraints.py
python scripts\simulation_verification\validate_contact_cases.py --episodes 3
python scripts\simulation_verification\evaluate_heldout.py --episodes 20
```

The tools are intended to make the remaining simulation work repeatable:

- `check_control_constraints.py` drives deliberately oversized commands and reports the observed twist/acceleration bounds and simulated control interval.
- `validate_contact_cases.py` runs scripted aligned, edge-offset, lateral-offset, and sustained-offset insertion attempts, saving per-step contact evidence and per-episode outcomes. These are diagnostic probes; inspect the CSV to confirm each case actually produced the intended contact condition.
- `evaluate_heldout.py` evaluates the existing scripted alignment baseline on the fixed seed list in `heldout_v1.json`, saving episode-level outcomes and metrics. This is a simulator/controller check, not an ACT evaluation.

The held-out seeds are reserved for evaluation. Do not use these seeds to collect training demonstrations. The JSON file records the list and its version; change the version if the configurations change.

Outputs default to `outputs/simulation_verification/`, which is ignored by Git.
