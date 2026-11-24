# Project Tasks

**Deadline: Dec 6 [Sunday]** · Scale: `S` small · `M` medium · `L` large · `XL` integration-heavy

## 1. Simulation - Dominik

- [x]  **Build MuJoCo peg-in-hole environment** `L` — **Nov 23 [Monday]**:

  - use a robot model compatible with 6-DoF Cartesian control, randomized initial peg/target configurations, deterministic reset from a supplied seed, and success/failure detection.
  - Expose robot pose, joint state, peg/hole state, and contact state through the agreed schema.
  - Support repeated reset → command → step → success/failure trials without manual intervention.
  - **Handoff to Graeme.**
- [ ]  **Match simulation control constraints to physical deployment** `M` — **Nov 24 [Tuesday]**:

  - realistic joint/workspace limits, bounded Cartesian velocity and acceleration, configurable latency and command filtering.
  - Run the controller at the intended external control frequency rather than MuJoCo's physics timestep, matching the physical Gen3's limits and timing.
  - **Handoff to Graeme.**
- [ ]  **Validate insertion contact behaviour** `M` — **Nov 25 [Wednesday]**:

  - test aligned insertion, edge contact, lateral offset, jamming, and successful insertion.
  - Fix stable contact parameters so identical initial states produce repeatable outcomes; begin with forgiving clearance and reduce it only after repeatable behaviour is established.
  - **Handoff to Graeme and Matvey.**
- [ ]  **(optional) Add domain randomization and define held-out evaluation configurations** `M` — **Randomization Nov 26 [Thursday]; evaluation configs Nov 27 [Friday]**:

  - randomize friction, peg/hole clearance, object pose, command delay, and small state noise.
  - Log all parameters per episode for reproducible but non-identical conditions.
  - Publish a versioned seed/config list of fixed unseen start/goal configurations outside the training demonstration set.
  - **Handoff to Matvey**

## 2. PHANToM and Teleoperation - Graeme

- [ ]  **Read the device** `M` — **Nov 23 [Monday]**:

  - timestamped 6-DoF pose and button/clutch state, with connection-loss detection.
- [ ]  **Map motion and implement clutching** `L` — **Nov 24 [Tuesday]**:

  - calibrate robot-base axes; use relative translation/rotation with separate scales.
  - Releasing the clutch freezes the command; re-engaging resets the origin without jumps.
- [ ]  **Add safety limits** `L` — **Nov 25 [Wednesday]**:

  - deadband, low-pass filtering, velocity bounds, workspace/joint limits, and safe stop on invalid or stale input.
- [ ]  **Validate direct insertion** `L` — **Nov 26 [Thursday]**:

  - complete 10 consecutive usable trials across randomized configurations, with stable contact and no unexplained jumps or axis inversions.
  - **Handoff to Matvey.**

## 3. Demonstrations and Dataset - Matvey

- [ ]  **Build logging and validation** `M` — **Logger Nov 25 [Wednesday]; validation Nov 26 [Thursday]**:

  - synchronize robot/joint and end-effector state, human/executed actions, target/contact state, timestamps, and episode metadata.
  - Verify lossless reload; flag missing/invalid samples, timing gaps, and failures.
  - Keep failed trials with labels.
- [ ]  **Collect and split the dataset** `L` — **Nov 27 [Friday]**:

  - everyone collects 50+ validated successful episodes from randomized starts.
  - Preserve raw data and operator IDs; retain labelled failures.
  - Publish a versioned train/validation manifest with exact episode IDs, excluding held-out evaluation poses from training.

## 4. ACT Policy - Matvey

- [ ]  **Prepare data and verify learning** `M` — **Nov 28 [Saturday]**:

  - deterministic ACT tensors, normalization fitted only on training data, and saved statistics.
  - Overfit 5–10 demonstrations and check that predictions reproduce known trajectories.
- [ ]  **Train the baseline** `XL` — **Nov 29 [Sunday]**:

  - full training set, validation loss tracking, saved checkpoint, and reproducible configuration.
- [ ]  **Run and evaluate ACT in simulation** `L` — **Inference Nov 30 [Monday]; evaluation Dec 1 [Tuesday]**:

  - use the agreed observation/action schema and Graeme's safety layer.
  - Validate the control path with Graeme, then measure success, pose error, path length, collisions, and failure modes on fixed held-out seeds.

## 5. Ensemble and Confidence - Matvey

- [ ]  **Train 3–5 ACT models** `L` — **Dec 1 [Tuesday]**:

  - same architecture/hyperparameters, different initialization seeds; optionally bootstrap demonstrations.
- [ ]  **Return policy action and uncertainty** `M` — **Dec 2 [Wednesday]**:

  - retain individual predictions; average only predictions in the same action mode.
  - Measure normalized translation/rotation disagreement separately and combine into scalar uncertainty.
  - For action chunks, weight near-term disagreement more strongly.

## 6. Shared Autonomy - Matvey

- [ ]  **Blend human and policy commands** `L` — **Dec 1 [Tuesday]**:

  - `u_cmd = (1 - alpha) * u_human + alpha * u_policy`, with `alpha < 1` and adjustable assistance.
- [ ]  **Adjust assistance by task phase** `L` — **Dec 2 [Wednesday]**:

  - smoothly increase assistance near final alignment and keep it low in free space.
- [ ]  **Add safeguards and finish integration** `XL` — **Dec 3 [Thursday]**:

  - reduce assistance for high ensemble uncertainty or strong human-policy disagreement, even when the policy is confident.
  - Log both signals and validate stable shared-control insertion.
  - **Handoff to Dominik.**

## 7. Evaluation - Dominik

- [ ]  **Define the protocol and automate metrics** `M` — **Protocol Nov 29 [Sunday]; metrics Nov 30 [Monday]**:

  - identical held-out configurations, operator instructions, and termination rules across modes.
  - Each trial produces success, completion time, final position error, contact/collision events, path length, and operator corrections/interventions.
- [ ]  **Run comparative trials** `M` — **Everyone: direct Dec 3 [Thursday]; shared Dec 4 [Friday]**:

  - at least 20 valid trials per mode under the same protocol and held-out distribution.
- [ ]  **Analyze results** `L` — **Dec 5 [Saturday]**:

  - compare distributions; classify simulation, teleoperation, policy, arbitration, and operator failures.
  - Produce final plots, summary tables, and representative failure cases.

## 8. Physical Gen3 - Matvey

- [ ]  **Implement the shared backend interface and Kortex adapter** `L` — **Interface Dec 1 [Tuesday]; adapter Dec 2 [Wednesday]**:

  - reuse Graeme's Cartesian API with matching frames and bounded Gen3 twists.
  - Verify zero and low-speed commands.
- [ ]  **Calibrate and validate manual teleoperation** `L` — **Dec 3 [Thursday]**:

  - check every axis, conservative translation/rotation scales, velocity/workspace/joint limits, communication-loss stop, and independent emergency stop.
- [ ]  **Demonstrate physical shared autonomy** `XL` — **Dec 5 [Saturday], only after simulation evaluation is complete**:

  - policy-assisted motion or peg insertion.

## 9. Optional Extensions - Matvey

Start only after the base shared-autonomy system works. Each is `XL`.

- [ ]  **Flow matching:**

  - compare against ACT using the same demonstrations and observation/action representation.
- [ ]  **Interactive imitation:**

  - log operator corrections, retrain, and compare before/after performance.
- [ ]  **Force-aware control:**

  - use physical force/torque sensing to distinguish alignment, contact, and jamming in the policy or arbitration.
- [ ]  **VLA intent selection:**

  - infer the target/skill from language or vision; keep ACT/flow matching responsible for precise motion.

## 10. Final Delivery - Graeme

- [ ]  **Freeze the evaluation revision** — **Dec 4 [Friday]**:

  - tag the implementation; no architectural changes once comparative trials begin.
- [ ]  **Prepare the demo and results** `M` — **Dec 5 [Saturday]**:

  - reliable direct/shared-control demo, recorded backup, and Dominik's final plots/tables.
  - Include physical or optional extensions if ready.
- [ ]  **Submit report and presentation** `L` — **Dec 6 [Sunday]**.
