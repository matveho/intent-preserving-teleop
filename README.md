# Intent-Preserving Learned Policy Arbitration for Contact-Rich Manipulation under Teleoperation

CMPUT 312 final project by Matvey Okoneshnikov, Dominik Vrbanek, and Graeme Aniskowicz.

## Overview

We will build a PHANToM-controlled teleoperation and shared-autonomy system for peg-in-hole insertion with a simulated 6-DoF robot arm.

The operator will demonstrate the task through the PHANToM interface. We will record successful demonstrations, train a Behaviour Cloning policy with Action Chunking with Transformers (ACT), and use the learned policy to assist later operators during precise final alignment.

The core comparison is direct teleoperation versus shared autonomy.

## Motivation

Contact-rich manipulation is difficult for both humans and autonomous robots.

- Pure teleoperation preserves human judgment but is sensitive to scaling, depth perception, and operator skill.
- Pure autonomy can be precise and repeatable but may generalize poorly outside its training distribution.
- Shared autonomy combines human intent with learned assistance.

Peg insertion provides a simple task with tight tolerances, contact, friction, jamming, and uncertainty.

## System

The PHANToM will provide 6-DoF operator input to a simulated 6-DoF robot arm in MuJoCo.

The system will require:

- coordinate-frame calibration
- workspace scaling
- clutching and filtering
- joint and workspace limits
- forward and inverse kinematics
- bounded Cartesian motion
- state-action demonstration logging

Human and policy commands will be combined through state-dependent arbitration:

[
U_{cmd} = (1-\alpha(s))U_{user} + \alpha(s)U_{policy}
]

During free-space motion, the operator retains control. Assistance increases during final alignment when learned corrections are useful.

## Base Goals

1. Build the simulated peg-in-hole task.
2. Map PHANToM motion to safe Cartesian robot control.
3. Complete peg insertion through direct teleoperation.
4. Record at least 50 successful demonstrations.
5. Train an ACT Behaviour Cloning policy.
6. Implement shared autonomy for final alignment.
7. Compare direct teleoperation and shared autonomy.

## Stretch Goals

- Train a Diffusion Policy and compare against Behaviour Cloning.
- Add haptic guidance or forbidden-region virtual fixtures.
- Evaluate pure autonomy.
- Transfer the system to a physical arm.
- Test robustness under latency and unseen peg geometries.

## Evaluation

We will run at least 20 trials per control mode on randomized poses, including held-out configurations.

Metrics include:

- insertion success rate
- completion time
- final position error
- collisions
- robot path length
- number of operator corrections

## Potential Challenges

- PHANToM calibration and coordinate mismatch
- unstable contact simulation
- insufficient demonstrations for generalization
- learned assistance opposing operator intent

## Mitigations

- calibrate coordinate frames and enforce bounded motion
- begin with forgiving insertion tolerances and validate MuJoCo contact behaviour
- randomize training poses and collect additional demonstrations if needed
- limit policy influence and preserve operator override

We will not begin demonstration collection or learned control until teleoperation and simulation are stable.

## Timeline

**Week 1:** Research and setup  
**Week 2:** Simulation, teleoperation, and demonstrations  
**Week 3:** ACT training, GPU sweeps, and shared-control integration  
**Week 4:** Comparative trials, analysis, and final demonstration
