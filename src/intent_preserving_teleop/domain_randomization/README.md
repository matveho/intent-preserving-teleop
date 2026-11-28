# Insertion layout and domain randomization

This package documents and implements the per-episode choice of insertion
orientation. The task has two configurations:

- **Top entry**: the hole is in the table and the peg approaches downward.
- **Horizontal entry**: the hole is in a vertical wall and the peg approaches
  horizontally through the wall opening.

`"vertical"` describes the wall in the horizontal-entry configuration; the
configuration name is `"horizontal"` because that is the peg's insertion
direction. The existing `side` hole key in the simulation is the horizontal
configuration.

## Episode setting and seeded selection

Set `DomainRandomizationConfig.insertion_layout` to one of:

| Value | Episode behavior |
| --- | --- |
| `"top"` | Always use the table-mounted top-entry hole. This is the default. |
| `"horizontal"` | Always use the hole in the vertical wall. |
| `"random"` | Choose top or horizontal once on each reset, with equal probability. |

The choice uses the episode's NumPy random generator. Repeating the same reset
seed and configuration reproduces the selected layout and the other randomized
episode parameters. A new reset seed can produce a different layout. Set
`DomainRandomizationConfig.enabled=False` to disable the continuous parameter
randomization; the explicit `insertion_layout` mode still selects the requested
configuration.

```python
from intent_preserving_teleop.schema import DomainRandomizationConfig, EpisodeConfig
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv

config = EpisodeConfig(
    seed=7,
    domain_randomization=DomainRandomizationConfig(insertion_layout="random"),
)
env = KinovaPegInHoleEnv(config=config)
obs = env.reset(seed=7)
print(env.episode_params["insertion_layout"])  # "top" or "horizontal"
```

The existing wall fixture is enabled automatically for `"horizontal"` and
`"random"`. The mode can also be overridden for one reset with
`env.reset(seed=7, hole="top")` or `env.reset(seed=7, hole="horizontal")`.

## Keep the insertion setup coherent

The selected layout must determine every quantity that defines the task, not
just the visible wall. For a generalized rotating-fixture implementation,
derive the wall pose, hole entry pose, support geometry, peg start target, and
approach axis from one sampled layout transform. Apply the transform to visual
and collision geometry together. Keep the table fixed if the intended change is
to rotate the fixture while leaving the work surface in place.

The peg should start at a seeded standoff along the selected hole's entry axis,
with its insertion axis pointing opposite that axis. Lateral start offsets
should be sampled in the hole's local plane, not hard-coded to world X/Y. This
lets the same setup logic work for both downward and horizontal insertion.

## Current environment behavior

The environment already has separate top and wall-mounted hole assemblies. It
does **not** currently rotate one shared wall-and-hole fixture: top entry uses
the table hole, while horizontal entry selects the existing side hole and
vertical wall. The model keeps the table fixed. For either selected hole, reset
randomizes hole position in that fixture's plane, rotates the hole about its
entry axis, sets the peg start along the hole axis, and aligns the peg to that
axis. The observation expresses peg-to-hole displacement in the active hole
frame, and success checks use local lateral error, insertion depth, and peg to
hole axis tilt. Contact checks classify peg contact against hole geometry and
the wall or table obstacles. Because the model can contain both hole assemblies,
future contact-based criteria should distinguish the active hole from the
inactive one.

The existing `DomainRandomizationConfig` bounds also cover:

- hole position within the table plane (top) or wall plane (horizontal),
- hole yaw about its own entry axis,
- peg start standoff and local lateral offset,
- peg/hole friction and hole clearance,
- key-light position and intensity/tint, and
- command latency steps.

Each episode records its selected `insertion_layout`, active hole, sampled hole
pose, start target, friction, clearance, latency, and lighting in
`env.episode_params`; step results include these parameters for logging.

## Viewer, training, and verification

Training and data collection should construct the environment with a fixed
layout when producing separate top and horizontal datasets, or with
`insertion_layout="random"` when mixing both configurations. Record the layout
from `episode_params` with every trajectory so examples can be stratified and
replayed with their original seed.

Verification should reset at fixed seeds for `"top"`, `"horizontal"`, and
`"random"`, checking that the selected geometry, peg start pose, observation
frame, contact classification, and success metrics agree with the active hole.
The viewer's scripted insertion should compute its tracking error in the hole
frame (or use the full world-space hole-to-peg vector) so it can replay both
approach directions. The current `verify_sim.py` insertion demo explicitly
resets to `hole="top"` and computes only world X/Y error, so it remains a
top-entry visualization until that script is generalized.

## Module API

`resolve_insertion_layout(layout, rng)` is the small deterministic selector
used by the environment. It accepts `"top"`, `"horizontal"`, or `"random"`
and returns the concrete simulation hole key (`"top"` or `"side"`). Pass the
episode's generator; do not create an unseeded generator inside the resolver.
