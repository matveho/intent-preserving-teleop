# Insertion layout randomization

`insertion_layout` in `DomainRandomizationConfig` selects which existing
fixture the episode uses:

- `"top"`: peg approaches down into the table-mounted hole (default).
- `"horizontal"`: peg approaches horizontally into the hole in the vertical
  wall.
- `"random"`: choose top or horizontal per reset, reproducibly from the episode
  seed.

Example:

```python
from intent_preserving_teleop.schema import DomainRandomizationConfig, EpisodeConfig
from intent_preserving_teleop.simulation import KinovaPegInHoleEnv

config = EpisodeConfig(
    seed=7,
    domain_randomization=DomainRandomizationConfig(insertion_layout="random"),
)
env = KinovaPegInHoleEnv(config=config)
obs = env.reset(seed=7)
print(env.episode_params["insertion_layout"])
```

The `domain_randomization` package contains the seeded layout resolver. The
environment continues to randomize hole placement, yaw, friction, clearance,
lighting, and latency using the configured ranges.
