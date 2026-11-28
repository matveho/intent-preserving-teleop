"""Seeded selection of the peg-in-hole insertion orientation."""

from typing import Literal

import numpy as np

InsertionLayout = Literal["top", "horizontal", "random"]


def resolve_insertion_layout(layout: str, rng: np.random.Generator) -> str:
    """Return the concrete environment hole key for a configured layout.

    ``horizontal`` uses the existing side hole in the vertical wall. ``random``
    samples equally between top entry and horizontal side entry using ``rng``,
    so the result is reproducible when the episode seed is fixed.
    """
    if layout == "top":
        return "top"
    if layout == "horizontal":
        return "side"
    if layout == "random":
        return ("top", "side")[int(rng.integers(0, 2))]
    raise ValueError(
        f"insertion_layout must be 'top', 'horizontal', or 'random'; got {layout!r}"
    )
