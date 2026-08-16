"""Robot models ported from the legacy C++ implementation.

The original (``includes/robots.hpp``) gave each system two distinct geometries:

* a **collision** body -- a single coarse primitive used for collision checking,
* a **display** model -- a detailed assembly used only for rendering.

That split is preserved here: :func:`collision_shapes` returns the coarse body a
planner should use, and :func:`model_shapes` returns the detailed assembly for
visualization. Dimensions and placements follow the original.

Callisto's cylinders are Y-axis aligned and the original rotated every one of
them by +90 degrees about X (mapping Y onto Z); this module's
:class:`~krrtstar.geometry.Cylinder` is already Z-aligned, so those rotations
collapse to the identity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .geometry import Box, Cylinder, PoseMapping, Shape, Sphere, rotation_from_euler


@dataclass
class ModelPart:
    """One piece of a robot's display model."""

    shape: Shape
    color: str = "dimgray"
    opacity: float = 1.0

# Physical constants from the original dynamicrrt.h
GRAVITY = 9.81
QUAD_MASS = 0.5
QUAD_INERTIA = 0.1
QUAD_LENGTH = 0.3429 / 2.0  # arm length, m

# Quadrotor display dimensions (metres), from robots.hpp
BEAM_WIDTH = 0.015
BEAM_HEIGHT = 0.0065
BEAM_RADIUS = 0.02
MOTOR_RADIUS = 0.015
MOTOR_HEIGHT = 0.02
ROTOR_RADIUS = 0.10
ROTOR_HEIGHT = 0.005
CENTER_SIDE = 0.0889
FLAG_LENGTH = 0.0508


# --------------------------------------------------------------------------- #
# Puck (single / double integrator)
# --------------------------------------------------------------------------- #
def puck_collision(radius: float = 1.25, height: float = 1.0) -> List[Shape]:
    """The ``Puck`` robot: a flat disc. Original default radius is 1.25."""
    return [Cylinder(center=[0.0, 0.0, 0.0], radius=radius, height=height)]


def puck_model(radius: float = 1.25, height: float = 1.0) -> List[ModelPart]:
    # The original's puck model was the collision cylinder, just darker.
    return [ModelPart(puck_collision(radius, height)[0], "dimgray")]


# --------------------------------------------------------------------------- #
# Quadrotor
# --------------------------------------------------------------------------- #
def quadrotor_collision(length: float = QUAD_LENGTH) -> List[Shape]:
    """The coarse collision body: a disc spanning the rotor tips.

    Matches the original ``CAL_CreateCylinder(group, length + rotorRadius,
    length / 2, ...)``.
    """
    return [
        Cylinder(
            center=[0.0, 0.0, 0.0],
            radius=length + ROTOR_RADIUS,
            height=length / 2.0,
        )
    ]


def quadrotor_model(length: float = QUAD_LENGTH) -> List[ModelPart]:
    """The detailed display model: cross beams, motors, rotors, hub, and flag.

    The rotor discs are drawn translucent, as in the original (which set their
    colour alpha to 0.1). That matters for legibility as well as fidelity: the
    rotors span the full arm length, so opaque discs hide the beams, the hub and
    the orange nose flag that marks the forward direction.
    """
    motor_z = BEAM_HEIGHT / 2.0 + MOTOR_HEIGHT / 2.0
    rotor_z = BEAM_HEIGHT / 2.0 + MOTOR_HEIGHT + ROTOR_HEIGHT / 2.0
    arms = [(length, 0.0), (-length, 0.0), (0.0, length), (0.0, -length)]

    parts: List[ModelPart] = [
        # Two crossing beams.
        ModelPart(Box(center=[0, 0, 0], extents=[2 * length, BEAM_WIDTH, BEAM_HEIGHT]), "black"),
        ModelPart(Box(center=[0, 0, 0], extents=[BEAM_WIDTH, 2 * length, BEAM_HEIGHT]), "black"),
        # Central hub, rotated 45 degrees about Z.
        ModelPart(
            Box(
                center=[0, 0, 0],
                extents=[CENTER_SIDE, CENTER_SIDE, BEAM_HEIGHT],
                rotation=rotation_from_euler(0.0, 0.0, np.pi / 4),
            ),
            "dimgray",
        ),
        # Orange nose flag marking the forward (+X) direction.
        ModelPart(
            Box(
                center=[length / 1.65, 0, 0],
                extents=[FLAG_LENGTH, BEAM_WIDTH + 0.001, BEAM_HEIGHT + 0.001],
            ),
            "orangered",
        ),
    ]

    for x, y in arms:
        parts.append(ModelPart(Cylinder(center=[x, y, 0.0], radius=BEAM_RADIUS,
                                        height=BEAM_HEIGHT), "black"))
        parts.append(ModelPart(Cylinder(center=[x, y, motor_z], radius=MOTOR_RADIUS,
                                        height=MOTOR_HEIGHT), "gray"))
        parts.append(ModelPart(Cylinder(center=[x, y, rotor_z], radius=ROTOR_RADIUS,
                                        height=ROTOR_HEIGHT), "steelblue", opacity=0.35))
    return parts


# --------------------------------------------------------------------------- #
# Nonholonomic car
# --------------------------------------------------------------------------- #
def car_collision(centered: bool = False) -> List[Shape]:
    """The ``Nonholonomic`` robot: a 5 x 3 x 2.5 box.

    The original placed the body at ``(2.5, 1.5, 0)`` in the robot frame -- offset
    by half its length and width, so the state's position sits at a corner of the
    body rather than its centre. Because heading rotates the robot frame, the body
    therefore swings *around* the state point instead of turning in place. That is
    reproduced by default for fidelity; pass ``centered=True`` for a body centred
    on the state.
    """
    center = [0.0, 0.0, 0.0] if centered else [2.5, 1.5, 0.0]
    return [Box(center=center, extents=[5.0, 3.0, 2.5])]


def car_model(centered: bool = False) -> List[ModelPart]:
    body = car_collision(centered)[0]
    cx, cy, _ = np.asarray(body.center, float)
    parts: List[ModelPart] = [ModelPart(body, "dimgray")]
    # Wheels are a legibility aid; the original had none. Seat them just outside
    # the hull so they read as wheels rather than blobs sunk into the body.
    wheel_r, wheel_w = 0.5, 0.3
    for dx in (cx - 1.4, cx + 1.4):
        for dy in (cy - 1.5 - wheel_w / 2.0, cy + 1.5 + wheel_w / 2.0):
            parts.append(
                ModelPart(
                    Cylinder(
                        center=[dx, dy, -1.25],
                        radius=wheel_r,
                        height=wheel_w,
                        rotation=rotation_from_euler(np.pi / 2.0, 0.0, 0.0),
                    ),
                    "black",
                )
            )
    return parts


# --------------------------------------------------------------------------- #
# Registry
# --------------------------------------------------------------------------- #
COLLISION_PRESETS = {
    "puck": puck_collision,
    "quadrotor": quadrotor_collision,
    "car": car_collision,
}

MODEL_PRESETS = {
    "puck": puck_model,
    "quadrotor": quadrotor_model,
    "car": car_model,
}

#: Pose mapping each system's state implies (see the example configs).
DEFAULT_POSES: Dict[str, PoseMapping] = {
    # [x, y] or [x, y, vx, vy]: position only.
    "puck": PoseMapping(x=0, y=1),
    # [x, y, z, vx, vy, vz, roll, pitch, roll_rate, pitch_rate]
    "quadrotor": PoseMapping(x=0, y=1, z=2, roll=6, pitch=7),
    # [x, y, theta, v, kappa]
    "car": PoseMapping(x=0, y=1, yaw=2),
}


def collision_shapes(name: str, **kwargs) -> List[Shape]:
    """Coarse collision geometry for a named robot preset."""
    try:
        builder = COLLISION_PRESETS[name]
    except KeyError:
        raise ValueError(
            f"Unknown robot preset {name!r}; available: {sorted(COLLISION_PRESETS)}"
        ) from None
    return builder(**kwargs)


def model_shapes(name: str, **kwargs) -> List[ModelPart]:
    """Detailed display geometry (:class:`ModelPart`) for a named robot preset."""
    try:
        builder = MODEL_PRESETS[name]
    except KeyError:
        raise ValueError(
            f"Unknown robot preset {name!r}; available: {sorted(MODEL_PRESETS)}"
        ) from None
    return builder(**kwargs)


def default_pose(name: str) -> PoseMapping:
    if name not in DEFAULT_POSES:
        raise ValueError(f"Unknown robot preset {name!r}")
    return DEFAULT_POSES[name]
