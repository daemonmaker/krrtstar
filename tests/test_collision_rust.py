"""Cross-checks of the native (Rust/parry3d) collision scene.

Every test compares the native backend against the pure-Python GJK narrow
phase in :mod:`krrtstar.collision`, which is the reference semantics. The two
may legitimately disagree only for poses that sit exactly on a contact surface,
so states whose signed clearance is within :data:`SURFACE_TOL` of zero are
excluded from the comparison.
"""

from __future__ import annotations

import numpy as np
import pytest

from krrtstar import accel
from krrtstar.collision import shapes_distance
from krrtstar.geometry import (
    Box,
    Capsule,
    CollisionChecker,
    Cylinder,
    Environment,
    Mesh,
    PoseMapping,
    Robot,
    Sphere,
    rotation_from_euler,
)

BOUNDS = np.array([[-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]])

#: Half-width of the ambiguous band around a contact surface.
SURFACE_TOL = 1e-6


def _native_scene_available() -> bool:
    if not hasattr(accel, "make_collision_scene"):
        return False
    robot = Robot(shapes=[Sphere([0.0, 0.0, 0.0], 0.5)], pose=PoseMapping(x=0, y=1, z=2))
    env = Environment(bounds=BOUNDS, obstacles=[Sphere([1.0, 0.0, 0.0], 0.5)])
    return accel.make_collision_scene(robot, env) is not None


pytestmark = pytest.mark.skipif(
    not _native_scene_available(),
    reason="native collision scene unavailable (build rust/ with maturin)",
)


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def _core_only(shape):
    """Copy of ``shape`` with its margin removed, exposing the core set."""
    if isinstance(shape, Sphere):
        return Sphere(shape.center, 0.0)
    if isinstance(shape, Capsule):
        return Capsule(shape.center, 0.0, shape.height, shape.rotation)
    return shape  # boxes, cylinders and meshes already have a zero margin


def signed_clearance(robot: Robot, env: Environment, state) -> float:
    """Signed distance from the robot to the nearest obstacle surface.

    ``shapes_distance`` clamps overlap to zero, which cannot distinguish a
    grazing contact from deep penetration. Comparing the *core* distance
    against the summed margins keeps the sign, so the result is negative when
    the shapes overlap and zero exactly on contact.
    """
    best = np.inf
    for shape in robot.placed_shapes(state):
        for obstacle in env.obstacles:
            core = shapes_distance(_core_only(shape), _core_only(obstacle))
            best = min(best, core - (float(shape.margin) + float(obstacle.margin)))
    return float(best)


def both_backends(robot: Robot, env: Environment):
    """The native and pure-Python checkers for one robot/environment pair."""
    native = CollisionChecker(robot, env, use_accel=True)
    python = CollisionChecker(robot, env, use_accel=False)
    assert native.backend == "rust"
    assert python.backend == "python"
    return native, python


def assert_agree(robot, env, native, python, states):
    """Both backends agree on every state outside the contact-surface band.

    Returns ``(collisions, skipped)`` counted over ``states``.
    """
    collisions = 0
    skipped = 0
    for state in states:
        got = native.in_collision(state)
        want = python.in_collision(state)
        if got != want:
            gap = signed_clearance(robot, env, state)
            assert abs(gap) <= SURFACE_TOL, (
                f"backends disagree away from a contact surface at state {state}: "
                f"rust={got}, python={want}, signed clearance={gap!r}"
            )
            skipped += 1
            continue
        collisions += int(want)
    return collisions, skipped


def sphere_robot(radius: float = 0.3) -> Robot:
    return Robot(shapes=[Sphere([0.0, 0.0, 0.0], radius)], pose=PoseMapping(x=0, y=1, z=2))


def sweep_states(
    seed: int,
    obstacles=(),
    extent: float = 2.6,
    grid_n: int = 6,
    n_random: int = 220,
    n_near: int = 220,
    pad: float = 0.5,
):
    """A deterministic grid plus random positions, as ``(n, 6)`` states.

    ``n_near`` extra positions are drawn from each obstacle's AABB inflated by
    ``pad``, so a good share of the sweep lands close to a contact surface
    instead of far out in free space.

    The trailing three components are velocities the pose mapping ignores, so
    the sweep also exercises the state -> pose index mapping.
    """
    rng = np.random.default_rng(seed)
    axis = np.linspace(-extent, extent, grid_n)
    grid = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
    blocks = [grid, rng.uniform(-extent, extent, size=(n_random, 3))]
    for obstacle in obstacles:
        lo, hi = obstacle.aabb()
        blocks.append(rng.uniform(lo - pad, hi + pad, size=(n_near, 3)))
    positions = np.vstack(blocks)
    velocities = rng.normal(size=(positions.shape[0], 3))
    return np.hstack([positions, velocities])


def obstacle_cases() -> dict:
    """One environment per obstacle shape type."""
    return {
        "sphere": Sphere([0.2, -0.1, 0.3], 1.1),
        "box": Box([0.0, 0.0, 0.0], [2.0, 1.2, 1.6]),
        "oriented_box": Box(
            [0.4, -0.3, 0.1], [2.4, 1.0, 1.4], rotation_from_euler(yaw=np.pi / 4)
        ),
        "capsule": Capsule([0.1, 0.2, 0.0], 0.45, 1.8, rotation_from_euler(pitch=0.4)),
        "cylinder": Cylinder([0.0, 0.3, -0.2], 0.7, 1.5, rotation_from_euler(roll=0.6)),
        # A single triangle: an open, zero-thickness mesh.
        "triangle_mesh": Mesh(
            vertices=[[-1.0, -1.0, 0.0], [1.5, -0.5, 0.0], [0.0, 1.5, 0.5]],
            faces=[[0, 1, 2]],
            center=[0.0, 0.0, 0.2],
            rotation=rotation_from_euler(roll=0.2, yaw=0.5),
            scale=1.2,
        ),
        # A closed shell, which exercises the multi-triangle dispatch. The
        # narrow phase is per-triangle on both sides, so only the shell itself
        # collides, not the enclosed volume.
        "tetrahedron_mesh": Mesh(
            vertices=[[0.0, 0.0, 0.0], [1.6, 0.0, 0.0], [0.0, 1.6, 0.0], [0.0, 0.0, 1.6]],
            faces=[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            center=[-0.6, -0.6, -0.4],
            rotation=rotation_from_euler(pitch=0.3, yaw=0.8),
            scale=1.1,
        ),
    }


# --------------------------------------------------------------------------- #
# Backend agreement over a randomized sweep
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("name,obstacle", sorted(obstacle_cases().items()))
def test_backends_agree_per_obstacle_type(name, obstacle):
    robot = sphere_robot(0.3)
    env = Environment(bounds=BOUNDS, obstacles=[obstacle])
    native, python = both_backends(robot, env)

    states = sweep_states(seed=20240816 + len(name), obstacles=env.obstacles)
    collisions, skipped = assert_agree(robot, env, native, python, states)

    n = len(states)
    # The sweep must straddle the surface, otherwise agreement is vacuous.
    assert min(collisions, n - collisions) >= 25, (
        f"{name}: sweep did not span free and colliding regions "
        f"({collisions}/{n} colliding)"
    )
    assert skipped <= 0.01 * n, f"{name}: too many ambiguous states ({skipped}/{n})"


def test_backends_agree_with_multiple_obstacles():
    robot = sphere_robot(0.35)
    obstacles = list(obstacle_cases().values())
    # Spread the obstacles out so several are reachable but not all overlapping.
    offsets = [
        [-2.0, -2.0, 0.0],
        [2.0, 2.0, 0.5],
        [-2.5, 2.0, -0.5],
        [2.5, -2.0, 0.0],
        [0.0, 0.0, 2.0],
        [0.0, 0.0, -2.0],
        [0.0, 2.5, 0.0],
    ]
    assert len(offsets) == len(obstacles)
    env = Environment(
        bounds=BOUNDS,
        obstacles=[o.translated(d) for o, d in zip(obstacles, offsets)],
    )
    native, python = both_backends(robot, env)

    states = sweep_states(
        seed=99, obstacles=env.obstacles, extent=3.4, grid_n=7, n_random=200, n_near=80
    )
    collisions, skipped = assert_agree(robot, env, native, python, states)
    assert min(collisions, len(states) - collisions) >= 25
    assert skipped <= 0.01 * len(states)


# --------------------------------------------------------------------------- #
# Axis convention: capsules and cylinders are Z-aligned in the Python model
# --------------------------------------------------------------------------- #
def _axis_env(obstacle):
    robot = sphere_robot(0.5)
    return robot, Environment(bounds=BOUNDS, obstacles=[obstacle])


@pytest.mark.parametrize(
    "obstacle",
    [
        # Inner segment of length 4 -> the caps reach z = +-2.5.
        Capsule([0.0, 0.0, 0.0], 0.5, 4.0),
        # Full height of 4 -> the flat caps sit at z = +-2.
        Cylinder([0.0, 0.0, 0.0], 0.5, 4.0),
    ],
    ids=["capsule", "cylinder"],
)
def test_z_axis_convention(obstacle):
    robot, env = _axis_env(obstacle)
    native, python = both_backends(robot, env)

    on_axis = np.array([0.0, 0.0, 2.4, 0.0, 0.0, 0.0])
    off_axis = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Would only collide if the shape's axis had been left on Y (parry's own
    # convention) instead of being mapped back to Z.
    swapped_axis = np.array([0.0, 2.4, 0.0, 0.0, 0.0, 0.0])
    # Would only collide if `height` had been read as a half-height.
    beyond_cap = np.array([0.0, 0.0, 3.5, 0.0, 0.0, 0.0])

    for state, expected in [
        (on_axis, True),
        (off_axis, False),
        (swapped_axis, False),
        (beyond_cap, False),
    ]:
        assert python.in_collision(state) is expected
        assert native.in_collision(state) is expected


def test_capsule_height_excludes_end_caps():
    """A capsule's ``height`` is the inner segment; a cylinder's is the total."""
    robot = sphere_robot(0.25)
    capsule = Environment(bounds=BOUNDS, obstacles=[Capsule([0.0] * 3, 0.5, 4.0)])
    cylinder = Environment(bounds=BOUNDS, obstacles=[Cylinder([0.0] * 3, 0.5, 4.0)])

    cap_native, cap_python = both_backends(robot, capsule)
    cyl_native, cyl_python = both_backends(robot, cylinder)

    # Capsule reaches z = 2.5, cylinder only z = 2, so z = 2.6 separates them.
    state = np.array([0.0, 0.0, 2.6, 0.0, 0.0, 0.0])
    assert cap_python.in_collision(state) is True
    assert cap_native.in_collision(state) is True
    assert cyl_python.in_collision(state) is False
    assert cyl_native.in_collision(state) is False


# --------------------------------------------------------------------------- #
# The robot's orientation must be composed, not just its translation
# --------------------------------------------------------------------------- #
def oriented_robot(shapes) -> Robot:
    return Robot(shapes=shapes, pose=PoseMapping(x=0, y=1, z=2, yaw=3))


def test_robot_orientation_is_honoured():
    robot = oriented_robot([Box([0.0, 0.0, 0.0], [4.0, 0.4, 0.4])])
    env = Environment(bounds=BOUNDS, obstacles=[Sphere([1.5, 0.0, 0.0], 0.2)])
    native, python = both_backends(robot, env)

    aligned = np.array([0.0, 0.0, 0.0, 0.0])
    turned = np.array([0.0, 0.0, 0.0, np.pi / 2])

    assert python.in_collision(aligned) is True
    assert native.in_collision(aligned) is True
    assert python.in_collision(turned) is False
    assert native.in_collision(turned) is False


def test_robot_shape_offset_rotates_with_the_robot():
    """A shape's own centre is rotated by the query, not merely translated."""
    robot = oriented_robot([Sphere([1.5, 0.0, 0.0], 0.2)])
    env = Environment(bounds=BOUNDS, obstacles=[Sphere([1.5, 0.0, 0.0], 0.2)])
    native, python = both_backends(robot, env)

    for yaw, expected in [(0.0, True), (np.pi / 2, False), (np.pi, False)]:
        state = np.array([0.0, 0.0, 0.0, yaw])
        assert python.in_collision(state) is expected
        assert native.in_collision(state) is expected


def test_backends_agree_for_an_oriented_robot():
    robot = oriented_robot(
        [Box([0.0, 0.0, 0.0], [3.0, 0.5, 0.5]), Sphere([1.2, 0.0, 0.0], 0.4)]
    )
    env = Environment(
        bounds=BOUNDS,
        obstacles=[
            Box([1.6, 1.6, 0.0], [1.0, 1.0, 1.0], rotation_from_euler(yaw=np.pi / 4)),
            Cylinder([-1.8, 0.6, 0.0], 0.5, 2.0, rotation_from_euler(pitch=0.3)),
            Capsule([0.0, -2.2, 0.2], 0.4, 1.4),
        ],
    )
    native, python = both_backends(robot, env)

    rng = np.random.default_rng(7)
    positions = rng.uniform(-3.0, 3.0, size=(260, 3))
    yaws = rng.uniform(-np.pi, np.pi, size=(260, 1))
    states = np.hstack([positions, yaws])

    collisions, skipped = assert_agree(robot, env, native, python, states)
    assert min(collisions, len(states) - collisions) >= 25
    assert skipped <= 0.01 * len(states)


# --------------------------------------------------------------------------- #
# Trajectories
# --------------------------------------------------------------------------- #
def test_trajectory_in_collision_agrees():
    robot = sphere_robot(0.3)
    env = Environment(
        bounds=BOUNDS,
        obstacles=[
            Sphere([0.0, 0.0, 0.0], 0.8),
            Cylinder([2.5, 0.0, 0.0], 0.4, 1.2),
            Capsule([-2.5, 0.0, 0.0], 0.3, 1.0, rotation_from_euler(pitch=np.pi / 2)),
        ],
    )
    native, python = both_backends(robot, env)

    def line(start, end, n=25):
        ts = np.linspace(0.0, 1.0, n)[:, None]
        positions = np.asarray(start, float) + ts * (
            np.asarray(end, float) - np.asarray(start, float)
        )
        return np.hstack([positions, np.zeros((n, 3))])

    trajectories = {
        "through_the_sphere": (line([-4.0, 0.0, 0.0], [4.0, 0.0, 0.0]), True),
        "clear_above": (line([-4.0, 0.0, 3.0], [4.0, 0.0, 3.0]), False),
        "grazes_the_cylinder": (line([2.5, -4.0, 0.0], [2.5, 4.0, 0.0]), True),
        "collides_at_the_first_pose": (line([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]), True),
        "collides_at_the_last_pose": (line([5.0, 5.0, 5.0], [0.0, 0.0, 0.0]), True),
        "single_free_pose": (line([4.0, 4.0, 4.0], [4.0, 4.0, 4.0], n=1), False),
    }

    for name, (states, expected) in trajectories.items():
        got = native.trajectory_in_collision(states)
        want = python.trajectory_in_collision(states)
        assert got == want, f"{name}: rust={got}, python={want}"
        assert got is expected, f"{name}: expected {expected}, got {got}"


def test_trajectory_in_collision_agrees_for_an_oriented_robot():
    robot = oriented_robot([Box([0.0, 0.0, 0.0], [3.0, 0.4, 0.4])])
    env = Environment(bounds=BOUNDS, obstacles=[Sphere([0.0, 1.2, 0.0], 0.3)])
    native, python = both_backends(robot, env)

    rng = np.random.default_rng(11)
    for _ in range(30):
        states = np.hstack(
            [
                rng.uniform(-2.5, 2.5, size=(12, 3)),
                rng.uniform(-np.pi, np.pi, size=(12, 1)),
            ]
        )
        assert native.trajectory_in_collision(states) == python.trajectory_in_collision(
            states
        )


# --------------------------------------------------------------------------- #
# Native scene API
# --------------------------------------------------------------------------- #
def test_unknown_side_is_rejected():
    import krrtstar_core

    scene = krrtstar_core.CollisionScene()
    with pytest.raises(ValueError):
        scene.add_sphere("both", np.zeros(3), 1.0)


def test_empty_scene_never_collides():
    robot = sphere_robot(0.5)
    env = Environment(bounds=BOUNDS, obstacles=[])
    native = CollisionChecker(robot, env, use_accel=True)
    assert native.backend == "rust"
    assert native.in_collision(np.zeros(6)) is False
    assert native.trajectory_in_collision(np.zeros((4, 6))) is False
