"""Config-driven construction of the richer geometry types."""

import os

import numpy as np
import pytest

from krrtstar.config import (
    build_collision_checker,
    build_dynamics,
    load_config,
    parse_config,
)
from krrtstar.geometry import (
    Box,
    Capsule,
    Cylinder,
    Mesh,
    Sphere,
    load_mesh,
    rotation_from_euler,
)

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")
RICH = os.path.join(EXAMPLES, "rich_geometry_3d.toml")
BLOCK_OBJ = os.path.join(EXAMPLES, "assets", "block.obj")


def _minimal(extra_env=None, robot=None):
    data = {
        "dynamics": {"x_dim": 2, "u_dim": 2},
        "environment": {"bounds": [[-5, -5, -5], [5, 5, 5]]},
    }
    if extra_env:
        data["environment"].update(extra_env)
    if robot:
        data["robot"] = robot
    return data


# --------------------------------------------------------------------------- #
# Shape parsing
# --------------------------------------------------------------------------- #
def test_parses_every_shape_type():
    cfg = parse_config(
        _minimal(
            {
                "obstacles": [
                    {"type": "sphere", "center": [0, 0, 0], "radius": 1.0},
                    {"type": "box", "center": [1, 0, 0], "extents": [1, 2, 3]},
                    {"type": "capsule", "center": [2, 0, 0], "radius": 0.5, "height": 2.0},
                    {"type": "cylinder", "center": [3, 0, 0], "radius": 0.5, "height": 2.0},
                ]
            }
        )
    )
    kinds = [type(o) for o in cfg.environment.obstacles]
    assert kinds == [Sphere, Box, Capsule, Cylinder]
    capsule = cfg.environment.obstacles[2]
    assert capsule.radius == pytest.approx(0.5)
    assert capsule.height == pytest.approx(2.0)


def test_rejects_unknown_shape_type():
    with pytest.raises(ValueError, match="Unsupported shape type"):
        parse_config(_minimal({"obstacles": [{"type": "torus", "radius": 1}]}))


@pytest.mark.parametrize(
    "spec",
    [
        {"euler": [0.0, 0.0, np.pi / 2]},
        {"rotation": [0.0, 0.0, np.pi / 2]},
    ],
)
def test_orientation_from_euler_variants(spec):
    entry = {"type": "box", "center": [0, 0, 0], "extents": [4, 1, 1], **spec}
    cfg = parse_config(_minimal({"obstacles": [entry]}))
    box = cfg.environment.obstacles[0]
    assert np.allclose(box.rotation, rotation_from_euler(0.0, 0.0, np.pi / 2), atol=1e-12)
    # a 90 degree yaw swings the long axis from X onto Y
    lo, hi = box.aabb()
    assert (hi - lo)[0] == pytest.approx(1.0, abs=1e-9)
    assert (hi - lo)[1] == pytest.approx(4.0, abs=1e-9)


def test_orientation_from_quaternion_and_matrix():
    quat_cfg = parse_config(
        _minimal({"obstacles": [
            {"type": "box", "center": [0, 0, 0], "extents": [2, 1, 1],
             "quaternion": [1.0, 0.0, 0.0, 0.0]}
        ]})
    )
    assert np.allclose(quat_cfg.environment.obstacles[0].rotation, np.eye(3))

    matrix_cfg = parse_config(
        _minimal({"obstacles": [
            {"type": "box", "center": [0, 0, 0], "extents": [2, 1, 1],
             "rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}
        ]})
    )
    rot = matrix_cfg.environment.obstacles[0].rotation
    assert np.allclose(rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


def test_axis_aligned_box_has_identity_rotation():
    cfg = parse_config(
        _minimal({"obstacles": [{"type": "box", "center": [0, 0, 0], "extents": [1, 1, 1]}]})
    )
    assert cfg.environment.obstacles[0].is_axis_aligned()


# --------------------------------------------------------------------------- #
# Mesh loading
# --------------------------------------------------------------------------- #
def test_load_obj_reads_cube():
    vertices, faces = load_mesh(BLOCK_OBJ)
    assert vertices.shape == (8, 3)
    assert faces.shape == (12, 3)  # 6 quads fan-triangulated
    assert faces.min() >= 0 and faces.max() == 7


def test_mesh_path_resolves_relative_to_config(tmp_path):
    # A config in another directory must still find its asset by relative path.
    cfg_path = tmp_path / "exp.toml"
    cfg_path.write_text(
        f"""
[dynamics]
x_dim = 2
u_dim = 2

[environment]
bounds = [[-5,-5,-5],[5,5,5]]
[[environment.obstacles]]
type = "mesh"
path = "{os.path.relpath(BLOCK_OBJ, tmp_path)}"
center = [0,0,0]
scale = 2.0
"""
    )
    cfg = load_config(str(cfg_path))
    mesh = cfg.environment.obstacles[0]
    assert isinstance(mesh, Mesh)
    assert mesh.scale == pytest.approx(2.0)
    lo, hi = mesh.aabb()
    # the 2x2x2 cube scaled by 2 spans [-2, 2] on every axis
    assert np.allclose(lo, [-2, -2, -2], atol=1e-9)
    assert np.allclose(hi, [2, 2, 2], atol=1e-9)


# --------------------------------------------------------------------------- #
# Pose mapping with orientation
# --------------------------------------------------------------------------- #
def test_pose_mapping_reads_orientation_indices():
    cfg = parse_config(
        _minimal(robot={"pose_from_state": {"x": 0, "y": 1, "z": 2, "yaw": 3}})
    )
    pose = cfg.robot.pose
    assert pose.has_orientation
    state = np.array([1.0, 2.0, 3.0, np.pi / 2])
    assert np.allclose(pose.position(state), [1, 2, 3])
    assert np.allclose(pose.rotation(state) @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-12)


def test_pose_mapping_without_orientation_is_identity():
    cfg = parse_config(_minimal(robot={"pose_from_state": {"x": 0, "y": 1}}))
    pose = cfg.robot.pose
    assert not pose.has_orientation
    assert np.allclose(pose.rotation(np.array([1.0, 2.0])), np.eye(3))


def test_oriented_robot_shape_follows_state_yaw():
    cfg = parse_config(
        _minimal(
            {"obstacles": [{"type": "box", "center": [0, 2.0, 0], "extents": [0.4, 0.4, 0.4]}]},
            robot={
                "pose_from_state": {"x": 0, "y": 1, "yaw": 2},
                "geometry": [{"type": "box", "center": [0, 0, 0], "extents": [4.0, 0.3, 0.3]}],
            },
        )
    )
    checker = build_collision_checker(cfg)
    # A long bar at the origin: pointing along X it misses a block at y=2;
    # rotated 90 degrees it reaches up and hits it.
    assert not checker.in_collision(np.array([0.0, 0.0, 0.0]))
    assert checker.in_collision(np.array([0.0, 0.0, np.pi / 2]))


# --------------------------------------------------------------------------- #
# The bundled rich-geometry example
# --------------------------------------------------------------------------- #
def test_rich_geometry_example_loads_and_collides():
    cfg = load_config(RICH)
    assert [type(o).__name__ for o in cfg.environment.obstacles] == [
        "Box", "Cylinder", "Capsule", "Sphere", "Mesh",
    ]
    assert isinstance(cfg.robot.shapes[0], Capsule)
    build_dynamics(cfg.dynamics)  # must be constructible

    checker = build_collision_checker(cfg)
    free = np.array([-7.0, -7.0, 0.0, 0, 0, 0])
    inside_wall = np.array([0.0, 0.0, 0.0, 0, 0, 0])
    inside_pillar = np.array([-4.0, 3.5, 0.0, 0, 0, 0])
    assert not checker.in_collision(free)
    assert checker.in_collision(inside_wall)
    assert checker.in_collision(inside_pillar)


def test_rich_geometry_example_plans():
    from krrtstar.run import build_planner

    cfg = load_config(RICH)
    cfg.planner.target_nodes = 150
    planner = build_planner(cfg)
    result = planner.grow(cfg.planner.target_nodes)
    assert len(result.tree.nodes) > 1
    if result.found:
        # Any returned solution must be collision free.
        checker = build_collision_checker(cfg)
        states = np.vstack([t.states for t in result.solution])
        assert not checker.trajectory_in_collision(states)
