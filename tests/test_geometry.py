import numpy as np

from krrtstar.geometry import (
    Box,
    CollisionChecker,
    Environment,
    PoseMapping,
    Robot,
    Sphere,
    shapes_collide,
)


def test_sphere_sphere():
    a = Sphere(np.array([0.0, 0.0, 0.0]), 1.0)
    b = Sphere(np.array([1.5, 0.0, 0.0]), 1.0)
    c = Sphere(np.array([3.0, 0.0, 0.0]), 1.0)
    assert shapes_collide(a, b)
    assert not shapes_collide(a, c)


def test_sphere_box():
    box = Box(np.array([0.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]))
    inside = Sphere(np.array([1.2, 0.0, 0.0]), 0.5)
    outside = Sphere(np.array([3.0, 0.0, 0.0]), 0.5)
    assert shapes_collide(inside, box)
    assert not shapes_collide(outside, box)


def test_box_box():
    a = Box(np.array([0.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]))
    b = Box(np.array([1.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]))
    c = Box(np.array([5.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]))
    assert shapes_collide(a, b)
    assert not shapes_collide(a, c)


def test_collision_checker_state_mapping():
    robot = Robot(
        shapes=[Sphere(np.array([0.0, 0.0, 0.0]), 0.25)],
        pose=PoseMapping(x=0, y=1),
    )
    env = Environment(
        bounds=np.array([[-5, -5, -1], [5, 5, 1]]),
        obstacles=[Box(np.array([0.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]))],
    )
    checker = CollisionChecker(robot, env)
    assert checker.in_collision(np.array([0.0, 0.0, 0.0, 0.0]))
    assert not checker.in_collision(np.array([4.0, 4.0, 0.0, 0.0]))


def test_trajectory_collision():
    robot = Robot(shapes=[Sphere(np.array([0.0, 0.0, 0.0]), 0.25)], pose=PoseMapping(x=0, y=1))
    env = Environment(
        bounds=np.array([[-5, -5, -1], [5, 5, 1]]),
        obstacles=[Box(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 2.0]))],
    )
    checker = CollisionChecker(robot, env)
    clear = np.array([[-4, -4, 0, 0], [-4, 4, 0, 0]], dtype=float)
    through = np.array([[-4, 0, 0, 0], [0, 0, 0, 0], [4, 0, 0, 0]], dtype=float)
    assert not checker.trajectory_in_collision(clear)
    assert checker.trajectory_in_collision(through)
