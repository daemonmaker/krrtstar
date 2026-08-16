"""Tests for the pure-Python GJK narrow phase in :mod:`krrtstar.collision`.

Expected values come from analytic geometry or from independent reference
implementations written below (point-to-OBB distance, segment-segment distance,
an OBB separating-axis test, and an exact feature-enumeration OBB-OBB distance),
never from the code under test.
"""

import itertools

import numpy as np
import pytest

from krrtstar.collision import (
    _closest_point_on_edges,
    _Triangle,
    closest_point_on_segment,
    closest_point_on_tetrahedron,
    closest_point_on_triangle,
    gjk_distance,
    shapes_distance,
    shapes_intersect,
)
from krrtstar.geometry import (
    Box,
    Capsule,
    Cylinder,
    Mesh,
    Sphere,
    rotation_from_euler,
    shapes_collide,
)

TOL = 1e-9
SQRT2 = np.sqrt(2.0)
SQRT3 = np.sqrt(3.0)


# --------------------------------------------------------------------------- #
# Reference implementations (independent of the code under test)
# --------------------------------------------------------------------------- #
def ref_point_obb_distance(point, box):
    """Exact distance from a point to a (possibly oriented) box."""
    local = box.rotation.T @ (np.asarray(point, float) - box.center)
    outside = np.maximum(0.0, np.abs(local) - box.half_extents)
    return float(np.linalg.norm(outside))


def ref_segment_segment_distance(p1, q1, p2, q2):
    """Exact distance between two segments (clamped closed form)."""
    p1, q1, p2, q2 = (np.asarray(v, float) for v in (p1, q1, p2, q2))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a <= 1e-300 and e <= 1e-300:
        return float(np.linalg.norm(r))
    if a <= 1e-300:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    elif e <= 1e-300:
        s, t = np.clip(-float(d1 @ r) / a, 0.0, 1.0), 0.0
    else:
        c = float(d1 @ r)
        b = float(d1 @ d2)
        denom = a * e - b * b
        s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > 1e-300 else 0.0
        t = (b * s + f) / e
        if t < 0.0:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        elif t > 1.0:
            t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


def ref_point_triangle_distance(point, tri):
    """Exact point-triangle distance via a plane projection with edge fallback."""
    point = np.asarray(point, float)
    a, b, c = (np.asarray(v, float) for v in tri)
    ab, ac, w = b - a, c - a, point - a
    d1, d2, d3 = float(ab @ ab), float(ab @ ac), float(ac @ ac)
    det = d1 * d3 - d2 * d2
    if det > 1e-24:
        r1, r2 = float(w @ ab), float(w @ ac)
        u = (d3 * r1 - d2 * r2) / det
        v = (d1 * r2 - d2 * r1) / det
        if u >= 0.0 and v >= 0.0 and u + v <= 1.0:
            return float(np.linalg.norm(w - u * ab - v * ac))
    return min(
        ref_segment_segment_distance(a, b, point, point),
        ref_segment_segment_distance(b, c, point, point),
        ref_segment_segment_distance(a, c, point, point),
    )


def ref_triangle_triangle_distance(t1, t2):
    """Exact distance between two disjoint triangles by feature enumeration."""
    best = min(
        min(ref_point_triangle_distance(v, t2) for v in t1),
        min(ref_point_triangle_distance(v, t1) for v in t2),
    )
    for i in range(3):
        for j in range(3):
            best = min(
                best,
                ref_segment_segment_distance(
                    t1[i], t1[(i + 1) % 3], t2[j], t2[(j + 1) % 3]
                ),
            )
    return best


def ref_mesh_sphere_distance(mesh, sphere):
    tris = mesh.triangles()
    closest = min(ref_point_triangle_distance(sphere.center, t) for t in tris)
    return max(0.0, closest - sphere.radius)


def ref_mesh_mesh_distance(a, b):
    ta, tb = a.triangles(), b.triangles()
    return min(ref_triangle_triangle_distance(t1, t2) for t1 in ta for t2 in tb)


def ref_edge_pierces_unit_square(mesh, z=0.0, lo=0.0, hi=1.0):
    """True when some mesh edge crosses the plane inside ``[lo, hi]^2``."""
    for tri in mesh.triangles():
        for i in range(3):
            p, q = tri[i], tri[(i + 1) % 3]
            if (p[2] - z) * (q[2] - z) < 0.0:
                point = p + ((z - p[2]) / (q[2] - p[2])) * (q - p)
                if lo <= point[0] <= hi and lo <= point[1] <= hi:
                    return True
    return False


def ref_segment_segment_distance_sampled(p1, q1, p2, q2, n=1200):
    """Brute-force cross-check for :func:`ref_segment_segment_distance`."""
    ts = np.linspace(0.0, 1.0, n)
    a = np.asarray(p1, float) + ts[:, None] * (np.asarray(q1, float) - np.asarray(p1, float))
    b = np.asarray(p2, float) + ts[:, None] * (np.asarray(q2, float) - np.asarray(p2, float))
    return float(np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)).min())


def box_edges(box):
    """The 12 edges of a box as ``(12, 2, 3)`` world-frame endpoints."""
    corners = box.core_points()  # ordered by (sx, sy, sz) bit pattern
    edges = []
    for i, j in itertools.combinations(range(8), 2):
        if bin(i ^ j).count("1") == 1:
            edges.append([corners[i], corners[j]])
    return np.asarray(edges, float)


def ref_obb_overlap(a, b):
    """Separating-axis test: exact overlap predicate for two oriented boxes."""
    axes = [a.rotation[:, i] for i in range(3)] + [b.rotation[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            axis = np.cross(a.rotation[:, i], b.rotation[:, j])
            if float(axis @ axis) > 1e-18:
                axes.append(axis)
    delta = b.center - a.center
    for axis in axes:
        axis = axis / np.linalg.norm(axis)
        ra = float(np.abs(a.rotation.T @ axis) @ a.half_extents)
        rb = float(np.abs(b.rotation.T @ axis) @ b.half_extents)
        if abs(float(delta @ axis)) > ra + rb:
            return False
    return True


def ref_obb_distance(a, b):
    """Exact distance between two *disjoint* boxes by feature enumeration.

    For disjoint convex polytopes the closest pair is realised by a
    vertex/face or an edge/edge pair, so enumerating both suffices.
    """
    best = min(
        min(ref_point_obb_distance(v, b) for v in a.core_points()),
        min(ref_point_obb_distance(v, a) for v in b.core_points()),
    )
    ea, eb = box_edges(a), box_edges(b)
    for sa in ea:
        for sb in eb:
            best = min(best, ref_segment_segment_distance(sa[0], sa[1], sb[0], sb[1]))
    return best


def test_reference_segment_distance_matches_brute_force():
    rng = np.random.default_rng(7)
    for _ in range(20):
        pts = rng.uniform(-2.0, 2.0, size=(4, 3))
        exact = ref_segment_segment_distance(*pts)
        sampled = ref_segment_segment_distance_sampled(*pts)
        assert exact <= sampled + 1e-12
        assert sampled - exact < 1e-4


# --------------------------------------------------------------------------- #
# Simplex closest-point helpers
# --------------------------------------------------------------------------- #
def brute_closest_on_simplex(points, n=60):
    """Brute-force closest point to the origin on a convex combination grid."""
    points = np.asarray(points, float)
    k = points.shape[0]
    grids = np.meshgrid(*[np.linspace(0.0, 1.0, n)] * (k - 1), indexing="ij")
    weights = np.stack([g.ravel() for g in grids], axis=1)
    keep = weights.sum(axis=1) <= 1.0 + 1e-12
    weights = weights[keep]
    full = np.hstack([1.0 - weights.sum(axis=1, keepdims=True), weights])
    pts = full @ points
    return float(np.linalg.norm(pts, axis=1).min())


def test_closest_point_on_segment_regions():
    a = np.array([1.0, 1.0, 0.0])
    b = np.array([3.0, 1.0, 0.0])
    point, kept = closest_point_on_segment(a, b)
    assert kept == (0,)
    assert np.allclose(point, a)

    point, kept = closest_point_on_segment(np.array([-3.0, 1.0, 0.0]), np.array([-1.0, 1.0, 0.0]))
    assert kept == (1,)
    assert np.allclose(point, [-1.0, 1.0, 0.0])

    point, kept = closest_point_on_segment(np.array([-1.0, 2.0, 0.0]), np.array([1.0, 2.0, 0.0]))
    assert kept == (0, 1)
    assert np.allclose(point, [0.0, 2.0, 0.0])

    # Degenerate segment (both endpoints coincide).
    point, kept = closest_point_on_segment(a, a.copy())
    assert kept == (0,)
    assert np.allclose(point, a)


def test_closest_point_on_triangle_regions():
    # Triangle in the z = 1 plane; the origin projects inside it.
    a = np.array([-1.0, -1.0, 1.0])
    b = np.array([2.0, -1.0, 1.0])
    c = np.array([-1.0, 2.0, 1.0])
    point, kept = closest_point_on_triangle(a, b, c)
    assert kept == (0, 1, 2)
    assert np.allclose(point, [0.0, 0.0, 1.0])

    # Vertex region.
    shifted = [v + np.array([3.0, 3.0, 0.0]) for v in (a, b, c)]
    point, kept = closest_point_on_triangle(*shifted)
    assert kept == (0,)
    assert np.allclose(point, shifted[0])

    # Edge region: origin closest to the interior of edge BC.
    a2 = np.array([0.0, 0.0, 3.0])
    b2 = np.array([-1.0, 0.0, 1.0])
    c2 = np.array([1.0, 0.0, 1.0])
    point, kept = closest_point_on_triangle(a2, b2, c2)
    assert kept == (1, 2)
    assert np.allclose(point, [0.0, 0.0, 1.0])

    # Degenerate (collinear) triangle behaves like the containing segment.
    point, kept = closest_point_on_triangle(
        np.array([-1.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([1.0, 1.0, 0.0])
    )
    assert np.allclose(np.linalg.norm(point), 1.0)


def test_closest_point_on_triangle_matches_brute_force():
    rng = np.random.default_rng(11)
    for _ in range(30):
        pts = rng.uniform(-2.0, 2.0, size=(3, 3))
        point, _ = closest_point_on_triangle(*pts)
        exact = float(np.linalg.norm(point))
        assert exact <= brute_closest_on_simplex(pts) + 1e-9


def test_closest_point_on_tetrahedron_inside_and_outside():
    verts = np.array(
        [[1.0, 1.0, 1.0], [-1.0, 1.0, 1.0], [0.0, -1.0, 1.0], [0.0, 0.0, -1.0]]
    )
    point, kept = closest_point_on_tetrahedron(*verts)
    assert kept == (0, 1, 2, 3)
    assert np.allclose(point, 0.0)

    outside = verts + np.array([0.0, 0.0, 4.0])
    point, kept = closest_point_on_tetrahedron(*outside)
    assert len(kept) < 4
    assert np.isclose(float(np.linalg.norm(point)), 3.0)

    # Degenerate (planar) tetrahedron: closest point lies on the plane.
    planar = np.array(
        [[0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [0.0, 1.0, 2.0], [1.0, 1.0, 2.0]]
    )
    point, kept = closest_point_on_tetrahedron(*planar)
    assert np.isclose(float(np.linalg.norm(point)), 2.0)


def test_closest_point_on_edges_fallback():
    """Safety net used when a triangle is too degenerate for the face branch."""
    verts = (
        np.array([-1.0, 2.0, 0.0]),
        np.array([1.0, 2.0, 0.0]),
        np.array([0.0, 5.0, 0.0]),
    )
    point, kept = _closest_point_on_edges(verts, ((0, 1), (0, 2), (1, 2)))
    assert kept == (0, 1)
    assert np.allclose(point, [0.0, 2.0, 0.0])

    point, kept = _closest_point_on_edges(verts, ((0, 2), (1, 2)))
    assert kept in {(0,), (1,)}
    assert np.isclose(float(np.linalg.norm(point)), np.sqrt(5.0))


def test_closest_point_on_tetrahedron_matches_brute_force():
    rng = np.random.default_rng(13)
    for _ in range(20):
        pts = rng.uniform(-2.0, 2.0, size=(4, 3)) + np.array([0.0, 0.0, 3.0])
        point, _ = closest_point_on_tetrahedron(*pts)
        exact = float(np.linalg.norm(point))
        assert exact <= brute_closest_on_simplex(pts, n=30) + 1e-9


# --------------------------------------------------------------------------- #
# Sphere / sphere
# --------------------------------------------------------------------------- #
def test_sphere_sphere_separated():
    a = Sphere([0.0, 0.0, 0.0], 1.0)
    b = Sphere([5.0, 0.0, 0.0], 1.5)
    assert shapes_distance(a, b) == pytest.approx(2.5, abs=TOL)
    assert not shapes_intersect(a, b)
    # The GJK call sees only the cores (two points), i.e. the centre distance.
    assert gjk_distance(a, b) == pytest.approx(5.0, abs=TOL)


def test_sphere_sphere_touching_counts_as_collision():
    a = Sphere([0.0, 0.0, 0.0], 1.0)
    b = Sphere([2.0, 0.0, 0.0], 1.0)
    assert shapes_distance(a, b) == pytest.approx(0.0, abs=TOL)
    assert shapes_intersect(a, b)


def test_sphere_sphere_overlapping():
    a = Sphere([0.0, 0.0, 0.0], 1.0)
    b = Sphere([1.0, 1.0, 1.0], 1.0)
    assert shapes_distance(a, b) == 0.0
    assert shapes_intersect(a, b)
    # Fully contained.
    assert shapes_intersect(Sphere([0.1, 0.0, 0.0], 0.2), a)


def test_sphere_sphere_diagonal_distance():
    a = Sphere([1.0, 2.0, 3.0], 0.25)
    b = Sphere([4.0, 6.0, 3.0], 0.75)
    assert shapes_distance(a, b) == pytest.approx(5.0 - 1.0, abs=TOL)


# --------------------------------------------------------------------------- #
# Sphere / box
# --------------------------------------------------------------------------- #
def test_sphere_axis_aligned_box_features():
    box = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])  # half extents of 1

    face = Sphere([3.0, 0.0, 0.0], 0.5)
    assert shapes_distance(face, box) == pytest.approx(1.5, abs=TOL)

    edge = Sphere([3.0, 3.0, 0.0], 0.5)
    assert shapes_distance(edge, box) == pytest.approx(2.0 * SQRT2 - 0.5, abs=TOL)

    corner = Sphere([3.0, 3.0, 3.0], 0.5)
    assert shapes_distance(corner, box) == pytest.approx(2.0 * SQRT3 - 0.5, abs=TOL)

    for shape in (face, edge, corner):
        assert not shapes_intersect(shape, box)
        assert ref_point_obb_distance(shape.center, box) - shape.radius == pytest.approx(
            shapes_distance(shape, box), abs=TOL
        )


def test_sphere_box_touching_and_containment():
    box = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    touching = Sphere([1.5, 0.0, 0.0], 0.5)
    assert shapes_distance(touching, box) == pytest.approx(0.0, abs=TOL)
    assert shapes_intersect(touching, box)

    contained = Sphere([0.2, -0.1, 0.3], 0.25)
    assert shapes_distance(contained, box) == 0.0
    assert shapes_intersect(contained, box)

    # A box fully inside a sphere.
    big = Sphere([0.0, 0.0, 0.0], 5.0)
    assert shapes_intersect(big, box)
    assert shapes_distance(big, box) == 0.0


def test_sphere_oriented_box_honours_rotation():
    """A box rotated 45 deg about Z reaches sqrt(2) along +x, unrotated only 1."""
    extents = [2.0, 2.0, 2.0]
    plain = Box([0.0, 0.0, 0.0], extents)
    rotated = Box([0.0, 0.0, 0.0], extents, rotation_from_euler(yaw=np.pi / 4))

    probe = Sphere([1.5, 0.0, 0.0], 0.2)
    assert shapes_intersect(probe, rotated)
    assert not shapes_intersect(probe, plain)
    assert shapes_distance(probe, plain) == pytest.approx(0.5 - 0.2, abs=TOL)

    # Separated along the rotated diagonal: distance is measured to the corner.
    far = Sphere([2.0, 0.0, 0.0], 0.2)
    assert shapes_distance(far, rotated) == pytest.approx(2.0 - SQRT2 - 0.2, abs=TOL)
    # ... and along the rotated face normal the box only reaches 1.0.
    side = Sphere([1.5 / SQRT2, 1.5 / SQRT2, 0.0], 0.2)
    assert shapes_distance(side, rotated) == pytest.approx(0.5 - 0.2, abs=TOL)
    assert shapes_intersect(side, plain)  # the unrotated box does reach there


def test_sphere_oriented_box_random_positions():
    rng = np.random.default_rng(17)
    box = Box([0.3, -0.2, 0.1], [1.0, 2.0, 0.5], rotation_from_euler(0.3, -0.7, 1.1))
    for _ in range(200):
        centre = rng.uniform(-3.0, 3.0, size=3)
        radius = float(rng.uniform(0.0, 0.5))
        sphere = Sphere(centre, radius)
        expected = max(0.0, ref_point_obb_distance(centre, box) - radius)
        assert shapes_distance(sphere, box) == pytest.approx(expected, abs=1e-9)
        if abs(expected) > 1e-6:
            assert shapes_intersect(sphere, box) == (expected == 0.0)


# --------------------------------------------------------------------------- #
# Box / box
# --------------------------------------------------------------------------- #
def test_axis_aligned_box_box():
    a = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    overlapping = Box([1.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    separated = Box([5.0, 0.0, 0.0], [1.0, 1.0, 1.0])
    touching = Box([2.0, 0.0, 0.0], [2.0, 2.0, 2.0])

    assert shapes_intersect(a, overlapping)
    assert shapes_distance(a, overlapping) == 0.0

    assert not shapes_intersect(a, separated)
    assert shapes_distance(a, separated) == pytest.approx(3.5, abs=TOL)

    assert shapes_intersect(a, touching)
    assert shapes_distance(a, touching) == pytest.approx(0.0, abs=TOL)

    # Diagonal separation: closest features are two corners.
    diagonal = Box([3.0, 3.0, 0.0], [2.0, 2.0, 2.0])
    assert shapes_distance(a, diagonal) == pytest.approx(SQRT2, abs=TOL)


def test_oriented_boxes_separated_despite_overlapping_aabbs():
    """Two 45 deg diamonds whose AABBs overlap but which are disjoint."""
    yaw = rotation_from_euler(yaw=np.pi / 4)
    a = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], yaw)
    b = Box([2.2, 2.2, 0.0], [2.0, 2.0, 2.0], yaw)

    alo, ahi = a.aabb()
    blo, bhi = b.aabb()
    assert np.all(ahi >= blo) and np.all(bhi >= alo), "AABBs must overlap for this test"

    assert not ref_obb_overlap(a, b)
    assert not shapes_intersect(a, b)
    # Corner-to-corner distance: sqrt(2) * (2.2 - sqrt(2)) = 2.2*sqrt(2) - 2.
    assert shapes_distance(a, b) == pytest.approx(2.2 * SQRT2 - 2.0, abs=TOL)
    assert shapes_distance(a, b) == pytest.approx(ref_obb_distance(a, b), abs=TOL)

    # Sliding b in along the diagonal makes them overlap.
    close = Box([1.3, 1.3, 0.0], [2.0, 2.0, 2.0], yaw)
    assert ref_obb_overlap(a, close)
    assert shapes_intersect(a, close)


def test_oriented_boxes_random_against_reference():
    rng = np.random.default_rng(23)
    checked_separated = 0
    checked_overlapping = 0
    for _ in range(150):
        a = Box(
            rng.uniform(-1.5, 1.5, size=3),
            rng.uniform(0.3, 2.0, size=3),
            rotation_from_euler(*rng.uniform(-np.pi, np.pi, size=3)),
        )
        b = Box(
            rng.uniform(-1.5, 1.5, size=3),
            rng.uniform(0.3, 2.0, size=3),
            rotation_from_euler(*rng.uniform(-np.pi, np.pi, size=3)),
        )
        overlap = ref_obb_overlap(a, b)
        assert shapes_intersect(a, b) == overlap
        if overlap:
            assert shapes_distance(a, b) == 0.0
            checked_overlapping += 1
        else:
            expected = ref_obb_distance(a, b)
            assert shapes_distance(a, b) == pytest.approx(expected, abs=1e-9)
            checked_separated += 1
    assert checked_separated > 10 and checked_overlapping > 10


# --------------------------------------------------------------------------- #
# Capsules
# --------------------------------------------------------------------------- #
def test_capsule_sphere():
    # Segment from (0,0,-1) to (0,0,1), radius 0.5.
    capsule = Capsule([0.0, 0.0, 0.0], 0.5, 2.0)
    a, b = capsule.segment()
    assert np.allclose(a, [0.0, 0.0, -1.0]) and np.allclose(b, [0.0, 0.0, 1.0])

    radial = Sphere([2.0, 0.0, 0.0], 0.25)
    expected = ref_segment_segment_distance(a, b, radial.center, radial.center) - 0.75
    assert shapes_distance(capsule, radial) == pytest.approx(expected, abs=TOL)
    assert shapes_distance(capsule, radial) == pytest.approx(2.0 - 0.75, abs=TOL)

    axial = Sphere([0.0, 0.0, 2.0], 0.25)
    assert shapes_distance(capsule, axial) == pytest.approx(1.0 - 0.75, abs=TOL)

    hit = Sphere([0.6, 0.0, 0.5], 0.25)
    assert shapes_intersect(capsule, hit)
    assert shapes_distance(capsule, hit) == 0.0

    # Sphere beyond the cap, diagonally.
    diagonal = Sphere([3.0, 0.0, 5.0], 0.5)
    assert shapes_distance(capsule, diagonal) == pytest.approx(5.0 - 1.0, abs=TOL)


def test_capsule_capsule_parallel_crossing_separated():
    base = Capsule([0.0, 0.0, 0.0], 0.5, 2.0)
    ba, bb = base.segment()

    # Parallel, offset along x: segment distance is 3, radii sum to 1.
    parallel = Capsule([3.0, 0.0, 0.0], 0.5, 2.0)
    pa, pb = parallel.segment()
    assert ref_segment_segment_distance(ba, bb, pa, pb) == pytest.approx(3.0, abs=TOL)
    assert shapes_distance(base, parallel) == pytest.approx(2.0, abs=TOL)
    assert not shapes_intersect(base, parallel)

    # Crossing (rotated onto the X axis) and separated along z.
    crossing = Capsule([0.0, 0.0, 3.0], 0.25, 2.0, rotation_from_euler(pitch=np.pi / 2))
    ca, cb = crossing.segment()
    seg = ref_segment_segment_distance(ba, bb, ca, cb)
    assert seg == pytest.approx(2.0, abs=TOL)
    assert shapes_distance(base, crossing) == pytest.approx(seg - 0.75, abs=TOL)
    assert not shapes_intersect(base, crossing)

    # Crossing and actually intersecting.
    touching = Capsule([0.0, 0.0, 0.5], 0.25, 2.0, rotation_from_euler(pitch=np.pi / 2))
    assert ref_segment_segment_distance(ba, bb, *touching.segment()) == pytest.approx(
        0.0, abs=TOL
    )
    assert shapes_intersect(base, touching)
    assert shapes_distance(base, touching) == 0.0

    # Skew capsules: distance exactly equals radii-reduced segment distance.
    skew = Capsule([1.0, 1.5, 0.0], 0.3, 3.0, rotation_from_euler(roll=np.pi / 2))
    expected = max(0.0, ref_segment_segment_distance(ba, bb, *skew.segment()) - 0.8)
    assert shapes_distance(base, skew) == pytest.approx(expected, abs=TOL)


def test_capsule_capsule_random_against_segment_reference():
    rng = np.random.default_rng(29)
    for _ in range(200):
        a = Capsule(
            rng.uniform(-2.0, 2.0, size=3),
            float(rng.uniform(0.05, 0.6)),
            float(rng.uniform(0.0, 2.5)),
            rotation_from_euler(*rng.uniform(-np.pi, np.pi, size=3)),
        )
        b = Capsule(
            rng.uniform(-2.0, 2.0, size=3),
            float(rng.uniform(0.05, 0.6)),
            float(rng.uniform(0.0, 2.5)),
            rotation_from_euler(*rng.uniform(-np.pi, np.pi, size=3)),
        )
        seg = ref_segment_segment_distance(*a.segment(), *b.segment())
        expected = max(0.0, seg - a.radius - b.radius)
        assert shapes_distance(a, b) == pytest.approx(expected, abs=1e-9)
        if abs(expected) > 1e-6:
            assert shapes_intersect(a, b) == (expected == 0.0)


def test_capsule_box():
    box = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    # Vertical capsule beside the box: purely radial separation.
    beside = Capsule([2.5, 0.0, 0.0], 0.25, 1.0)
    assert shapes_distance(beside, box) == pytest.approx(1.5 - 0.25, abs=TOL)
    assert not shapes_intersect(beside, box)
    # Long horizontal capsule that passes through the box.
    through = Capsule([0.0, 0.0, 0.0], 0.1, 8.0, rotation_from_euler(pitch=np.pi / 2))
    assert shapes_intersect(through, box)


# --------------------------------------------------------------------------- #
# Cylinder
# --------------------------------------------------------------------------- #
def test_cylinder_sphere_side_and_cap():
    cylinder = Cylinder([0.0, 0.0, 0.0], 1.0, 2.0)  # z from -1 to 1

    side = Sphere([3.0, 0.0, 0.0], 0.5)
    assert shapes_distance(cylinder, side) == pytest.approx(3.0 - 1.0 - 0.5, abs=TOL)

    cap = Sphere([0.0, 0.0, 2.5], 0.5)
    assert shapes_distance(cylinder, cap) == pytest.approx(2.5 - 1.0 - 0.5, abs=TOL)

    # Off-axis but still over the cap: distance stays purely axial.
    over_cap = Sphere([0.4, 0.3, 2.0], 0.25)
    assert shapes_distance(cylinder, over_cap) == pytest.approx(2.0 - 1.0 - 0.25, abs=TOL)

    # Beyond the rim: distance to the circular edge.
    rim = Sphere([2.0, 0.0, 2.0], 0.5)
    assert shapes_distance(cylinder, rim) == pytest.approx(SQRT2 - 0.5, abs=TOL)

    # Diagonal radial direction.
    diagonal = Sphere([2.0, 2.0, 0.0], 0.25)
    assert shapes_distance(cylinder, diagonal) == pytest.approx(
        2.0 * SQRT2 - 1.0 - 0.25, abs=TOL
    )

    assert shapes_intersect(cylinder, Sphere([1.2, 0.0, 0.0], 0.25))
    assert shapes_intersect(cylinder, Sphere([0.0, 0.0, 1.2], 0.25))
    assert not shapes_intersect(cylinder, Sphere([1.4, 0.0, 0.0], 0.25))


def test_rotated_cylinder_sphere():
    # Axis along world X after a 90 deg pitch.
    cylinder = Cylinder([0.0, 0.0, 0.0], 1.0, 4.0, rotation_from_euler(pitch=np.pi / 2))
    along_axis = Sphere([4.0, 0.0, 0.0], 0.5)
    assert shapes_distance(cylinder, along_axis) == pytest.approx(4.0 - 2.0 - 0.5, abs=TOL)
    radial = Sphere([0.0, 0.0, 3.0], 0.5)
    assert shapes_distance(cylinder, radial) == pytest.approx(3.0 - 1.0 - 0.5, abs=TOL)


# --------------------------------------------------------------------------- #
# Meshes
# --------------------------------------------------------------------------- #
def square_mesh(**kwargs):
    """Unit square in the z = 0 plane built from two triangles."""
    vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    return Mesh(vertices, faces, **kwargs)


def tetrahedron_mesh(**kwargs):
    """Closed tetrahedron with unit legs at the origin."""
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    faces = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])
    return Mesh(vertices, faces, **kwargs)


def l_shaped_mesh():
    """Non-convex L in the z = 0 plane: [0,2]x[0,1] plus [0,1]x[1,2]."""
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ]
    )
    faces = np.array([[0, 1, 2], [0, 2, 3], [3, 4, 5], [3, 5, 6]])
    return Mesh(vertices, faces)


def test_mesh_square_sphere():
    mesh = square_mesh()
    hit = Sphere([0.5, 0.5, 0.2], 0.3)
    assert shapes_intersect(hit, mesh)
    assert shapes_distance(hit, mesh) == 0.0

    miss = Sphere([0.5, 0.5, 1.0], 0.3)
    assert not shapes_intersect(miss, mesh)
    assert shapes_distance(miss, mesh) == pytest.approx(0.7, abs=TOL)

    # Off to the side: closest feature is the square's corner.
    corner = Sphere([2.0, 2.0, 0.0], 0.5)
    assert shapes_distance(corner, mesh) == pytest.approx(SQRT2 - 0.5, abs=TOL)

    # Touching the surface exactly counts as a collision.
    touching = Sphere([0.5, 0.5, 0.25], 0.25)
    assert shapes_intersect(touching, mesh)


def test_mesh_placement_is_honoured():
    mesh = square_mesh(center=[0.0, 0.0, 5.0], rotation=rotation_from_euler(yaw=0.7), scale=2.0)
    tris = mesh.triangles()
    assert np.allclose(tris[:, :, 2], 5.0)
    sphere = Sphere([0.0, 0.0, 6.0], 0.25)
    # The scaled/rotated square still spans the origin corner, so the closest
    # point is directly below the sphere on the z = 5 plane.
    assert shapes_distance(sphere, mesh) == pytest.approx(0.75, abs=TOL)
    assert shapes_intersect(Sphere([0.0, 0.0, 5.1], 0.25), mesh)


def test_mesh_tetrahedron():
    mesh = tetrahedron_mesh()
    # Just outside the slanted face x + y + z = 1.
    normal = np.ones(3) / SQRT3
    outside = Sphere(normal * (1.0 / SQRT3 + 0.4), 0.2)
    assert not shapes_intersect(outside, mesh)
    assert shapes_distance(outside, mesh) == pytest.approx(0.2, abs=1e-9)

    hitting = Sphere(normal * (1.0 / SQRT3 + 0.1), 0.2)
    assert shapes_intersect(hitting, mesh)

    far = Sphere([5.0, 5.0, 5.0], 1.0)
    assert not shapes_intersect(far, mesh)

    # A triangle soup is a surface, not a solid: a small sphere floating in the
    # interior touches no triangle.
    interior = Sphere([0.25, 0.25, 0.25], 0.05)
    assert not shapes_intersect(interior, mesh)
    assert shapes_distance(interior, mesh) == pytest.approx(0.25 / SQRT3 - 0.05, abs=1e-9)


def test_non_convex_mesh_concavity_is_not_filled():
    mesh = l_shaped_mesh()
    probe = np.array([1.4, 1.4, 0.0])
    # The notch point is 0.4 from the L but inside the convex hull.
    small = Sphere(probe, 0.3)
    assert not shapes_intersect(small, mesh)
    assert shapes_distance(small, mesh) == pytest.approx(0.4 - 0.3, abs=TOL)
    # gjk_distance uses Mesh.support, i.e. the convex hull, which does contain it.
    assert gjk_distance(Sphere(probe, 0.0), mesh) == 0.0

    big = Sphere(probe, 0.5)
    assert shapes_intersect(big, mesh)
    assert shapes_distance(big, mesh) == 0.0


def test_non_convex_two_disjoint_triangles():
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [4.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [4.0, 1.0, 0.0],
        ]
    )
    mesh = Mesh(vertices, np.array([[0, 1, 2], [3, 4, 5]]))
    between = Sphere([2.5, 0.0, 0.0], 0.5)
    assert not shapes_intersect(between, mesh)
    assert shapes_distance(between, mesh) == pytest.approx(1.5 - 0.5, abs=TOL)
    assert shapes_intersect(Sphere([2.5, 0.0, 0.0], 1.6), mesh)


def test_mesh_vs_mesh():
    a = square_mesh()
    stacked = square_mesh(center=[0.0, 0.0, 2.0])
    assert shapes_distance(a, stacked) == pytest.approx(2.0, abs=TOL)
    assert not shapes_intersect(a, stacked)

    beside = square_mesh(center=[3.0, 0.0, 0.0])
    assert shapes_distance(a, beside) == pytest.approx(2.0, abs=TOL)

    crossing = square_mesh(center=[0.5, 0.5, -0.5], rotation=rotation_from_euler(roll=np.pi / 2))
    assert shapes_intersect(a, crossing)
    assert shapes_distance(a, crossing) == 0.0

    assert shapes_intersect(a, square_mesh())


def test_mesh_sphere_random_against_reference():
    rng = np.random.default_rng(31)
    meshes = [
        square_mesh(center=[0.0, 0.0, 0.0]),
        tetrahedron_mesh(center=[0.5, 0.0, 0.0], rotation=rotation_from_euler(0.2, 0.5, 1.0)),
        l_shaped_mesh(),
    ]
    for mesh in meshes:
        for _ in range(60):
            sphere = Sphere(rng.uniform(-2.5, 2.5, size=3), float(rng.uniform(0.0, 0.6)))
            expected = ref_mesh_sphere_distance(mesh, sphere)
            assert shapes_distance(mesh, sphere) == pytest.approx(expected, abs=1e-9)
            if abs(expected) > 1e-6:
                assert shapes_intersect(mesh, sphere) == (expected == 0.0)


def grid_mesh(n, **kwargs):
    """``n`` x ``n`` grid of quads (2 n^2 triangles) in the z = 0 plane."""
    coords = np.arange(n + 1) * 0.5
    vertices = np.array([[x, y, 0.0] for x in coords for y in coords])
    faces = []
    for i in range(n):
        for j in range(n):
            v00 = i * (n + 1) + j
            v10 = v00 + (n + 1)
            faces += [[v00, v10, v10 + 1], [v00, v10 + 1, v00 + 1]]
    return Mesh(vertices, np.array(faces), **kwargs)


def test_mesh_mesh_many_triangles_against_reference():
    """Exercises the AABB pruning on both loop levels (32 x 32 triangles)."""
    a = grid_mesh(4)
    b = grid_mesh(4, center=[0.7, 0.3, 2.0], rotation=rotation_from_euler(0.4, 0.2, 0.9))
    expected = ref_mesh_mesh_distance(a, b)
    assert expected > 0.0
    assert shapes_distance(a, b) == pytest.approx(expected, abs=1e-9)
    assert not shapes_intersect(a, b)

    # Pushed together until the soups cross. ``a`` tiles the square
    # [0, 2] x [0, 2] at z = 0, so an edge of ``c`` piercing that square inside
    # those bounds proves the two surfaces intersect.
    c = grid_mesh(4, center=[0.7, 0.3, 0.2], rotation=rotation_from_euler(0.4, 0.2, 0.9))
    assert ref_edge_pierces_unit_square(c, z=0.0, hi=2.0)
    assert shapes_intersect(a, c)
    assert shapes_distance(a, c) == 0.0


def test_mesh_without_faces_is_infinitely_far():
    empty = Mesh(np.zeros((3, 3)), np.zeros((0, 3), dtype=int))
    sphere = Sphere([0.0, 0.0, 0.0], 1.0)
    assert shapes_distance(empty, sphere) == float("inf")
    assert not shapes_intersect(empty, sphere)
    assert shapes_distance(empty, Mesh(np.zeros((3, 3)), np.zeros((0, 3), dtype=int))) == float(
        "inf"
    )
    assert shapes_distance(square_mesh(), empty) == float("inf")


def test_mesh_vs_box_and_capsule():
    mesh = square_mesh()
    box = Box([0.5, 0.5, 1.5], [1.0, 1.0, 1.0])
    assert shapes_distance(mesh, box) == pytest.approx(1.0, abs=TOL)
    assert not shapes_intersect(mesh, box)
    assert shapes_intersect(mesh, Box([0.5, 0.5, 0.4], [1.0, 1.0, 1.0]))

    capsule = Capsule([0.5, 0.5, 1.0], 0.25, 1.0)
    assert shapes_distance(mesh, capsule) == pytest.approx(0.5 - 0.25, abs=TOL)
    assert shapes_intersect(mesh, Capsule([0.5, 0.5, 0.3], 0.25, 1.0))


# --------------------------------------------------------------------------- #
# Symmetry over a randomized sweep
# --------------------------------------------------------------------------- #
def random_shape(rng):
    kind = rng.integers(0, 5)
    centre = rng.uniform(-2.0, 2.0, size=3)
    rotation = rotation_from_euler(*rng.uniform(-np.pi, np.pi, size=3))
    if kind == 0:
        return Sphere(centre, float(rng.uniform(0.0, 0.8)))
    if kind == 1:
        return Box(centre, rng.uniform(0.2, 1.6, size=3), rotation)
    if kind == 2:
        return Capsule(centre, float(rng.uniform(0.05, 0.6)), float(rng.uniform(0.0, 2.0)), rotation)
    if kind == 3:
        return Cylinder(centre, float(rng.uniform(0.1, 0.9)), float(rng.uniform(0.2, 2.0)), rotation)
    mesh = rng.integers(0, 3)
    if mesh == 0:
        return square_mesh(center=centre, rotation=rotation, scale=float(rng.uniform(0.5, 2.0)))
    if mesh == 1:
        return tetrahedron_mesh(center=centre, rotation=rotation, scale=float(rng.uniform(0.5, 2.0)))
    return Mesh(
        l_shaped_mesh().vertices, l_shaped_mesh().faces, center=centre, rotation=rotation
    )


def test_symmetry_over_random_shape_pairs():
    rng = np.random.default_rng(2024)
    hits = 0
    for _ in range(400):
        a = random_shape(rng)
        b = random_shape(rng)
        dab = shapes_distance(a, b)
        dba = shapes_distance(b, a)
        assert dab == pytest.approx(dba, abs=1e-9)
        iab = shapes_intersect(a, b)
        assert iab == shapes_intersect(b, a)
        hits += bool(iab)
        if dab > 1e-7:
            assert not iab
        if dab == 0.0:
            assert iab
    # The sweep must actually exercise both branches.
    assert 20 < hits < 380


def test_distance_is_translation_invariant():
    rng = np.random.default_rng(5)
    for _ in range(50):
        a = random_shape(rng)
        b = random_shape(rng)
        offset = rng.uniform(-10.0, 10.0, size=3)
        assert shapes_distance(a, b) == pytest.approx(
            shapes_distance(a.translated(offset), b.translated(offset)), abs=1e-8
        )


# --------------------------------------------------------------------------- #
# Degenerate shapes
# --------------------------------------------------------------------------- #
def test_zero_height_capsule_behaves_like_a_sphere():
    capsule = Capsule([1.0, 2.0, 3.0], 0.4, 0.0)
    sphere = Sphere([1.0, 2.0, 3.0], 0.4)
    a, b = capsule.segment()
    assert np.allclose(a, b)

    probes = [
        Sphere([4.0, 2.0, 3.0], 0.5),
        Sphere([1.0, 2.0, 3.6], 0.2),
        Box([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
        Capsule([1.0, 2.0, 3.0], 0.1, 0.0),
    ]
    for probe in probes:
        assert shapes_distance(capsule, probe) == pytest.approx(
            shapes_distance(sphere, probe), abs=1e-9
        )
        assert shapes_intersect(capsule, probe) == shapes_intersect(sphere, probe)


def test_zero_radius_sphere_is_a_point():
    point = Sphere([2.0, 0.0, 0.0], 0.0)
    box = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    assert shapes_distance(point, box) == pytest.approx(1.0, abs=TOL)
    assert not shapes_intersect(point, box)

    on_surface = Sphere([1.0, 0.0, 0.0], 0.0)
    assert shapes_distance(on_surface, box) == pytest.approx(0.0, abs=TOL)
    assert shapes_intersect(on_surface, box)

    inside = Sphere([0.25, 0.25, 0.25], 0.0)
    assert shapes_intersect(inside, box)
    assert shapes_distance(inside, box) == 0.0

    # Two coincident points.
    assert shapes_intersect(point, Sphere([2.0, 0.0, 0.0], 0.0))
    assert shapes_distance(point, Sphere([2.0, 1.0, 0.0], 0.0)) == pytest.approx(1.0, abs=TOL)


def test_very_thin_box():
    thin = Box([0.0, 0.0, 0.0], [1e-6, 2.0, 2.0])
    near = Sphere([0.5, 0.0, 0.0], 0.25)
    assert shapes_distance(near, thin) == pytest.approx(0.5 - 5e-7 - 0.25, abs=1e-9)
    assert not shapes_intersect(near, thin)
    assert shapes_intersect(Sphere([0.2, 0.0, 0.0], 0.25), thin)

    # Thin box against a thin box, offset out of plane.
    other = Box([0.25, 0.0, 0.0], [1e-6, 2.0, 2.0])
    assert shapes_distance(thin, other) == pytest.approx(0.25 - 1e-6, abs=1e-9)
    assert not shapes_intersect(thin, other)

    # A degenerate (zero extent) box is a plane patch.
    flat = Box([0.0, 0.0, 0.0], [0.0, 2.0, 2.0])
    assert shapes_distance(Sphere([1.0, 0.0, 0.0], 0.25), flat) == pytest.approx(0.75, abs=TOL)
    assert shapes_intersect(Sphere([0.1, 0.0, 0.0], 0.25), flat)

    # A fully degenerate box collapses to a point.
    dot = Box([1.0, 1.0, 1.0], [0.0, 0.0, 0.0])
    assert shapes_distance(dot, Sphere([1.0, 1.0, 3.0], 0.5)) == pytest.approx(1.5, abs=TOL)


@pytest.mark.parametrize("gap", [1e-3, 1e-5, 1e-7, 1e-9])
def test_tiny_separations_are_resolved_not_snapped_to_zero(gap):
    """Small positive distances must survive rather than collapse to contact."""
    box = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])

    other = Box([2.0 + gap, 0.0, 0.0], [2.0, 2.0, 2.0])
    assert shapes_distance(box, other) == pytest.approx(gap, rel=1e-6)
    assert not shapes_intersect(box, other)

    sphere = Sphere([1.5 + gap, 0.0, 0.0], 0.5)
    assert shapes_distance(box, sphere) == pytest.approx(gap, rel=1e-6)

    # Same test along the diagonal of a rotated box, where the closest features
    # are two corners rather than two faces.
    yaw = rotation_from_euler(yaw=np.pi / 4)
    a = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], yaw)
    b = Box([SQRT2 * (2.0 + gap), 0.0, 0.0], [2.0, 2.0, 2.0], yaw)
    assert shapes_distance(a, b) == pytest.approx(SQRT2 * gap, rel=1e-5)

    capsule = Capsule([0.0, 0.0, 0.0], 0.5, 2.0)
    near = Capsule([1.0 + gap, 0.0, 0.0], 0.5, 2.0)
    assert shapes_distance(capsule, near) == pytest.approx(gap, rel=1e-6)


def test_zero_extent_capsule_and_cylinder_edge_cases():
    # Zero-height cylinder is a disc.
    disc = Cylinder([0.0, 0.0, 0.0], 1.0, 0.0)
    assert shapes_distance(disc, Sphere([0.0, 0.0, 2.0], 0.5)) == pytest.approx(1.5, abs=TOL)
    assert shapes_distance(disc, Sphere([3.0, 0.0, 0.0], 0.5)) == pytest.approx(1.5, abs=TOL)

    # Zero-radius cylinder is a segment.
    needle = Cylinder([0.0, 0.0, 0.0], 0.0, 2.0)
    assert shapes_distance(needle, Sphere([2.0, 0.0, 0.0], 0.5)) == pytest.approx(1.5, abs=TOL)


# --------------------------------------------------------------------------- #
# GJK behaviour and integration with geometry.shapes_collide
# --------------------------------------------------------------------------- #
def test_triangle_wrapper_is_a_convex_shape():
    """The internal triangle wrapper must satisfy the shape protocol GJK uses."""
    verts = np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [0.0, 3.0, 1.0]])
    tri = _Triangle(verts)
    assert tri.margin == 0.0
    assert np.allclose(tri.support(np.array([1.0, 0.0, 0.0])), verts[1])
    assert np.allclose(tri.support(np.array([0.0, 1.0, 0.0])), verts[2])
    assert np.allclose(tri.support(np.array([-1.0, -1.0, 0.0])), verts[0])
    lo, hi = tri.aabb()
    assert np.allclose(lo, [0.0, 0.0, 1.0])
    assert np.allclose(hi, [2.0, 3.0, 1.0])
    assert np.allclose(tri.core_points(), verts)
    assert np.allclose(tri.center, verts.mean(axis=0))

    # Triangles flow through the same GJK path as any other convex core.
    assert gjk_distance(tri, Sphere([0.5, 0.5, 4.0], 0.0)) == pytest.approx(3.0, abs=TOL)
    assert gjk_distance(tri, Box([0.5, 0.5, 3.0], [1.0, 1.0, 1.0])) == pytest.approx(
        1.5, abs=TOL
    )

    # A cached AABB is reported verbatim.
    cached = _Triangle().set_vertices(verts, np.full(3, -9.0), np.full(3, 9.0))
    lo, hi = cached.aabb()
    assert np.allclose(lo, -9.0) and np.allclose(hi, 9.0)


def test_gjk_returns_zero_for_intersecting_cores():
    a = Box([0.0, 0.0, 0.0], [2.0, 2.0, 2.0])
    b = Box([0.5, 0.5, 0.5], [2.0, 2.0, 2.0], rotation_from_euler(0.3, 0.4, 0.5))
    assert gjk_distance(a, b) == 0.0
    # Cores touching at a single corner.
    assert gjk_distance(a, Box([2.0, 2.0, 2.0], [2.0, 2.0, 2.0])) == pytest.approx(0.0, abs=1e-9)


def test_gjk_respects_max_iter_and_tolerance():
    a = Cylinder([0.0, 0.0, 0.0], 1.0, 2.0, rotation_from_euler(0.3, 0.4, 0.5))
    b = Sphere([4.0, 1.0, 0.5], 0.5)
    exact = gjk_distance(a, b)
    coarse = gjk_distance(a, b, max_iter=2, tol=1e-2)
    assert coarse >= exact - 1e-12  # the simplex value is an upper bound
    assert gjk_distance(a, b, max_iter=64, tol=1e-12) == pytest.approx(exact, abs=1e-9)


def test_geometry_shapes_collide_delegates_here():
    a = Sphere([0.0, 0.0, 0.0], 1.0)
    b = Sphere([1.5, 0.0, 0.0], 1.0)
    c = Box([4.0, 0.0, 0.0], [1.0, 1.0, 1.0])
    assert shapes_collide(a, b) is shapes_intersect(a, b) is True
    assert shapes_collide(a, c) is shapes_intersect(a, c) is False
