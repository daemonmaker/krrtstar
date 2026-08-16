"""Pure-Python narrow-phase collision engine (GJK distance).

Every shape in :mod:`krrtstar.geometry` is modelled as a convex *core* set plus a
scalar *margin* (its round radius). Two shapes intersect exactly when the
distance between their cores is at most the sum of their margins, so a single
GJK distance routine over support functions covers spheres, boxes (oriented or
not), capsules, cylinders and triangles.

Meshes are triangle soups and therefore not convex: they are handled by running
GJK per triangle, with axis-aligned bounding box lower bounds used to prune
triangles that cannot beat the best distance found so far.

The public entry points are :func:`gjk_distance`, :func:`shapes_distance` and
:func:`shapes_intersect`; :mod:`krrtstar.geometry` delegates its
``shapes_collide`` to the latter.
"""

from __future__ import annotations

from math import sqrt
from typing import Optional, Sequence, Tuple

import numpy as np

from .geometry import Mesh

__all__ = [
    "closest_point_on_segment",
    "closest_point_on_triangle",
    "closest_point_on_tetrahedron",
    "gjk_distance",
    "shapes_distance",
    "shapes_intersect",
]

# Squared length below which a vector counts as the zero vector.
_ZERO_SQ = 1e-24
# Denominators below this are treated as degenerate (areas/volumes are squared
# quantities, hence the fairly small value).
_DEGENERATE = 1e-30
# Slack that makes exact touching count as a collision.
_TOUCH_EPS = 1e-9

_INF = float("inf")
_DEFAULT_DIRECTION = np.array([1.0, 0.0, 0.0])


# --------------------------------------------------------------------------- #
# Closest point on a simplex to the origin
#
# Each routine returns ``(closest_point, kept)`` where ``kept`` indexes the
# vertices that actually support the closest point, which is what GJK needs in
# order to shrink its simplex.
# --------------------------------------------------------------------------- #
def closest_point_on_segment(
    a: np.ndarray, b: np.ndarray
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    """Closest point of the segment ``ab`` to the origin."""
    ab = b - a
    denom = float(ab @ ab)
    if denom <= _ZERO_SQ:
        return a, (0,)
    t = -float(a @ ab) / denom
    if t <= 0.0:
        return a, (0,)
    if t >= 1.0:
        return b, (1,)
    return a + t * ab, (0, 1)


def _closest_point_on_edges(
    verts: Sequence[np.ndarray], edges: Sequence[Tuple[int, int]]
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    """Best of several segments, used as the degenerate-simplex fallback."""
    best_point = None
    best_kept: Tuple[int, ...] = ()
    best_sq = _INF
    for i, j in edges:
        point, kept = closest_point_on_segment(verts[i], verts[j])
        sq = float(point @ point)
        if sq < best_sq:
            best_sq = sq
            best_point = point
            best_kept = (i,) if kept == (0,) else (j,) if kept == (1,) else (i, j)
    return best_point, best_kept


def closest_point_on_triangle(
    a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    """Closest point of triangle ``abc`` to the origin (Voronoi region test)."""
    ab = b - a
    ac = c - a

    # Vertex region A.
    d1 = -float(ab @ a)
    d2 = -float(ac @ a)
    if d1 <= 0.0 and d2 <= 0.0:
        return a, (0,)

    # Vertex region B.
    d3 = -float(ab @ b)
    d4 = -float(ac @ b)
    if d3 >= 0.0 and d4 <= d3:
        return b, (1,)

    # Edge region AB.
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        denom = d1 - d3
        if denom > 0.0:
            return a + (d1 / denom) * ab, (0, 1)

    # Vertex region C.
    d5 = -float(ab @ c)
    d6 = -float(ac @ c)
    if d6 >= 0.0 and d5 <= d6:
        return c, (2,)

    # Edge region AC.
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        denom = d2 - d6
        if denom > 0.0:
            return a + (d2 / denom) * ac, (0, 2)

    # Edge region BC.
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        denom = (d4 - d3) + (d5 - d6)
        if denom > 0.0:
            return b + ((d4 - d3) / denom) * (c - b), (1, 2)

    # Face region.
    denom = va + vb + vc
    if denom <= _DEGENERATE:
        # Collinear or duplicated vertices: the Voronoi tests above can all
        # fail, so fall back to the three edges.
        return _closest_point_on_edges((a, b, c), ((0, 1), (0, 2), (1, 2)))
    v = vb / denom
    w = vc / denom
    return a + v * ab + w * ac, (0, 1, 2)


_TETRA_FACES = ((1, 2, 3), (0, 2, 3), (0, 1, 3), (0, 1, 2))


def closest_point_on_tetrahedron(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    """Closest point of tetrahedron ``abcd`` to the origin.

    Returns the origin itself (with all four vertices kept) when the origin is
    contained in the tetrahedron.
    """
    verts = (a, b, c, d)
    basis = np.empty((3, 3))
    basis[:, 0] = b - a
    basis[:, 1] = c - a
    basis[:, 2] = d - a

    faces = _TETRA_FACES
    if abs(float(np.linalg.det(basis))) > _DEGENERATE:
        try:
            lam = np.linalg.solve(basis, -a)
        except np.linalg.LinAlgError:  # pragma: no cover - guarded by det
            lam = None
        if lam is not None:
            bary = (1.0 - float(lam.sum()), float(lam[0]), float(lam[1]), float(lam[2]))
            outside = tuple(i for i, value in enumerate(bary) if value < 0.0)
            if not outside:
                return np.zeros(3), (0, 1, 2, 3)
            faces = tuple(_TETRA_FACES[i] for i in outside)

    best_point = None
    best_kept: Tuple[int, ...] = ()
    best_sq = _INF
    for face in faces:
        i, j, k = face
        point, kept = closest_point_on_triangle(verts[i], verts[j], verts[k])
        sq = float(point @ point)
        if sq < best_sq:
            best_sq = sq
            best_point = point
            best_kept = tuple(face[m] for m in kept)
    return best_point, best_kept


def _closest_point_on_simplex(
    simplex: Sequence[np.ndarray],
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    n = len(simplex)
    if n == 1:
        return simplex[0], (0,)
    if n == 2:
        return closest_point_on_segment(simplex[0], simplex[1])
    if n == 3:
        return closest_point_on_triangle(simplex[0], simplex[1], simplex[2])
    return closest_point_on_tetrahedron(simplex[0], simplex[1], simplex[2], simplex[3])


# --------------------------------------------------------------------------- #
# Convex wrapper for a single mesh triangle
# --------------------------------------------------------------------------- #
class _Triangle:
    """Convex core of one triangle, exposing the shape protocol GJK needs."""

    __slots__ = ("vertices", "_lo", "_hi")

    margin = 0.0

    def __init__(self, vertices: Optional[np.ndarray] = None) -> None:
        self.vertices = None
        self._lo = None
        self._hi = None
        if vertices is not None:
            self.set_vertices(np.asarray(vertices, float).reshape(3, 3))

    def set_vertices(
        self, vertices: np.ndarray, lo: Optional[np.ndarray] = None, hi: Optional[np.ndarray] = None
    ) -> "_Triangle":
        """Point the wrapper at new vertices, optionally with a cached AABB."""
        self.vertices = vertices
        self._lo = lo
        self._hi = hi
        return self

    @property
    def center(self) -> np.ndarray:
        return self.vertices.mean(axis=0)

    def support(self, direction: np.ndarray) -> np.ndarray:
        return self.vertices[int(np.argmax(self.vertices @ direction))]

    def core_points(self) -> np.ndarray:
        return self.vertices

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        if self._lo is None:
            return self.vertices.min(axis=0), self.vertices.max(axis=0)
        return self._lo, self._hi


# --------------------------------------------------------------------------- #
# GJK
# --------------------------------------------------------------------------- #
def _initial_direction(shape_a, shape_b) -> np.ndarray:
    """A direction pointing from ``shape_a`` towards ``shape_b``."""
    ca = getattr(shape_a, "center", None)
    cb = getattr(shape_b, "center", None)
    if ca is not None and cb is not None:
        d = np.asarray(cb, float) - np.asarray(ca, float)
        if float(d @ d) > _ZERO_SQ:
            return d
    return _DEFAULT_DIRECTION


def _gjk(shape_a, shape_b, max_iter: int, tol: float, cutoff: float) -> float:
    """GJK distance between two convex cores.

    ``cutoff`` allows an early exit: as soon as a certified lower bound exceeds
    it, that lower bound is returned instead of the exact distance. Callers that
    only need to compare against a threshold (or prune a running minimum) can
    therefore skip the tail of the iteration.
    """
    support_a = shape_a.support
    support_b = shape_b.support

    d = _initial_direction(shape_a, shape_b)
    simplex = [np.asarray(support_a(d), float) - np.asarray(support_b(-d), float)]
    # Invariant maintained below: ``v`` is the closest point of ``simplex`` to
    # the origin, so ``|v|`` is an upper bound on the distance.
    v, _ = _closest_point_on_simplex(simplex)
    # Support points already visited; revisiting one means the search has
    # started to cycle on a degenerate simplex.
    seen = {tuple(simplex[0].tolist())}

    for _ in range(max_iter):
        vv = float(v @ v)
        if vv <= _ZERO_SQ:
            return 0.0
        d = -v
        w = np.asarray(support_a(d), float) - np.asarray(support_b(-d), float)

        vw = float(v @ w)
        # The Minkowski difference lies in the half space ``x @ v >= vw``, so
        # ``vw / |v|`` is a lower bound on the true distance while ``|v|`` is an
        # upper bound.
        if vv - vw <= tol * vv:
            break
        if vw > 0.0 and cutoff < _INF:
            v_norm = sqrt(vv)
            if vw > cutoff * v_norm:
                return vw / v_norm
        key = tuple(w.tolist())
        if key in seen:
            break
        seen.add(key)

        simplex.append(w)
        v, kept = _closest_point_on_simplex(simplex)
        if len(kept) == 4:  # origin enclosed by the simplex
            return 0.0
        if len(kept) != len(simplex):
            simplex = [simplex[i] for i in kept]

    return sqrt(float(v @ v))


def gjk_distance(shape_a, shape_b, max_iter: int = 64, tol: float = 1e-9) -> float:
    """Distance between the convex cores of two shapes, 0.0 if they intersect.

    Only the shapes' ``support`` functions are used, so this works for any
    convex core (spheres degenerate to a point, capsules to a segment).
    """
    return _gjk(shape_a, shape_b, max_iter, tol, _INF)


# --------------------------------------------------------------------------- #
# AABB helpers
# --------------------------------------------------------------------------- #
def _core_aabb(shape) -> Tuple[np.ndarray, np.ndarray]:
    """World AABB of a shape's core (``shape.aabb()`` is margin-inflated)."""
    lo, hi = shape.aabb()
    margin = float(shape.margin)
    if margin == 0.0:
        return lo, hi
    return lo + margin, hi - margin


def _aabbs_overlap(alo, ahi, blo, bhi) -> bool:
    return not (bool(np.any(ahi < blo)) or bool(np.any(bhi < alo)))


def _aabb_gaps(lo, hi, blo, bhi) -> np.ndarray:
    """Distance from each AABB in ``(lo, hi)`` (both ``(n, 3)``) to one AABB."""
    gap = np.maximum(0.0, np.maximum(lo - bhi, blo - hi))
    return np.sqrt(np.einsum("ij,ij->i", gap, gap))


# --------------------------------------------------------------------------- #
# Mesh (triangle soup) core distances
# --------------------------------------------------------------------------- #
def _triangle_shape_core_distance(mesh: Mesh, other, stop_below: float) -> float:
    """Minimum core distance between a mesh's triangles and a convex core."""
    best = _INF
    tris = mesh.triangles()
    if tris.shape[0] == 0:
        return best
    lo = tris.min(axis=1)
    hi = tris.max(axis=1)
    olo, ohi = _core_aabb(other)
    bounds = _aabb_gaps(lo, hi, olo, ohi)

    wrapper = _Triangle()
    for i in np.argsort(bounds):
        if bounds[i] >= best:
            break
        wrapper.set_vertices(tris[i], lo[i], hi[i])
        dist = _gjk(wrapper, other, 64, 1e-9, best)
        if dist < best:
            best = dist
            if best <= stop_below:
                break
    return best


def _mesh_mesh_core_distance(mesh_a: Mesh, mesh_b: Mesh, stop_below: float) -> float:
    """Minimum core distance between the triangles of two meshes."""
    best = _INF
    tris_a = mesh_a.triangles()
    tris_b = mesh_b.triangles()
    if tris_a.shape[0] == 0 or tris_b.shape[0] == 0:
        return best
    alo, ahi = tris_a.min(axis=1), tris_a.max(axis=1)
    blo, bhi = tris_b.min(axis=1), tris_b.max(axis=1)

    outer_bounds = _aabb_gaps(alo, ahi, blo.min(axis=0), bhi.max(axis=0))
    wrap_a = _Triangle()
    wrap_b = _Triangle()
    for i in np.argsort(outer_bounds):
        if outer_bounds[i] >= best:
            break
        inner_bounds = _aabb_gaps(blo, bhi, alo[i], ahi[i])
        order = np.argsort(inner_bounds)
        if inner_bounds[order[0]] >= best:
            continue
        wrap_a.set_vertices(tris_a[i], alo[i], ahi[i])
        for j in order:
            if inner_bounds[j] >= best:
                break
            wrap_b.set_vertices(tris_b[j], blo[j], bhi[j])
            dist = _gjk(wrap_a, wrap_b, 64, 1e-9, best)
            if dist < best:
                best = dist
                if best <= stop_below:
                    return best
    return best


def _core_distance(a, b, stop_below: float = -_INF) -> float:
    """Core-set distance, dispatching meshes to their per-triangle handling."""
    a_mesh = isinstance(a, Mesh)
    b_mesh = isinstance(b, Mesh)
    if a_mesh and b_mesh:
        return _mesh_mesh_core_distance(a, b, stop_below)
    if a_mesh:
        return _triangle_shape_core_distance(a, b, stop_below)
    if b_mesh:
        return _triangle_shape_core_distance(b, a, stop_below)
    # A finite ``stop_below`` means the caller only needs a threshold test, so
    # GJK may stop once its lower bound clears it.
    return _gjk(a, b, 64, 1e-9, stop_below if stop_below > -_INF else _INF)


# --------------------------------------------------------------------------- #
# Public shape queries
# --------------------------------------------------------------------------- #
def shapes_distance(a, b) -> float:
    """Distance between two shapes, 0.0 when they touch or overlap.

    This is the core-set distance reduced by both margins, so a sphere is a
    point core inflated by its radius and a capsule a segment core inflated by
    its radius. A mesh with no faces has no geometry and is infinitely far away.
    """
    margins = float(a.margin) + float(b.margin)
    core = _core_distance(a, b)
    if core == _INF:  # empty mesh
        return _INF
    return max(0.0, core - margins)


def shapes_intersect(a, b) -> bool:
    """Narrow-phase intersection test; touching counts as intersecting."""
    alo, ahi = a.aabb()
    blo, bhi = b.aabb()
    if not _aabbs_overlap(alo, ahi, blo, bhi):
        return False

    margins = float(a.margin) + float(b.margin)
    threshold = margins + _TOUCH_EPS * (1.0 + margins)
    core = _core_distance(a, b, stop_below=threshold)
    return bool(core <= threshold)
