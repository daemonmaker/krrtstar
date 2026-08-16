//! Native collision scene backed by parry3d.
//!
//! A scene holds two shape sets: the *robot* shapes, expressed in the robot
//! frame, and the *obstacle* shapes, expressed in world coordinates. A query
//! supplies the robot's world transform, so the planner's state -> pose mapping
//! stays on the Python side (see `krrtstar.geometry.PoseMapping`).
//!
//! Axis conventions differ between the Python shape model and parry:
//! `krrtstar.geometry.Capsule`/`Cylinder` put their axis on the local **Z**
//! axis, whereas parry's `Capsule`/`Cylinder` are aligned with **Y**. Every
//! such shape therefore carries an extra rotation mapping Z onto Y (see
//! [`Z_TO_Y`]).
//!
//! The `parry3d` dependency is the 64-bit build (`parry3d-f64` renamed to
//! `parry3d`): the Python model is `float64` throughout, and the pure-Python
//! GJK narrow phase is the reference these results are compared against, so
//! single precision would introduce disagreements well above the geometric
//! tolerances used there.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use parry3d::bounding_volume::{Aabb, BoundingVolume};
use parry3d::query::intersection_test;
use parry3d::shape::{Ball, Capsule, Cuboid, Cylinder, SharedShape, TriMesh};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

/// Rotation taking the local Z axis onto the local Y axis, i.e. from the
/// Python (Z-aligned) capsule/cylinder frame into parry's (Y-aligned) one.
///
/// Both shapes are symmetric about their centre, so the sign of the resulting
/// axis is irrelevant.
fn z_to_y() -> Rotation3<f64> {
    Rotation3::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2)
}

/// One registered shape together with its placement in the frame it was added
/// in (the robot frame for robot shapes, the world frame for obstacles).
struct Entry {
    shape: SharedShape,
    iso: Isometry3<f64>,
    /// AABB of `shape` at `iso`, in that same frame.
    aabb: Aabb,
}

impl Entry {
    fn new(shape: SharedShape, iso: Isometry3<f64>) -> Self {
        let aabb = shape.compute_aabb(&iso);
        Self { shape, iso, aabb }
    }
}

/// Which set a shape belongs to.
enum Side {
    Robot,
    Obstacle,
}

fn parse_side(side: &str) -> PyResult<Side> {
    match side {
        "robot" => Ok(Side::Robot),
        "obstacle" => Ok(Side::Obstacle),
        other => Err(PyValueError::new_err(format!(
            "side must be 'robot' or 'obstacle', got {other:?}"
        ))),
    }
}

/// Read a 3-vector, rejecting any other length.
fn read_vec3(arr: &PyReadonlyArray1<f64>, name: &str) -> PyResult<Vector3<f64>> {
    let a = arr.as_array();
    if a.len() != 3 {
        return Err(PyValueError::new_err(format!(
            "{name} must have 3 elements, got {}",
            a.len()
        )));
    }
    Ok(Vector3::new(a[0], a[1], a[2]))
}

/// Read a 3x3 rotation matrix flattened row-major into nine values.
fn read_rotation(rot9: &PyReadonlyArray1<f64>) -> PyResult<Rotation3<f64>> {
    let r = rot9.as_array();
    if r.len() != 9 {
        return Err(PyValueError::new_err(format!(
            "rotation must be a flattened 3x3 matrix (9 elements), got {}",
            r.len()
        )));
    }
    // `Matrix3::new` takes its arguments row-major, matching the input layout.
    let m = Matrix3::new(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    Ok(Rotation3::from_matrix_unchecked(m))
}

fn isometry(rotation: Rotation3<f64>, translation: Vector3<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(translation),
        UnitQuaternion::from_rotation_matrix(&rotation),
    )
}

/// Reject non-finite or non-positive shape dimensions early: parry panics or
/// silently misbehaves on degenerate shapes.
fn check_positive(value: f64, name: &str) -> PyResult<()> {
    if !value.is_finite() || value <= 0.0 {
        return Err(PyValueError::new_err(format!(
            "{name} must be finite and positive, got {value}"
        )));
    }
    Ok(())
}

/// A collision scene: robot shapes in the robot frame, obstacles in world
/// coordinates.
#[pyclass]
pub struct CollisionScene {
    robot: Vec<Entry>,
    obstacles: Vec<Entry>,
    /// Union of all obstacle AABBs, used as a cheap first reject.
    obstacle_bound: Option<Aabb>,
}

impl CollisionScene {
    fn push(&mut self, side: Side, entry: Entry) {
        match side {
            Side::Robot => self.robot.push(entry),
            Side::Obstacle => {
                self.obstacles.push(entry);
                self.obstacle_bound = None;
            }
        }
    }

    /// True when any robot shape, placed by `query`, intersects an obstacle.
    fn collides(&self, query: &Isometry3<f64>) -> PyResult<bool> {
        for r in &self.robot {
            let world = query * r.iso;
            let aabb = r.shape.compute_aabb(&world);
            if let Some(bound) = &self.obstacle_bound {
                if !aabb.intersects(bound) {
                    continue;
                }
            }
            for o in &self.obstacles {
                // Broad phase: skip clearly separated pairs.
                if !aabb.intersects(&o.aabb) {
                    continue;
                }
                let hit = intersection_test(&world, r.shape.as_ref(), &o.iso, o.shape.as_ref())
                    .map_err(|_| {
                        PyRuntimeError::new_err(
                            "parry3d does not support an intersection test between this pair of shapes",
                        )
                    })?;
                if hit {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }
}

#[pymethods]
impl CollisionScene {
    #[new]
    fn new() -> Self {
        Self {
            robot: Vec::new(),
            obstacles: Vec::new(),
            obstacle_bound: None,
        }
    }

    fn add_sphere(
        &mut self,
        side: &str,
        center: PyReadonlyArray1<f64>,
        radius: f64,
    ) -> PyResult<()> {
        let side = parse_side(side)?;
        let center = read_vec3(&center, "center")?;
        check_positive(radius, "radius")?;
        let iso = isometry(Rotation3::identity(), center);
        self.push(side, Entry::new(SharedShape::new(Ball::new(radius)), iso));
        Ok(())
    }

    fn add_box(
        &mut self,
        side: &str,
        center: PyReadonlyArray1<f64>,
        rot9: PyReadonlyArray1<f64>,
        extents: PyReadonlyArray1<f64>,
    ) -> PyResult<()> {
        let side = parse_side(side)?;
        let center = read_vec3(&center, "center")?;
        let rotation = read_rotation(&rot9)?;
        let extents = read_vec3(&extents, "extents")?;
        for (i, e) in extents.iter().enumerate() {
            check_positive(*e, &format!("extents[{i}]"))?;
        }
        let iso = isometry(rotation, center);
        self.push(
            side,
            Entry::new(SharedShape::new(Cuboid::new(extents / 2.0)), iso),
        );
        Ok(())
    }

    /// `height` is the length of the inner segment, excluding the end caps,
    /// matching `krrtstar.geometry.Capsule`.
    fn add_capsule(
        &mut self,
        side: &str,
        center: PyReadonlyArray1<f64>,
        rot9: PyReadonlyArray1<f64>,
        radius: f64,
        height: f64,
    ) -> PyResult<()> {
        let side = parse_side(side)?;
        let center = read_vec3(&center, "center")?;
        let rotation = read_rotation(&rot9)?;
        check_positive(radius, "radius")?;
        check_positive(height, "height")?;
        let iso = isometry(rotation * z_to_y(), center);
        self.push(
            side,
            Entry::new(SharedShape::new(Capsule::new_y(height / 2.0, radius)), iso),
        );
        Ok(())
    }

    /// `height` is the full height, matching `krrtstar.geometry.Cylinder`.
    fn add_cylinder(
        &mut self,
        side: &str,
        center: PyReadonlyArray1<f64>,
        rot9: PyReadonlyArray1<f64>,
        radius: f64,
        height: f64,
    ) -> PyResult<()> {
        let side = parse_side(side)?;
        let center = read_vec3(&center, "center")?;
        let rotation = read_rotation(&rot9)?;
        check_positive(radius, "radius")?;
        check_positive(height, "height")?;
        let iso = isometry(rotation * z_to_y(), center);
        self.push(
            side,
            Entry::new(SharedShape::new(Cylinder::new(height / 2.0, radius)), iso),
        );
        Ok(())
    }

    fn add_mesh(
        &mut self,
        side: &str,
        center: PyReadonlyArray1<f64>,
        rot9: PyReadonlyArray1<f64>,
        vertices: PyReadonlyArray2<f64>,
        faces: PyReadonlyArray2<u32>,
        scale: f64,
    ) -> PyResult<()> {
        let side = parse_side(side)?;
        let center = read_vec3(&center, "center")?;
        let rotation = read_rotation(&rot9)?;
        if !scale.is_finite() {
            return Err(PyValueError::new_err(format!(
                "scale must be finite, got {scale}"
            )));
        }

        let v = vertices.as_array();
        let f = faces.as_array();
        if v.ncols() != 3 {
            return Err(PyValueError::new_err(format!(
                "vertices must be (n, 3), got (_, {})",
                v.ncols()
            )));
        }
        if f.ncols() != 3 {
            return Err(PyValueError::new_err(format!(
                "faces must be (m, 3), got (_, {})",
                f.ncols()
            )));
        }
        if v.nrows() == 0 || f.nrows() == 0 {
            return Err(PyValueError::new_err(
                "mesh must have at least one vertex and one face",
            ));
        }

        let points: Vec<Point3<f64>> = (0..v.nrows())
            .map(|i| Point3::new(v[[i, 0]] * scale, v[[i, 1]] * scale, v[[i, 2]] * scale))
            .collect();
        let mut indices: Vec<[u32; 3]> = Vec::with_capacity(f.nrows());
        for i in 0..f.nrows() {
            let tri = [f[[i, 0]], f[[i, 1]], f[[i, 2]]];
            for idx in tri {
                if idx as usize >= points.len() {
                    return Err(PyValueError::new_err(format!(
                        "face index {idx} is out of range for {} vertices",
                        points.len()
                    )));
                }
            }
            indices.push(tri);
        }

        let iso = isometry(rotation, center);
        self.push(
            side,
            Entry::new(SharedShape::new(TriMesh::new(points, indices)), iso),
        );
        Ok(())
    }

    /// Precompute the broad-phase bound over all obstacles.
    fn finalize(&mut self) {
        self.obstacle_bound = self.obstacles.iter().fold(None, |acc, o| match acc {
            None => Some(o.aabb),
            Some(bound) => Some(bound.merged(&o.aabb)),
        });
    }

    /// True when the robot, placed at `(rot9, translation)`, collides with any
    /// obstacle. Exact touching counts as a collision.
    fn in_collision_at(
        &self,
        rot9: PyReadonlyArray1<f64>,
        translation: PyReadonlyArray1<f64>,
    ) -> PyResult<bool> {
        let rotation = read_rotation(&rot9)?;
        let translation = read_vec3(&translation, "translation")?;
        self.collides(&isometry(rotation, translation))
    }

    /// True as soon as one pose of the sampled trajectory collides.
    ///
    /// `rot9s` is `(n, 9)` (row-major 3x3 rotations) and `translations` is
    /// `(n, 3)`.
    fn trajectory_in_collision(
        &self,
        rot9s: PyReadonlyArray2<f64>,
        translations: PyReadonlyArray2<f64>,
    ) -> PyResult<bool> {
        let r = rot9s.as_array();
        let t = translations.as_array();
        if r.ncols() != 9 {
            return Err(PyValueError::new_err(format!(
                "rot9s must be (n, 9), got (_, {})",
                r.ncols()
            )));
        }
        if t.ncols() != 3 {
            return Err(PyValueError::new_err(format!(
                "translations must be (n, 3), got (_, {})",
                t.ncols()
            )));
        }
        if r.nrows() != t.nrows() {
            return Err(PyValueError::new_err(format!(
                "rot9s and translations must have the same number of rows, got {} and {}",
                r.nrows(),
                t.nrows()
            )));
        }

        for i in 0..r.nrows() {
            let m = Matrix3::new(
                r[[i, 0]],
                r[[i, 1]],
                r[[i, 2]],
                r[[i, 3]],
                r[[i, 4]],
                r[[i, 5]],
                r[[i, 6]],
                r[[i, 7]],
                r[[i, 8]],
            );
            let iso = isometry(
                Rotation3::from_matrix_unchecked(m),
                Vector3::new(t[[i, 0]], t[[i, 1]], t[[i, 2]]),
            );
            if self.collides(&iso)? {
                return Ok(true);
            }
        }
        Ok(false)
    }
}
