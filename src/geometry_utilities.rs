use std::f32::consts::PI;
use std::iter::IntoIterator;

use kiss3d::nalgebra::{distance, Point3, Vector3};
use kiss3d::ncollide3d::shape::Tetrahedron;
use rand::Rng;
use std::prelude::v1::Vec;

struct ConicalFrustrum<N> {
    half_height: N,
    radius_1: N,
    radius_2: N,
}

struct ConicalFrustrumDiscretizationParameters {
    n_segments: usize,
    make_caps: bool,
}

pub fn elevation_azimuth_vector(elevation: f32, azimuth: f32) -> Vector3<f32> {
    Vector3::new(
        azimuth.cos() * elevation.cos(),
        azimuth.sin() * elevation.cos(),
        elevation.sin(),
    )
}

pub fn bounding_tetrahedron<'a, I>(points: I) -> Tetrahedron<f32> where I: IntoIterator<Item=&'a Point3<f32>> {
    let plane_normals = [
        elevation_azimuth_vector(PI / 2.0, 0.0),
        elevation_azimuth_vector(-PI / 6.0, 0.0),
        elevation_azimuth_vector(-PI / 6.0, 2.0 * PI / 3.0),
        elevation_azimuth_vector(-PI / 6.0, -2.0 * PI / 3.0),
    ];

    let center = Point3::new(0.0, 0.0, 0.0);

    let t = points.into_iter().map(|pt| distance(pt, &center)).max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) * 5.0;

    Tetrahedron::new(
        center + t * plane_normals[0],
        center + t * plane_normals[1],
        center + t * plane_normals[2],
        center + t * plane_normals[3],
    )
}

pub fn point_inside_tetrahedron(tet: &Tetrahedron<f32>, pt: &Point3<f32>) -> bool {
    let points = [&tet.a, &tet.b, &tet.c, &tet.d];

    (0..4).all(|i| {
        let a1 = points[i];
        let b1 = points[(i + 1) % 4];
        let c1 = points[(i + 2) % 4];
        let d1 = points[(i + 3) % 4];

        let normal = (b1 - a1).cross(&(c1 - a1));

        (pt - a1).dot(&normal).signum() == (d1 - a1).dot(&normal).signum()
    })
}

pub(crate) fn gen_points<R: Rng>(mut rng: &mut R) -> Vec<Point3<f32>> {
    (0..100).map(|_| {
        Point3::new(
            rng.gen_range(-10.0..10.0),
            rng.gen_range(-10.0..10.0),
            rng.gen_range(-10.0..10.0),
        )
    }).collect()
}

#[cfg(test)]
mod tests {
    use std::convert::From;

    use rand::{Rng, thread_rng};

    use crate::geometry_utilities::{bounding_tetrahedron, elevation_azimuth_vector, point_inside_tetrahedron};

    use super::*;

    #[test]
    fn containing_tetrahedron() {
        let points = gen_points(&mut thread_rng());

        let tet = bounding_tetrahedron(&points);

        dbg!(tet);

        for pt in points {
            dbg!(pt);
            assert!(point_inside_tetrahedron(&tet, &pt));
        }
    }

    #[test]
    fn tet_contains_center() {
        let tet = Tetrahedron::new(Point3::new(1.0, 0.0, 0.0),
                                   Point3::new(-1.0, 0.0, 0.0),
                                   Point3::new(0.0, 1.0, 0.0),
                                   Point3::new(0.0, 0.0, 1.0));

        let cog = Point3::from(
            (tet.a.coords + tet.b.coords + tet.c.coords + tet.d.coords) / 4.0
        );

        assert!(point_inside_tetrahedron(&tet, &cog));

        let mut rng = thread_rng();

        for _ in 0..100 {
            let v = elevation_azimuth_vector(rng.gen_range(-PI / 2.0..PI / 2.0), rng.gen_range(-PI..PI));
            dbg!(&v);
            assert!(!point_inside_tetrahedron(&tet, &Point3::from(v * 100.0)));
        }
    }
}