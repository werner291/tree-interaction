use std::boxed::Box;
use std::iter::{IntoIterator, Iterator};
use std::option::Option::{None, Some};
use std::option::Option;
use std::prelude::v1::Vec;

use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use nalgebra::{Isometry3, Point3, Vector3, distance};
use ncollide3d::query::PointQuery;
use ncollide3d::shape::{Tetrahedron, Plane};

use crate::tetrahedral_mesh::TetrahedralMesh;
use ncollide3d::bounding_volume::{AABB, BoundingSphere};
use std::f32::consts::PI;

//
// enum Subdivision {
//     Leaf,
//     Split {
//         split_point: Point3<f32>,
//         partitions: Box<[Subdivision; 4]>
//     }
// }
//
// struct TetrahedralMesh {
//     boundary: Tetrahedron<f32>,
//     root: Subdivision
// }
//
// impl TetrahedralMesh {
//
//     fn empty() -> TetrahedralMesh {
//         TetrahedralMesh {
//             boundary: Tetrahedron::new(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
//             root: Subdivision::Leaf
//         }
//     }
// }

//

fn elevation_azimuth_vector(elevation: f32, azimuth: f32) -> Vector3<f32> {
    Vector3::new(
        azimuth.cos() * elevation.cos(),
        azimuth.sin() * elevation.cos(),
        elevation.sin(),
    )
}

fn bounding_tetrahedron<'a, I>(points: I) -> Tetrahedron<f32> where I: IntoIterator<Item=&'a Point3<f32>> {
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


fn build_tetrahedral_mesh(points: &[Point3<f32>], edges: &[(usize, usize)]) -> TetrahedralMesh {
    assert!(points.len() >= 4);

    let bounding_tetrahedron = [
        Vector3::new(0.0, 1.0, 0.0),
    ];


    todo!()
}

fn point_inside_tetrahedron(tet: &Tetrahedron<f32>, pt: &Point3<f32>) -> bool {
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

#[cfg(test)]
mod tests {
    use rand::{Rng, thread_rng};

    use super::*;
    use std::convert::From;

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

    #[test]
    fn fuzztest() {
        let mut rng = thread_rng();

        let points = gen_points(&mut rng);

        let mut kdtree = KdTree::new(3);

        for (i, pt) in points.iter().enumerate() {
            kdtree.add(pt.coords.as_slice(), i);
        }

        let edges = points.iter().enumerate().flat_map(|(i, pt)| {
            kdtree.nearest(pt.coords.as_slice(), rng.gen_range(1..5), &squared_euclidean).unwrap()
        });


        // let tets = build_tetrahedral_mesh(&points, &edges);
        //
        // for t in tets.tets {
        //     let t = Tetrahedron
        // }
    }

    fn gen_points<R: Rng>(mut rng: &mut R) -> Vec<Point3<f32>> {
        (0..100).map(|_| {
            Point3::new(
                rng.gen_range(-10.0..10.0),
                rng.gen_range(-10.0..10.0),
                rng.gen_range(-10.0..10.0),
            )
        }).collect()
    }
}

