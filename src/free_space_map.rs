use std::boxed::Box;
use std::f32::consts::PI;
use std::iter::{IntoIterator, Iterator};
use std::option::Option::{None, Some};
use std::option::Option;
use std::prelude::v1::Vec;

use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use nalgebra::{distance, Isometry3, Point3, Vector3};
use ncollide3d::bounding_volume::{AABB, BoundingSphere};
use ncollide3d::query::PointQuery;
use ncollide3d::shape::{Plane, Tetrahedron};

use crate::tetrahedral_mesh::{TetrahedralMesh, VertexKey};
use crate::geometry_utilities::{bounding_tetrahedron, gen_points};
use std::convert::From;

fn build_tetrahedral_mesh(points: &[Point3<f32>], edges: &[(usize, usize)]) -> TetrahedralMesh {
    assert!(points.len() >= 4);

    let bounding_tetrahedron = bounding_tetrahedron(points);

    let mut tetmesh = TetrahedralMesh::empty();

    let bounding_tet = tetmesh.add_free_tetrahedron(bounding_tetrahedron);

    let mut points_cache: Vec<Option<VertexKey>> = points.iter().map(|_| None).collect();

    let mut lookup_pt = |i: usize, tetmesh: &mut TetrahedralMesh| {
        match points_cache[i] {
            Some(vk) => {
                vk
            }
            None => {
                let tet = tetmesh.find_tet_containing(&points[i]).expect("Bounding tetrahedron should be big enough.");
                let (vi, _) = tetmesh.split_cell(tet, points[i]);
                points_cache[i] = Some(vi);
                vi
            }
        }
    };

    for (i, j) in edges {
        let from_vertex = lookup_pt(*i, &mut tetmesh);
        let to_vertex = lookup_pt(*j, &mut tetmesh);

        let dir = points[*j] - points[*i];

        while tetmesh.points_share_tetrahedron(from_vertex, to_vertex).is_none() {
            tetmesh.walk_from_vertex(from_vertex, dir);
        }

        //
        //
        // let pt_a = points[*i];
        // let pt_b = points[*j];
        //
        // let from_tet =
        // let to_tet = tetmesh.find_tet_containing(&pt_b).expect("Bounding tetrahedron should be big enough.");
        //
        // let from_vertex = points_cache.[*i].unwrap_or_else(|| {
        //     tetmesh.split_cell(from_tet, pt_a);
        //
        // })
    }

    todo!()
}

#[cfg(test)]
mod tests {
    use std::convert::From;

    use rand::{Rng, thread_rng};

    use crate::geometry_utilities::{bounding_tetrahedron, elevation_azimuth_vector, point_inside_tetrahedron};

    use super::*;

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
}

