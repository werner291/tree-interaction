use std::prelude::v1::Vec;

use kdtree::KdTree;
use kiss3d::nalgebra::Point3;

struct Vertex {
    position: Point3<f32>,
    hulls: Vec<usize>,
}

struct IndexTetrahedron {
    points: [usize; 4]
}

pub struct TetrahedralMesh {
    points: Vec<Vertex>,

    kdtree: KdTree<f32, usize, [f32; 3]>,

    tets: Vec<IndexTetrahedron>,
}
