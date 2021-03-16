#[feature(array_map)]
use std::prelude::v1::Vec;

use kdtree::KdTree;
use kiss3d::nalgebra::Point3;
use ncollide3d::shape::{Tetrahedron, Triangle};
use std::convert::From;
use slotmap::{SlotMap, new_key_type};
use std::option::Option;
use std::option::Option::{None, Some};
use crate::geometry_utilities::point_inside_tetrahedron;
use std::collections::HashSet;
use array_init::array_init;
use itertools::Itertools;
use nalgebra::{Vector3, Isometry3};
use ncollide3d::query::{RayCast, Ray};
use std::f32::INFINITY;
use std::iter::IntoIterator;

#[derive(Debug)]
struct Vertex {
    position: Point3<f32>,
    hulls: HashSet<TetrahedronKey>,
}

#[derive(Debug, Clone)]
struct IndexTriangle {
    points: [VertexKey; 3]
}

impl IndexTriangle {
    fn same_vertices(&self, other: &IndexTriangle) -> bool {
        self.points.iter().sorted().zip_eq(other.points.iter().sorted()).all(|(a, b)| a == b)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
struct FaceKey {
    tet: TetrahedronKey,
    face: usize,
}

#[derive(Debug, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
struct IndexTetrahedron {
    points: [VertexKey; 4],
    neighbours: [Option<FaceKey>; 4],
}

impl IndexTetrahedron {
    fn faces(&self) -> [IndexTriangle; 4] {
        [
            // Indexing should be identical to Tetrahedron::face()
            IndexTriangle { points: [self.points[0], self.points[1], self.points[2]] },
            IndexTriangle { points: [self.points[0], self.points[1], self.points[3]] },
            IndexTriangle { points: [self.points[0], self.points[2], self.points[3]] },
            IndexTriangle { points: [self.points[1], self.points[2], self.points[3]] },
        ]
    }

    fn far_side(&self, v: VertexKey) -> IndexTriangle {
        self.faces()
            .into_iter()
            .cloned()
            .find(|tri| !tri.points.contains(&v))
            .expect("Given vertex should be in the tetrahedron.")
    }
}

new_key_type! { pub struct VertexKey; }
new_key_type! { pub struct TetrahedronKey; }

#[derive(Debug)]
pub struct TetrahedralMesh {
    points: SlotMap<VertexKey, Vertex>,
    tets: SlotMap<TetrahedronKey, IndexTetrahedron>,
}

impl TetrahedralMesh {
    pub(crate) fn empty() -> Self {
        TetrahedralMesh {
            points: SlotMap::with_key(),
            tets: SlotMap::with_key(),
        }
    }

    pub(crate) fn add_free_tetrahedron(&mut self, tet: Tetrahedron<f32>) -> TetrahedronKey {
        let Tetrahedron { a, b, c, d } = tet;

        let a = self.create_vertex(a);
        let b = self.create_vertex(b);
        let c = self.create_vertex(c);
        let d = self.create_vertex(d);

        let tet_id = self.tets.insert(IndexTetrahedron {
            points: [a, b, c, d],
            neighbours: [None; 4],
        });

        self.points[a].hulls.insert(tet_id);
        self.points[b].hulls.insert(tet_id);
        self.points[c].hulls.insert(tet_id);
        self.points[d].hulls.insert(tet_id);

        tet_id
    }

    pub fn points_share_tetrahedron(&self, v1: VertexKey, v2: VertexKey) -> Option<TetrahedronKey> {
        self.points[v1].hulls.intersection(&self.points[v2].hulls).cloned().next()
    }

    pub fn find_tet_containing(&self, pt: &Point3<f32>) -> Option<TetrahedronKey> {
        // TODO Surely I can do better than O(n)?
        self.tets.iter().find_map(|(ti, tt)| {
            if point_inside_tetrahedron(&self.extract_cell(tt), &pt) {
                Some(ti)
            } else {
                None
            }
        })
    }

    pub fn split_cell(&mut self, cell_id: TetrahedronKey, point: Point3<f32>) -> (VertexKey, [TetrahedronKey; 4]) {
        let cell = self.tets.remove(cell_id).expect("Tetrahedron key invalid.");

        for vid in cell.points.iter() {
            self.points[*vid].hulls.remove(&cell_id);
        }

        let new_vid = self.create_vertex(point);

        let face_vertices = cell.faces();

        let new_tets: [TetrahedronKey; 4] = array_init(|i| {
            let tri = &face_vertices[i];

            let tet_id = self.tets.insert(IndexTetrahedron {
                points: [tri.points[0], tri.points[1], tri.points[2], new_vid],
                neighbours: [
                    cell.neighbours[i],
                    None,
                    None,
                    None,
                ],
            });

            for pt in self.tets[tet_id].points.iter() {
                self.points[*pt].hulls.insert(tet_id);
            }

            if let Some(nb) = cell.neighbours[i] {
                self.tets[nb.tet].neighbours[nb.face] = Some(FaceKey {
                    tet: tet_id,
                    face: 0,
                });
            }

            tet_id
        });

        self.fix_neighbours(&new_tets);

        (new_vid, new_tets)
    }

    fn fix_neighbours(&mut self, cells: &[TetrahedronKey]) {
        for ta in cells.iter() {
            for (i, tra) in self.tets[*ta].faces().iter().enumerate() {
                for tb in cells.iter() {
                    if ta != tb {
                        for (j, trb) in self.tets[*tb].faces().iter().enumerate() {
                            if tra.same_vertices(trb) {
                                self.tets[*ta].neighbours[i] = Some(FaceKey {
                                    tet: *tb,
                                    face: j,
                                })
                            }
                        }
                    }
                }
            }
        }
    }

    fn extract_cell(&self, tt: &IndexTetrahedron) -> Tetrahedron<f32> {
        Tetrahedron {
            a: self.points[tt.points[0]].position,
            b: self.points[tt.points[1]].position,
            c: self.points[tt.points[2]].position,
            d: self.points[tt.points[3]].position,
        }
    }

    fn create_vertex(&mut self, pt: Point3<f32>) -> VertexKey {
        self.points.insert(Vertex { position: pt, hulls: HashSet::new() })
    }

    pub(crate) fn walk_from_vertex(&self, vertex: VertexKey, direction: Vector3<f32>) -> Option<TetrahedronKey> {
        let vpos = self.points[vertex].position;
        let ray = Ray::new(vpos, direction);

        self.points[vertex].hulls.iter().cloned().find(|tk| {
            let tri: IndexTriangle = self.tets[*tk].far_side(vertex);
            let tri = Triangle::new(self.points[tri.points[0]].position, self.points[tri.points[1]].position, self.points[tri.points[2]].position);
            tri.intersects_ray(&Isometry3::identity(), &ray, f32::INFINITY)
        })
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use rand::thread_rng;
    use std::collections::{HashSet, VecDeque};
    use rand::prelude::IteratorRandom;

    fn assert_invariants(mesh: &TetrahedralMesh) {
        for (tet_i, tet) in mesh.tets.iter() {
            for v_i in tet.points.iter() {
                assert!(mesh.points[*v_i].hulls.contains(&tet_i));
            }

            for (i, nb) in tet.neighbours.iter().enumerate() {
                if let Some(nb) = nb {
                    assert_eq!(mesh.tets[nb.tet].neighbours[nb.face], Some(FaceKey { tet: tet_i, face: i }));
                }
            }
        }

        for (p_i, pt) in mesh.points.iter() {
            for m_i in pt.hulls.iter() {
                assert!(mesh.tets[*m_i].points.contains(&p_i));
            }
        }
    }

    #[test]
    fn split_test() {
        let mut mesh = TetrahedralMesh::empty();
        let mut rng = thread_rng();

        let mut tet_refs: VecDeque<TetrahedronKey> = VecDeque::new();
        tet_refs.push_back(mesh.add_free_tetrahedron(
            Tetrahedron::new(Point3::new(0.0, 0.0, 0.0),
                             Point3::new(0.0, 0.0, 0.0),
                             Point3::new(0.0, 0.0, 0.0),
                             Point3::new(0.0, 0.0, 0.0)))
        );

        assert_invariants(&mesh);

        for i in 0..100 {
            let ti = tet_refs.pop_front().unwrap();

            let (vk, [a, b, c, d]) = mesh.split_cell(ti, Point3::new(0.0, 0.0, 0.0));

            tet_refs.push_back(a);
            tet_refs.push_back(b);
            tet_refs.push_back(c);
            tet_refs.push_back(d);

            assert_invariants(&mesh);
        }
    }
}