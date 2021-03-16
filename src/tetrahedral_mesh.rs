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
use std::iter::IntoIterator;

/// A struct representing a Vertex in the tetrahedral mesh. It has a position,
/// as well as a set of references to tetrahedral cells incident to the vertex.
#[derive(Debug)]
struct Vertex {
    position: Point3<f32>,
    incident_cells: HashSet<TetrahedronKey>,
}

/// A struct representing a triangle within the mesh, containing three references to vertices.
///
/// Note that the mesh itself has no explicit concept of a "triangle".
#[derive(Debug, Clone)]
struct IndexTriangle {
    points: [VertexKey; 3]
}

impl IndexTriangle {
    /// Check if the triangles have the same vertex references, independent of order.
    fn same_vertices(&self, other: &IndexTriangle) -> bool {
        self.points.iter().sorted().zip_eq(other.points.iter().sorted()).all(|(a, b)| a == b)
    }
}

/// A reference to a face of a tetrahedral cell.
///
/// Consists of a reference to a cell, and an index in range 0..4,
/// corresponding to an entry of IndexTetrahedron::faces.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
struct FaceKey {
    /// Reference to the tetrahedral cell.
    cell: TetrahedronKey,
    face: usize,
}

/// A tetrahedral cell within the tetrahedral mesh.
///
/// Contains exactly four references to vertices, as well as four "neighbour" references,
/// one for each in IndexTetrahedron::faces(), to a neighbouring cell's face that shares
/// the vertices, if there is such a neighbour.
#[derive(Debug, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
struct IndexTetrahedron {
    points: [VertexKey; 4],
    neighbours: [Option<FaceKey>; 4],
}

impl IndexTetrahedron {
    /// Returns an array of the four faces of the tetrahedron, each represented by an IndexTriangle.
    ///
    /// The indexing matches that of ncollide3d::shape::Tetrahedron::face(usize).
    fn faces(&self) -> [IndexTriangle; 4] {
        [
            IndexTriangle { points: [self.points[0], self.points[1], self.points[2]] },
            IndexTriangle { points: [self.points[0], self.points[1], self.points[3]] },
            IndexTriangle { points: [self.points[0], self.points[2], self.points[3]] },
            IndexTriangle { points: [self.points[1], self.points[2], self.points[3]] },
        ]
    }

    /// Given a vertex, return the IndexTriangle corresponding to the face opposite that
    /// vertex within the cell.
    ///
    /// Panics if the vertex does not appear exactly once in the cell.
    fn far_side(&self, v: VertexKey) -> IndexTriangle {
        assert!(self.points.contains(&v));

        self.faces()
            .into_iter()
            .cloned()
            // The face opposite is the triangle that does not contain the vertex.
            .find(|tri| !tri.points.contains(&v))
            .expect("Vertex must appear twice in the cell.")
    }
}

new_key_type! { pub struct VertexKey; }
new_key_type! { pub struct TetrahedronKey; }

/// A volume mesh consisting of tetrahedral cells, including some bookkeeping to allow for
/// easier manipulation and traversal of the mesh.
///
/// Note the mesh is not guaranteed to be intersection-free, though every cell does have
/// references to its' neighbours (which must be cycle-consistent), and every vertex
/// has references to its' incident cells (must also be cycle-consistent).
///
/// Also, note that the mesh is not necessarily connected.
#[derive(Debug)]
pub struct TetrahedralMesh {
    points: SlotMap<VertexKey, Vertex>,
    cells: SlotMap<TetrahedronKey, IndexTetrahedron>,
}

impl TetrahedralMesh {
    /// Instantiates a TetrahedralMesh without contents.
    pub fn empty() -> Self {
        TetrahedralMesh {
            points: SlotMap::with_key(),
            cells: SlotMap::with_key(),
        }
    }

    /// Creates four new vertices, and then creates a tetrahedral cell out of them.
    ///
    /// The cell and vertices are disconnected from the rest of the mesh, and this method
    /// is valid to call on an empty mesh.
    ///
    /// Operation takes expected O(1) (+ amortized allocations and hashmap insertions)
    pub fn add_free_tetrahedron(&mut self, tet: Tetrahedron<f32>) -> TetrahedronKey {

        // Unpack the vertices.
        let Tetrahedron { a, b, c, d } = tet;

        // Insert the vertices (and create bookkeeping structures) in the mesh and obtain an index.
        let a = self.create_vertex(a);
        let b = self.create_vertex(b);
        let c = self.create_vertex(c);
        let d = self.create_vertex(d);

        // Insert a tetrahedral cell made of the four new vertices.
        let tet_id = self.cells.insert(IndexTetrahedron {
            points: [a, b, c, d],
            neighbours: [None; 4],
        });

        // Update the vertices to refer back to the newly-created cell.
        self.points[a].incident_cells.insert(tet_id);
        self.points[b].incident_cells.insert(tet_id);
        self.points[c].incident_cells.insert(tet_id);
        self.points[d].incident_cells.insert(tet_id);

        tet_id
    }

    /// Check if the two vertices share at least one incident cell. This implies that the mesh
    /// contains a direct edge between the two vertices as well.
    ///
    /// Will take `O(n + m)` time, where `m` and `n` are the number of incident cells to each vertex.
    pub fn points_share_tetrahedron(&self, v1: VertexKey, v2: VertexKey) -> Option<TetrahedronKey> {
        self.points[v1].incident_cells.intersection(&self.points[v2].incident_cells).cloned().next()
    }

    /// Find a tetrahedral cell that spatially contains the given point in its' interior. This cell
    /// should be unique, assuming that cells are disjoint (this is up to the user).
    ///
    /// Takes `O(n)` in the number of cells
    ///
    /// TODO: surely that can be done in around O(log n)?
    pub fn find_tet_containing(&self, pt: &Point3<f32>) -> Option<TetrahedronKey> {
        self.cells.iter().find_map(|(ti, tt)| {
            if point_inside_tetrahedron(&self.extract_cell(tt), &pt) {
                Some(ti)
            } else {
                None
            }
        })
    }

    /// Delete the given cell, and replace it with four new cells built from the given cell's
    /// original face, and a new vertex that is to be created from the given point.
    ///
    /// Returns the index of the given vertex, as well as the indices of the new cells.
    ///
    /// Warning: it is not checked whether the given point lies inside the cell.
    pub fn split_cell(&mut self, cell_id: TetrahedronKey, point: Point3<f32>) -> (VertexKey, [TetrahedronKey; 4]) {

        // Delete the cell and retrieve the struct.
        let cell = self.cells.remove(cell_id).expect("Tetrahedron key invalid.");

        // Remove the cell reference from its' vertices.
        for vid in cell.points.iter() {
            self.points[*vid].incident_cells.remove(&cell_id);
        }

        // Create a new vertex from the given point.
        let new_vid = self.create_vertex(point);

        // Extract the cell's IndexTriangle faces.
        let face_vertices = cell.faces();

        // For each face of the cell, create a new cell.
        let new_tets: [TetrahedronKey; 4] = array_init(|i| {
            let tri = &face_vertices[i];

            // Allocate a new cell using the vertices of the face and the new vertex.
            let tet_id = self.cells.insert(IndexTetrahedron {
                points: [tri.points[0], tri.points[1], tri.points[2], new_vid],
                neighbours: [
                    // Copy the neighbour information from the previous cell.
                    // Note: we need to correct the back-reference later.
                    cell.neighbours[i],
                    // We fix the other neighbours later, since the exact way the topology
                    // ends up fitting together can be a bit tricky.
                    None,
                    None,
                    None,
                ],
            });

            // Ensure that the vertices properly refer back to the new cells.
            for pt in self.cells[tet_id].points.iter() {
                self.points[*pt].incident_cells.insert(tet_id);
            }

            // If the original cell had a neighbour on this face, chance that neighbour's
            // reference to the new cell.
            if let Some(nb) = cell.neighbours[i] {
                self.cells[nb.cell].neighbours[nb.face] = Some(FaceKey {
                    cell: tet_id,
                    face: 0, // Note the indexing earlier: the original triangle is face 0.
                });
            }

            // Keep the reference of the new cell.
            tet_id
        });

        // Ensure that the new tetrahedra correctly refer to each other.
        // Links to/from old cells have already been fixed at this point.
        self.fix_neighbours(&new_tets);

        // Return the references.
        (new_vid, new_tets)
    }

    /// An operation that brute-force iterates through all pairs of all faces of all given cells,
    /// and sets up neighbour links.
    ///
    /// Note this method does not require the TetrahedralMesh's invariants to be valid.
    fn fix_neighbours(&mut self, cells: &[TetrahedronKey]) {

        // I did say it was brute-force... Maybe there's a more elegant way?
        //
        // Really, these are just a few nested loops that
        // go over every possible pairing of faces among the cells, and links them up if
        // they share the same vertices (order-independent).
        for ta in cells.iter() {
            for (i, tra) in self.cells[*ta].faces().iter().enumerate() {
                for tb in cells.iter() {
                    if ta != tb { // Avoid self-pairing.
                        for (j, trb) in self.cells[*tb].faces().iter().enumerate() {
                            if tra.same_vertices(trb) {
                                self.cells[*ta].neighbours[i] = Some(FaceKey {
                                    cell: *tb,
                                    face: j,
                                })
                            }
                            // Could probably skip a few iterations here,
                            // but I'm too scared to call break; at this point.
                        }
                    }
                }
            }
        }
    }

    /// Turn an IndexTetrahedron into an ncollide3d::shape::Tetrahedron<f32>
    /// by looking up the vertices.
    fn extract_cell(&self, tt: &IndexTetrahedron) -> Tetrahedron<f32> {
        Tetrahedron {
            a: self.points[tt.points[0]].position,
            b: self.points[tt.points[1]].position,
            c: self.points[tt.points[2]].position,
            d: self.points[tt.points[3]].position,
        }
    }

    /// Create a vertex in the mesh, an allocate the appropriate bookkeeping structures,
    /// then return the resulting index.
    fn create_vertex(&mut self, pt: Point3<f32>) -> VertexKey {
        self.points.insert(Vertex { position: pt, incident_cells: HashSet::new() })
    }

    /// Starting at the given vertex, and moving along the provided direction,
    /// find the first tetrahedral cell encountered, if any.
    ///
    /// FIXME: This method has no concept of moving along a face or an edge,
    ///        in which case it may return either of the adjacent cells or None.
    ///        Should be fine if vertices are in general position, but still...
    pub fn walk_from_vertex(&self, vertex: VertexKey, direction: Vector3<f32>) -> Option<TetrahedronKey> {

        // Look up vertex position and create a ray.
        let vpos = self.points[vertex].position;
        let ray = Ray::new(vpos, direction);

        // Iterate over all adjacent faces and raycast against the far face,
        // returning the first hit.
        self.points[vertex].incident_cells.iter().cloned().find(|tk| {
            let tri: IndexTriangle = self.cells[*tk].far_side(vertex);
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

    /// Check the TetrahedralMesh's structural invariants, panicking if one is violated.
    /// Actual spatial properties are not checked.
    fn assert_invariants(mesh: &TetrahedralMesh) {

        // Iterate over every cell.
        for (tet_i, tet) in mesh.cells.iter() {

            // Assert that the cell's vertices refer back to it.
            for v_i in tet.points.iter() {
                assert!(mesh.points[*v_i].incident_cells.contains(&tet_i));
            }

            // Assert that the cell's neighbours, if any, refer back to the cell
            // with correct face indices.
            for (i, nb) in tet.neighbours.iter().enumerate() {
                if let Some(nb) = nb {
                    assert_eq!(mesh.cells[nb.cell].neighbours[nb.face], Some(FaceKey { cell: tet_i, face: i }));
                }
            }
        }

        // Make sure that the vertices don't refer to any missing cells,
        // and that these cells refer back to the vertex.
        for (p_i, pt) in mesh.points.iter() {
            for m_i in pt.incident_cells.iter() {
                assert!(mesh.cells[*m_i].points.contains(&p_i));
            }
        }
    }

    /// A test that repeatedly splits the TetrahedralMesh's cells, and checks the invariants.
    #[test]
    fn split_test() {
        let mut mesh = TetrahedralMesh::empty();
        let mut rng = thread_rng();

        let mut tet_refs: VecDeque<TetrahedronKey> = VecDeque::new();

        // Start with a single tetrahedron. Vertices are all (0,0,0), positions shouldn't matter.
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