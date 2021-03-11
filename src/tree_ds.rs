use std::iter::Iterator;
use std::marker::Copy;
use std::option::Option;
use std::prelude::v1::Vec;

use nalgebra::{Point3, Vector3, Isometry3, distance};
use slotmap::{new_key_type, SlotMap};
use rand::prelude::SliceRandom;
use nalgebra::{Rotation3, Unit};
use rand::{Rng, SeedableRng};
use std::f32::consts::PI;
use std::convert::{From, Into};
use ncollide3d::shape::{Compound, ShapeHandle, Ball, Cylinder, Capsule};
use std::option::Option::Some;
use ncollide3d::transformation::ToTriMesh;
use ncollide3d::procedural::TriMesh;
use rand_pcg::Pcg64;


//
new_key_type! {
    pub struct TreeNodeKey;
}
//
// pub struct TreeNode<D> {
//     data: D,
//     children: Vec<TreeNodeKey>
// }
//
// pub struct Tree<D> {
//     nodes: SlotMap<TreeNodeKey, D>,
//     root: TreeNodeKey
// }
//
// struct Tree<Point3<f32>>

pub struct TreeNode {
    pub position: Point3<f32>,
    pub children: Vec<TreeNodeKey>,
}

pub struct ConstantRadiusTreeModel {
    pub nodes: SlotMap<TreeNodeKey, TreeNode>,
    pub radius: f32,
    pub root: TreeNodeKey,
}

impl ConstantRadiusTreeModel {
    pub(crate) fn dfs(&self) -> Dfs {
        Dfs {
            tree: self,
            stack: vec![self.root],
        }
    }

    pub(crate) fn transformed_capsules(&self) -> impl Iterator<Item=(Isometry3<f32>, Capsule<f32>)> + '_ {
        self.dfs().flat_map(move |(_, current)| {
            current.children.iter().cloned().map(move |c| {
                let p1 = current.position;
                let p2 = self.nodes[c].position;

                (
                    Isometry3::face_towards(&(p1 + (p2 - p1) * 0.5), &p2, &Vector3::z()) * Isometry3::rotation(Vector3::new(PI / 2.0, 0.0, 0.0)),
                    Capsule::new(distance(&p1, &p2) / 2.0, self.radius)
                )
            })
        })
    }
}

pub struct Dfs<'a> {
    tree: &'a ConstantRadiusTreeModel,
    stack: Vec<TreeNodeKey>,
}

impl<'a> Iterator for Dfs<'a> {
    type Item = (TreeNodeKey, &'a TreeNode);

    fn next(&mut self) -> Option<Self::Item> {
        self.stack.pop().map(|current| {
            let curr_data = &self.tree.nodes[current];
            self.stack.extend_from_slice(&curr_data.children);
            (current, curr_data)
        })
    }
}

pub fn gen_tree(seed: u64) -> ConstantRadiusTreeModel {
    let mut rng = Pcg64::seed_from_u64(seed);

    let mut nodes: SlotMap<TreeNodeKey, TreeNode> = SlotMap::with_key();

    let root = nodes.insert(TreeNode {
        position: Point3::new(0.0, 0.0, 0.0),
        children: vec![],
    });

    let mut node_sampling_pool = vec![root];

    for _ in 0..20 {
        let parent = *node_sampling_pool.choose(&mut rng).expect("Should never be empty.");

        let delta = Rotation3::from_axis_angle(&Unit::new_unchecked(Vector3::y()), rng.gen_range(0.0..(2.0 * PI))) *
            Rotation3::from_axis_angle(&Unit::new_unchecked(Vector3::x()), rng.gen_range(0.0..1.0)) *
            Vector3::new(0.0, 1.0, 0.0);

        dbg!(&delta);

        let child_pos = nodes[parent].position + delta;

        let child_node = nodes.insert(TreeNode {
            position: child_pos,
            children: vec![],
        });

        nodes[parent].children.push(child_node);
        node_sampling_pool.push(child_node);
    }

    ConstantRadiusTreeModel {
        nodes,
        radius: 0.1,
        root,
    }
}

impl From<&ConstantRadiusTreeModel> for Compound<f32> {
    fn from(tm: &ConstantRadiusTreeModel) -> Self {
        Compound::new(tm.transformed_capsules().map(|(tf, caps)| (tf, ShapeHandle::new(caps))).collect())
    }
}

// impl ToTriMesh<f32> for ConstantRadiusTreeModel {
//     type DiscretizationParameter = usize;
//
//     fn to_trimesh(&self, i: Self::DiscretizationParameter) -> TriMesh<f32> {
//
//         let coords = Vec::new();
//         let indices = Vec::new();
//
//         for (tf,caps) in self.transformed_capsules() {
//             let sub_mesh : TriMesh<f32> = caps.to_trimesh();
//
//             coords.extend_from_slice(sub_mesh.)
//         }
//
//         unimplemented!()
//     }
// }