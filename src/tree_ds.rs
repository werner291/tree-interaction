use std::iter::Iterator;
use std::marker::Copy;
use std::option::Option;
use std::prelude::v1::Vec;

use nalgebra::{distance, Isometry3, Point3, Vector3};
use nalgebra::{Rotation3, Unit};
use ncollide3d::shape::{Capsule, Compound, ShapeHandle};
use rand::prelude::{SliceRandom, IteratorRandom};
use rand::{Rng, SeedableRng};
use slotmap::{new_key_type, SlotMap};
use std::convert::From;
use std::f32::consts::PI;

use rand_pcg::Pcg64;
use kiss3d::window::Window;
use ncollide3d::transformation::ToTriMesh;

type TreeNodeKey = usize;

//
// new_key_type! {
//     pub struct TreeNodeKey;
// }
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
    pub nodes: Vec<TreeNode>,
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

    pub(crate) fn transformed_capsules(
        &self,
    ) -> impl Iterator<Item=(Isometry3<f32>, Capsule<f32>)> + '_ {
        self.dfs().flat_map(move |(_, current)| {
            current.children.iter().cloned().map(move |c| {
                let p1 = current.position;
                let p2 = self.nodes[c].position;

                (
                    Isometry3::face_towards(&(p1 + (p2 - p1) * 0.5), &p2, &Vector3::z())
                        * Isometry3::rotation(Vector3::new(PI / 2.0, 0.0, 0.0)),
                    Capsule::new(distance(&p1, &p2) / 2.0, self.radius),
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

    // let mut nodes: SlotMap<TreeNodeKey, TreeNode> = SlotMap::with_key();
    let mut nodes = Vec::new();

    let root = 0;
    nodes.push(TreeNode {
        position: Point3::new(0.0, 0.0, 0.0),
        children: vec![],
    });

    // let mut node_sampling_pool = vec![root];

    for _ in 0..20 {
        let parent = (0..nodes.len())
            .choose(&mut rng)
            .expect("Should never be empty.");

        let delta = Rotation3::from_axis_angle(
            &Unit::new_unchecked(Vector3::y()),
            rng.gen_range(0.0..(2.0 * PI)),
        ) * Rotation3::from_axis_angle(
            &Unit::new_unchecked(Vector3::x()),
            rng.gen_range(0.0..1.0),
        ) * Vector3::new(0.0, 1.0, 0.0);

        let child_pos = &nodes[parent].position + delta;

        nodes.push(TreeNode {
            position: child_pos,
            children: vec![],
        });

        let child_id = nodes.len() - 1;

        nodes[parent].children.push(child_id);
        // node_sampling_pool.push(child_node);
    }

    ConstantRadiusTreeModel {
        nodes,
        radius: 0.1,
        root,
    }
}

impl From<&ConstantRadiusTreeModel> for Compound<f32> {
    fn from(tm: &ConstantRadiusTreeModel) -> Self {
        Compound::new(
            tm.transformed_capsules()
                .map(|(tf, caps)| (tf, ShapeHandle::new(caps)))
                .collect(),
        )
    }
}

pub fn put_tree_in_scene(window: &mut Window, tree: &ConstantRadiusTreeModel) {
    for (tf, child) in tree.transformed_capsules() {
        let mut node =
            window.add_trimesh(child.to_trimesh((5, 5)), Vector3::new(1.0, 1.0, 1.0));
        node.set_local_transformation(tf);
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
