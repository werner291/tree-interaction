#![feature(array_map)]

mod free_space_map;
mod geometry_utilities;
mod rrt;
mod tetrahedral_mesh;
mod tree_ds;
mod visualize;
mod tetgen;

// impl ToTriMesh<f32> for ConstantRadiusTreeModel {
//     type DiscretizationParameter = usize;
//
//     fn to_trimesh(&self, i: Self::DiscretizationParameter) -> TriMesh<f32> {
//         let Compunself
//     }
// }

#[cfg(test)]
mod tests {
    use crate::tree_ds::{gen_tree, ConstantRadiusTreeModel, put_tree_in_scene};
    use kiss3d::window::Window;
    use ncollide3d::transformation::ToTriMesh;

    use nalgebra::Vector3;

    #[test]
    fn test_visualize() {
        let mut window = Window::new("Trees");

        let tree = gen_tree(42);

        put_tree_in_scene(&mut window, &tree);

        // let as_compound : Compound<f32> = tree.into();

        while window.render() {
            // draw_tree_skeleton(&mut window, &tree)
        }
    }


}
