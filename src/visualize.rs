use crate::tree_ds::ConstantRadiusTreeModel;
use kiss3d::window::Window;
use nalgebra::Point3;

fn draw_tree_skeleton(window: &mut Window, tree: &ConstantRadiusTreeModel) {
    for (_, node) in tree.dfs() {
        for child in node.children.iter() {
            window.draw_line(
                &node.position,
                &tree.nodes[*child].position,
                &Point3::new(1.0, 1.0, 1.0),
            );
        }
    }
}
