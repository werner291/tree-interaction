use crate::tree_ds::ConstantRadiusTreeModel;
use nalgebra::Point3;
use std::env::temp_dir;
use std::fs::File;
use std::io::{LineWriter, Write, BufReader, BufRead};
use std::process::Command;
use std::str::FromStr;
use crate::tree_ds::put_tree_in_scene;

fn make_tetgen(vertices: &[Point3<f32>], edges: &[(usize, usize)]) -> std::io::Result<Vec<(usize, usize)>> {

    let mut dir = temp_dir();
    dir.push("tetgen_lines.poly");

    {
        let mut file = File::create(&dir).unwrap();

        writeln!(&mut file, "{} 3 0 0", vertices.len())?;

        for (i, pt) in vertices.iter().enumerate() {
            writeln!(&mut file, "{} {} {} {}", i, pt.x, pt.y, pt.z)?;
        }

        writeln!(&mut file, "{} 0", edges.len())?;

        for (j, k) in edges.iter() {
            writeln!(&mut file, "1")?;
            writeln!(&mut file, "2 {} {}", j, k)?;
        }

        writeln!(&mut file, "0");

        writeln!(&mut file, "0")?;
    }


    // exec!("tetgen -p {}", dir)
    Command::new("tetgen")
        .arg("-p")
        .arg(&dir)
        .output()
        .expect("failed to execute process");

    dir.pop();
    dir.push("tetgen_lines.1.edge");

    let mut reader = BufReader::new(File::open(&mut dir)?).lines();

    let header = reader.next().unwrap().unwrap();

    let num_edges = usize::from_str(&header.split(" ").next().unwrap()).unwrap();

    Ok((0..num_edges).map(|l| {
        dbg!(l);
        let line = reader.next().unwrap().unwrap();
        let mut numbers = line.split(" ").filter_map(|c| usize::from_str(c).ok());

        assert_eq!(numbers.next(), Some(l));
        let from_node = numbers.next().unwrap();
        let to_node = numbers.next().unwrap();

        (from_node, to_node)
    }).collect())
}

#[cfg(test)]
mod tests {
    use crate::tree_ds::{ConstantRadiusTreeModel, gen_tree, put_tree_in_scene};
    use nalgebra::Point3;
    use crate::tetgen::make_tetgen;
    use kiss3d::window::Window;
    use ncollide3d::transformation::ToTriMesh;

    #[test]
    fn view_test() {

        let tree_model = gen_tree(42);

        let points : Vec<Point3<f32>> = tree_model.nodes.iter().map(|node| node.position.clone()).collect();

        let edges : Vec<(usize, usize)> = tree_model.dfs().flat_map(|(node_id, node)| node.children.iter().map(move |child_id| (node_id, *child_id))).collect();

        let tet_edges = make_tetgen(&points, &edges).unwrap();
        let tet_edges = make_tetgen(&points, &edges).unwrap();

        let mut window = Window::new("Trees");

        let tree = gen_tree(42);

        // put_tree_in_scene(&mut window, &tree);

        // let as_compound : Compound<f32> = tree.into();

        while window.render() {
            for (i,j) in edges.iter() {
                window.draw_line(&points[*i], &points[*j], &Point3::new(1.0, 1.0, 1.0));
            }
        }

        // let points : Vec<f32> = tree_model.dfs().map(|n| n.1.position.clone()).collect();
        //
        // self.dfs().flat_map(move |(_, current)| {
        //     current.children.iter().cloned().map(move |c| {
        //         let p1 = current.position;
        //         let p2 = self.nodes[c].position;
        //
        //         (
        //             Isometry3::face_towards(&(p1 + (p2 - p1) * 0.5), &p2, &Vector3::z())
        //                 * Isometry3::rotation(Vector3::new(PI / 2.0, 0.0, 0.0)),
        //             Capsule::new(distance(&p1, &p2) / 2.0, self.radius),
        //         )
        //     })
        // })



    }

}