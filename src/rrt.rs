use std::ops::{Fn};
use ncollide3d::shape::Segment;

use nalgebra::{Point, DimName, Point3, Scalar, zero, distance, Vector3};
use std::prelude::v1::Vec;
use std::option::Option;
use std::option::Option::{None, Some};
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use std::cmp::PartialEq;
use std::convert::Into;
use ncollide3d::bounding_volume::AABB;
use rand::{thread_rng, Rng};

pub type PointIndex = usize;

struct PointData {
    point: Point3<f32>,
    from: Option<usize>,
    total_length: f32,
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Copy, Clone)]
enum Occupancy {
    Occupied,
    Free,
}

trait TestSegment {
    fn collides(&self, seg: Segment<f32>) -> Occupancy;
}

impl<F> TestSegment for F where F: Fn(Segment<f32>) -> Occupancy {
    fn collides(&self, seg: Segment<f32>) -> Occupancy {
        self(seg)
    }
}

struct RRT<Test: TestSegment> {
    explored_nodes: Vec<PointData>,
    spatial_lookup: KdTree<f32, usize, [f32; 3]>,
    validity_test: Test,
    step_size: f32,
}

impl<Test: TestSegment> RRT<Test> {

    fn new(start_from: Point3<f32>, validity_test: Test, step_size: f32) -> Self {
        let mut spatial_lookup = KdTree::new(3);
        spatial_lookup.add(start_from.coords.into(), 0);

        RRT {
            explored_nodes: vec![PointData {
                point: start_from,
                from: None,
                total_length: zero(),
            }],
            spatial_lookup,
            validity_test,
            step_size,
        }
    }

    fn explore_towards(&mut self, sample: &Point3<f32>) -> Option<PointIndex> {
        let closest = *self.spatial_lookup.nearest(sample.coords.as_slice(), 1, &squared_euclidean).unwrap().pop().unwrap().1;
        let closest_pos = self.explored_nodes[closest].point;

        let delta = sample - closest_pos;
        let distance_to_sample = delta.norm();

        let clamped_delta = if distance_to_sample > self.step_size {
            self.step_size * delta / distance_to_sample
        } else {
            delta
        };

        let after_step = closest_pos + clamped_delta;

        if self.validity_test.collides(Segment::new(closest_pos, after_step)) == Occupancy::Free {
            self.spatial_lookup.add(sample.coords.into(), self.explored_nodes.len()).unwrap();

            self.explored_nodes.push(PointData {
                point: after_step,
                from: Some(closest),
                total_length: clamped_delta.norm(),
            });

            Some(self.explored_nodes.len() - 1)
        } else {
            None
        }
    }

    fn explore_within(&mut self, aabb: &AABB<f32>) -> Option<usize> {
        let mut rng = thread_rng();
        self.explore_towards(&Point3::new(
            rng.gen_range(aabb.mins.x..aabb.maxs.x),
            rng.gen_range(aabb.mins.y..aabb.maxs.y),
            rng.gen_range(aabb.mins.z..aabb.maxs.z),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STEP_SIZE: f32 = 0.1;

    #[test]
    fn explore_straight_to_point() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let goal = Point3::new(10.0, 10.0, 10.0);

        let mut last_distance = distance(&start, &goal);

        let mut rrt = RRT::new(start, |_| Occupancy::Free, STEP_SIZE);

        while last_distance < 2.0 * STEP_SIZE {
            let result = rrt.explore_towards(&goal).expect("Should never fail without obstacles.");

            let is_at = &rrt.explored_nodes[result].point;

            let distance_remaining = distance(is_at, &goal);

            assert!(distance_remaining < last_distance);
            assert!(distance_remaining > last_distance + 2.0 * STEP_SIZE);

            last_distance = distance_remaining;
        }
    }
}