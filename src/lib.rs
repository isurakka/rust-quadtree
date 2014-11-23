extern crate "nalgebra" as na;
use na::{Vec2};
use std::option::{Option};
use std::cell::{RefCell};
use std::rc::{Rc};

struct AABB2<T> {
    lower_bound: Vec2<T>,
    upper_bound: Vec2<T>,
}

impl<T> AABB2<T> {
    fn new(lower_bound: Vec2<T>, upper_bound: Vec2<T>) -> AABB2<T> {
        AABB2 {
            lower_bound: lower_bound,
            upper_bound: upper_bound,
        }
    }
}

trait RegionQuadtreeEventHandler<T> {
    fn on_added(quadtree: &RegionQuadtree<T>);
    fn on_removing(quadtree: &RegionQuadtree<T>);
    fn on_modified(quadtree: &RegionQuadtree<T>);
}

pub enum Node<T> 
    where T: Clone 
{
    // Has four quadrants
    Children(
        Box<RegionQuadtree<T>>, // top left
        Box<RegionQuadtree<T>>, // top right
        Box<RegionQuadtree<T>>, // bottom right
        Box<RegionQuadtree<T>>),// bottom left
    // Leaf with value
    Full(T),
    // Empty leaf
    Empty,
}

pub struct RegionQuadtree<T>
    where T: Clone {
    resolution: u32,
    depth: u32,
    node: Node<T>,
    parent: Option<Box<RegionQuadtree<T>>>,
    aabb: AABB2<u32>,
    event_handler: Option<RegionQuadtreeEventHandler<T>>,
}

impl<T> RegionQuadtree<T> {
    pub fn new(resolution: u32) -> RegionQuadtree<T> {
        RegionQuadtree {
            resolution: resolution,
            depth: 0,
            node: Node::Empty,
            parent: None,
            //aabb: 
        }
    }
}

#[cfg(test)]
mod test {
    use super::RegionQuadtree;
}