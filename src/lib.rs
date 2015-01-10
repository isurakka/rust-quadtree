extern crate "nalgebra" as na;
use na::{Vec2};
use std::option::{Option};
use std::cell::{RefCell};
use std::rc::{Rc, Weak};
use std::num::{Int, UnsignedInt};

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
    fn on_added(&self, quadtree: &RegionQuadtree<T>);
    fn on_removing(&self, quadtree: &RegionQuadtree<T>);
    fn on_changed(&self, quadtree: &RegionQuadtree<T>);
}

pub enum Node<'a, T> 
    where T: Clone 
{
    // Has four quadrants
    Children(
        Box<RegionQuadtree<'a, T>>, // top left
        Box<RegionQuadtree<'a, T>>, // top right
        Box<RegionQuadtree<'a, T>>, // bottom right
        Box<RegionQuadtree<'a, T>>),// bottom left
    // Leaf with value
    Full(T),
    // Empty leaf
    Empty,
}

pub struct RegionQuadtree<'a, T>
    where T: Clone {
    resolution: u32,
    depth: u32,
    node: Node<'a, T>,
    aabb: AABB2<u32>,
    //parent: Option<Weak<&'a RegionQuadtree<'a, T>>>,
    event_handler: Option<&'a (RegionQuadtreeEventHandler<T> + 'a)>,
}

enum EventType {
    Added,
    Removing,
    Changed,
}

impl<'a, T> RegionQuadtree<'a, T>
    where T: Clone {
    pub fn new(resolution: u32) -> RegionQuadtree<'a, T> {
        let size = RegionQuadtree::<T>::calculate_size(resolution);
        RegionQuadtree {
            resolution: resolution,
            depth: 0,
            node: Node::Empty,
            aabb: AABB2::new(Vec2::new(0, 0), Vec2::new(size, size)),
            event_handler: None,
        }
    }

    fn new_child(resolution: u32, depth: u32, value: Option<T>, aabb: AABB2<u32>) 
        -> RegionQuadtree<'a, T> 
    {
        let node = match value {
            Some(v) => Node::Full(v),
            None => Node::Empty,
        };

        RegionQuadtree {
            resolution: resolution,
            depth: depth,
            node: node,
            aabb: aabb,
            event_handler: None,
        }
    }

    fn calculate_size(resolution: u32) -> u32 {
        (2 as u32).pow(resolution as usize)
    }

    fn propagate_event(&self, event_type: EventType, start: Option<&RegionQuadtree<'a, T>>, old_value: Option<T>)
    {
        let always = match start {
            Some(v) => v,
            None => self
        };

        let par_opt = match start {
            Some(v) => Some(v),
            None => Some(self)
        };

        while !par_opt.is_none() {
            let par = &par_opt.unwrap();
            if par.event_handler.is_none() {
                continue;
            }
            let event_handler = par.event_handler.unwrap();

            match event_type {
                EventType::Added    => event_handler.on_added(always),
                EventType::Removing => event_handler.on_removing(always),
                EventType::Changed  => event_handler.on_changed(always),
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::RegionQuadtree;
}