extern crate "nalgebra" as na;
use na::{Vec2};
use std::option::{Option};
use std::cell::{RefCell};
use std::rc::{Rc, Weak};
use std::num::{Int, UnsignedInt};
use std::ops::{Sub};

#[derive(Copy)]
struct AABB2<T> 
    where T: Copy + Sub<T> {
    lower_bound: Vec2<T>,
    upper_bound: Vec2<T>,
}

impl<T> AABB2<T>
    where T: Copy + Sub<T> {
    fn new(lower_bound: Vec2<T>, upper_bound: Vec2<T>) -> AABB2<T> {
        AABB2 {
            lower_bound: lower_bound,
            upper_bound: upper_bound,
        }
    }

    pub fn get_width(&self) -> <T as Sub>::Output {
        self.upper_bound.x - self.lower_bound.x
    }

    pub fn get_height(&self) -> <T as Sub>::Output {
        self.upper_bound.y - self.lower_bound.y
    }
}

trait RegionQuadtreeEventHandler<T> {
    fn on_added(&self, quadtree: &RegionQuadtree<T>);
    fn on_removing(&self, quadtree: &RegionQuadtree<T>);
    fn on_changed(&self, quadtree: &RegionQuadtree<T>);
}

pub struct RegionQuadtree<'a, T: 'a>
    where T: Copy + PartialEq {
    nodes: Vec<Option<T>>,
    resolution: u32,
    event_handler: Option<&'a (RegionQuadtreeEventHandler<T> + 'a)>,
}

enum EventType {
    Added,
    Removing,
    Changed,
}

enum NodeType {
    Black,
    White,
    Grey
}

impl<'a, T> RegionQuadtree<'a, T>
    where T: Copy + PartialEq {
    pub fn new(resolution: u32, initial_value: Option<T>) -> Option<RegionQuadtree<'a, T>> {
        let size = RegionQuadtree::<T>::calculate_size(resolution);
        if (size.is_none())
        {
            return None;
        }
        Some(RegionQuadtree {
            resolution: resolution,
            nodes: vec![None, size],
            event_handler: None,
        })
    }

    fn calculate_size(resolution: u32) -> Option<u32> {
        let size = 4 * resolution + 4;

        if (size <= 0)
        {
            return None;
        }
        
        Some(size)
    }

    fn get_resolution(&self) -> u32 {
        self.resolution
    }

    fn get_aabb(&self, index: usize) -> AABB2<u32> {
        let mut aabb_topleft = Vec2<u32>::new(0, 0);
        let mut decr_index = index;
        let full_aabb = self.get_aabb(0);
        let index_res = (index / 4.0).floor();
        let size = Vec2::new(
            full_aabb.get_width() / index_res, 
            full_aabb.get_height() / index_res);
        for r in index_res..0 {
            let pos = (decr_index - 1) % 4;
            aabb_topleft += Vec2::new(
                full_aabb.get_width() / r, 
                full_aabb.get_height() / r);
        }
        self.aabb
    }

    fn propagate_event(&self, event_type: EventType, 
        start: Option<&RegionQuadtree<'a, T>>, old_value: Option<T>)
    {
        
    }

    pub fn set(&mut self, value: Option<T>) -> bool {


        false
    }

    fn subdivide(&'a mut self) -> bool {
        let value = match self.node {
            Node::Full(v) => Some(v),
            _ => None
        };

        if (value.is_some()) {
            self.propagate_event(EventType::Removing, None, None);
        }

        self.node = Node::Children(RefCell::new([None, None, None, None]));

        let self_rc = Rc::new(&*self);

        self.node = Node::Children(RefCell::new([
            Some(Rc::new(RegionQuadtree::<T>::new_child(self.resolution, 
                self.depth + 1, value, self_rc.clone(),
                AABB2::new(
                    self.aabb.lower_bound, 
                    self.aabb.lower_bound + Vec2::new(self.aabb.get_width() / 2, self.aabb.get_height() / 2)))).downgrade()),
            Some(Rc::new(RegionQuadtree::<T>::new_child(self.resolution, 
                self.depth + 1, value, self_rc.clone(),
                AABB2::new(
                    self.aabb.lower_bound, 
                    self.aabb.lower_bound + Vec2::new(self.aabb.get_width() / 2, self.aabb.get_height() / 2)))).downgrade()),
            Some(Rc::new(RegionQuadtree::<T>::new_child(self.resolution, 
                self.depth + 1, value, self_rc.clone(),
                AABB2::new(
                    self.aabb.lower_bound, 
                    self.aabb.lower_bound + Vec2::new(self.aabb.get_width() / 2, self.aabb.get_height() / 2)))).downgrade()),
            Some(Rc::new(RegionQuadtree::<T>::new_child(self.resolution, 
                self.depth + 1, value, self_rc.clone(),
                AABB2::new(
                    self.aabb.lower_bound, 
                    self.aabb.lower_bound + Vec2::new(self.aabb.get_width() / 2, self.aabb.get_height() / 2)))).downgrade()),
            ]));

        true
    }
}

#[cfg(test)]
mod test {
    use super::RegionQuadtree;

    #[test]
    fn new_different_resolutions()
    {
        let qt1 = RegionQuadtree::new(3, Some(3u32)).unwrap();
        assert_eq!(8, qt1.get_aabb().get_width());

        let qt2 = RegionQuadtree::new(-1, Some(3u32));
        assert!(qt2.is_none());

        let qt3 = RegionQuadtree::new(31, Some(3u32));
        assert!(qt3.is_some());

        let qt4 = RegionQuadtree::new(32, Some(3u32));
        assert!(qt4.is_none());
    }
}