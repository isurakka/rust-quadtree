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

pub enum Node<'a, T: 'a> 
    where T: Copy + PartialEq
{
    // Has four quadrants
    Children([Box<RegionQuadtree<'a, T>>; 4]),
    // Leaf with value
    Full(T),
    // Empty leaf
    Empty,
}

pub struct RegionQuadtree<'a, T: 'a>
    where T: Copy + PartialEq {
    resolution: u32,
    depth: u32,
    node: Node<'a, T>,
    aabb: AABB2<u32>,
    parent: Option<&'a RegionQuadtree<'a, T>>,
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
            depth: 0,
            node: initial_value.map_or(Node::Empty, |v| Node::Full(v)),
            aabb: AABB2::new(Vec2::new(0, 0), Vec2::new(size.unwrap(), size.unwrap())),
            parent: None,
            event_handler: None,
        })
    }

    fn new_child(resolution: u32, depth: u32, value: Option<T>, parent: &'a RegionQuadtree<'a, T>, aabb: AABB2<u32>) 
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
            parent: Some(parent),
            event_handler: None,
        }
    }

    fn calculate_size(resolution: u32) -> Option<u32> {
        let size = (2 as u32).pow(resolution as usize);
        match size {
            0 => None,
            _ => Some(size)
        }
    }

    fn get_type(&self) -> (NodeType, Option<T>) {
        match self.node {
            Node::Children(_) => (NodeType::Grey, None),
            Node::Full(value) => (NodeType::Black, Some(value)),
            Node::Empty => (NodeType::White, None),
        }
    }

    fn get_resolution(&self) -> u32 {
        self.resolution
    }

    fn get_aabb(&self) -> AABB2<u32> {
        self.aabb
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

            let par_opt = par.parent;
        }
    }

    pub fn set(&mut self, value: Option<T>) -> bool {
        let anySet = self.set_internal(value);
        if (anySet)
        {
            //unsubdivide()
        }
        anySet
    }

    fn set_internal(&mut self, opt_value: Option<T>) -> bool {
        let black = match self.node {
            Node::Full(v) => Some(v),
            _ => None
        };

        match opt_value.is_some() {
            true => {
                let value = opt_value.unwrap();

                if (black.is_some()) {
                    let old_value = black.unwrap();
                    if (old_value == value) {
                        return false;
                    } else {
                        self.node = Node::Full(value);
                        self.propagate_event(EventType::Changed, None, Some(old_value));
                        return true;
                    }
                }

                //unset();
                self.node = Node::Full(value);
                self.propagate_event(EventType::Added, None, None);

                return true
            },
            false => { 
                if (black.is_some()) {
                    self.propagate_event(EventType::Removing, None, None);
                    self.node = Node::Empty;
                    return true;
                }

                match self.node {
                    Node::Empty => return false,
                    _ => (),
                };

                match self.node {
                    Node::Children(ref mut children) => {
                        let mut any = false;
                        for c in children.iter_mut() {
                            any |= c.set_internal(opt_value);
                        }
                        return any;
                    },
                    _ => ()
                };
            },
        };
        
        false
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