//! (SLOW! AND EASY) Mapping for conversion of osm ids to local ids and vice versa.
//!
//! # Example
//!
//! ```
//! use bit_vec::BitVec;
//! use stud_rust_base::osm_id_mapper::*;
//!
//! let bv = BitVec::from_fn(5, |i| i == 1 || i == 2 || i == 4);
//!
//! for i in [1,2,4] {
//! 	assert_eq!(bv.to_osm(bv.to_local(i).unwrap()),i);
//! }
//!
//! for i in [0,3] {
//! 	assert_eq!(bv.to_local(i), None);
//! }
//!
//! ```

use crate::types::NodeId;
use bit_vec::BitVec;

pub trait OSMIDMapper {
    fn to_local(&self, osm_id: NodeId) -> Option<NodeId>;
    fn to_osm(&self, local_id: NodeId) -> NodeId;
}

impl OSMIDMapper for BitVec {
    fn to_local(&self, osm_id: NodeId) -> Option<NodeId> {
        if !self[osm_id as usize] {
            return None;
        }

        Some(self.iter().enumerate().filter(|(_, e)| *e).take_while(|(i, _)| *i < osm_id as usize).count() as NodeId)
    }

    fn to_osm(&self, local_id: NodeId) -> NodeId {
        self.iter().enumerate().filter(|(_, e)| *e).nth(local_id as usize).unwrap().0 as NodeId
    }
}
