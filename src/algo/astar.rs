use crate::types::*;

pub trait Potential {
    fn potential(&mut self, node: NodeId) -> Weight;
    fn init_new_t(&mut self, t: NodeId);
}

#[derive(Clone)]
pub struct NoPotential {}

impl Potential for NoPotential {
    fn potential(&mut self, _node: NodeId) -> Weight {
        Weight::zero()
    }
    fn init_new_t(&mut self, _t: NodeId) {}
}
