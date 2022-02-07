use crate::types::*;

pub trait Potential<W: WeightOps> {
    fn init_potentials(&mut self, potentials: &[W]);
    fn potential(&mut self, node: NodeId) -> W;
    fn init_new_t(&mut self, t: NodeId);
}

pub struct NoPotential {}

impl<W> Potential<W> for NoPotential
where
    W: WeightOps,
{
    fn init_potentials(&mut self, _potentials: &[W]) {}
    fn potential(&mut self, _node: NodeId) -> W {
        W::zero()
    }
    fn init_new_t(&mut self, _t: NodeId) {}
}
