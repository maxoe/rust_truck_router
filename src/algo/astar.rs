use crate::types::*;

pub trait Potential<W: WeightOps> {
    fn init_potentials(&mut self, potentials: &[W]);
    fn potential(&self, node: NodeId) -> W;
}

pub struct NoPotential {}

impl<W> Potential<W> for NoPotential
where
    W: WeightOps,
{
    fn init_potentials(&mut self, _potentials: &[W]) {}
    fn potential(&self, _node: NodeId) -> W {
        W::zero()
    }
}
