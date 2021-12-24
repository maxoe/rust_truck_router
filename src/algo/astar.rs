use crate::types::*;

pub trait Potential {
    fn init_potentials(&mut self, potentials: &[Weight]);
    fn potential(&self, node: NodeId) -> Weight;
}

pub struct NoPotential();

impl Potential for NoPotential {
    fn init_potentials(&mut self, _potentials: &[Weight]) {}
    fn potential(&self, _node: NodeId) -> Weight {
        return 0;
    }
}
