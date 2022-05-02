use bit_vec::BitVec;
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        core_ch::CoreContractionHierarchy,
        csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
    },
    types::{Graph, OwnedGraph},
};
use std::{error::Error, path::Path};

#[test]
fn test_instance_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let mut core_ch = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch.check();
    let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    let is_parking_node = BitVec::from_fn(5, |i| i == 2 || i == 3);
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let max_restriction = 10;
    let pause_time = 5;

    for s in 0..5 {
        for t in 0..5 {
            for max_driving_time in 1..(max_restriction + 1) {
                println!("Testing {} -> {}; max_driving_time: {}", s, t, max_driving_time);
                let s = 0;
                let t = 4;
                let max_driving_time = 5;
                core_ch.init_new_s(s);
                core_ch.init_new_t(t);
                core_ch.set_restriction(max_driving_time, pause_time);
                let dist = core_ch.run_query();

                csp_pot_state.init_new_s(s);
                csp_pot_state.set_restriction(max_driving_time, pause_time);
                let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

                assert_eq!(dist, csp_pot_dist);
            }
        }
    }

    Ok(())
}

#[test]
fn test_needs_two_breaks_instance() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance_2"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let mut core_ch = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch.check();

    let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    let is_parking_node = BitVec::from_fn(5, |i| i == 2 || i == 3);
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let max_restriction = 10;
    let pause_time = 5;

    for (s, t) in (0..5).zip(0..5) {
        for max_driving_time in 1..(max_restriction + 1) {
            // println!("Testing {} -> {}; max_driving_time: {}", s, t, max_driving_time);
            core_ch.init_new_s(s);
            core_ch.init_new_t(t);
            core_ch.set_restriction(max_driving_time, pause_time);
            let dist = core_ch.run_query();

            csp_pot_state.init_new_s(s);
            csp_pot_state.set_restriction(max_driving_time, pause_time);
            let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

            assert_eq!(dist, csp_pot_dist);
        }
    }

    Ok(())
}
