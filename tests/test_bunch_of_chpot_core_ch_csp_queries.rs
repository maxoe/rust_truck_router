use bit_vec::BitVec;
use rand::{Rng, SeedableRng};
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        core_ch::CoreContractionHierarchy,
        csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
    },
    io::{load_routingkit_bitvector, Load},
    types::{EdgeId, Graph, NodeId, OwnedGraph, Weight},
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
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow());

    csp_pot_state.set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes());

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
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow());
    csp_pot_state.set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes());

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

#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let max_driving_time = 4_000_000;
    let pause_time = 2_700_000;
    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let mut core_ch = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch.check();
    core_ch.set_restriction(max_driving_time, pause_time);

    let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow());
    csp_pot_state
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(max_driving_time, pause_time);

    for i in 0..100 {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {} with constraint", i, s, t);
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
