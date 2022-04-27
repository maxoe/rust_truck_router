use std::{error::Error, path::Path};

use bit_vec::BitVec;
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        ch_potential::CHPotential,
        csp::*,
        dijkstra::{Dijkstra, DijkstraData},
    },
    io::*,
    types::{OwnedGraph, *},
};

use rand::{Rng, SeedableRng};

#[test]
fn some_ch_pot_queries() -> Result<(), Box<dyn Error>> {
    // 0 -> 1 -> 2p -> 4
    //      | -> 3p -> |
    let first_out = vec![0, 1, 3, 4, 5, 5];
    let head = vec![1, 2, 3, 4, 4];
    let travel_time = vec![1, 4, 3, 2, 4];

    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/"));
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
    let dijkstra = Dijkstra::new(graph.borrow());
    let mut dijkstra_pot_state = DijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let dijkstra_pot = Dijkstra::new(graph.borrow());
    let mut instance_mcd_acc_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
    let instance_mcd_acc = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    for s in 0..5 {
        dijkstra_state.init_new_s(s);
        dijkstra_pot_state.init_new_s(s);
        instance_mcd_acc_state.init_new_s(s);
        for t in 0..5 {
            let dijkstra_dist = dijkstra.dist_query(&mut dijkstra_state, t);
            assert_eq!(dijkstra_dist, instance_mcd_acc.dist_query_propagate_all_labels(&mut instance_mcd_acc_state, t));
            assert_eq!(dijkstra_dist, dijkstra_pot.dist_query(&mut dijkstra_pot_state, t));

            let dijkstra_path = dijkstra_state.current_node_path_to(t);
            assert_eq!(dijkstra_path, dijkstra_pot_state.current_node_path_to(t));
            assert_eq!(dijkstra_path, instance_mcd_acc_state.current_best_node_path_to(t));
        }
    }

    Ok(())
}

/// Careful, can produce false positive if random query has an ambiguous shortest path
#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
    let dijkstra = Dijkstra::new(graph.borrow());
    let mut dijkstra_pot_state = DijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let dijkstra_pot = Dijkstra::new(graph.borrow());
    let mut instance_mcd_acc_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
    let instance_mcd_acc = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542290214824);

    for i in 0..100 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        println!("Query #{} from {} to {}", i, s, t);

        dijkstra_state.init_new_s(s);
        dijkstra_pot_state.init_new_s(s);
        instance_mcd_acc_state.init_new_s(s);

        let dijkstra_dist = dijkstra.dist_query(&mut dijkstra_state, t);
        assert_eq!(dijkstra_dist, instance_mcd_acc.dist_query_propagate_all_labels(&mut instance_mcd_acc_state, t));
        assert_eq!(dijkstra_dist, dijkstra_pot.dist_query(&mut dijkstra_pot_state, t));

        let dijkstra_path = dijkstra_state.current_node_path_to(t);

        assert_eq!(dijkstra_path, dijkstra_pot_state.current_node_path_to(t));
        assert_eq!(dijkstra_path, instance_mcd_acc_state.current_best_node_path_to(t));
    }

    Ok(())
}
