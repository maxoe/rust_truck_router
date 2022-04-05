use std::{error::Error, path::Path};

use rust_truck_router::{
    algo::{ch::ContractionHierarchy, ch_potential::CHPotential, dijkstra::Dijkstra, mcd::*},
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

    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    let graph_mcd = FirstOutGraph::new(first_out, head, travel_time);
    let mut dijkstra = Dijkstra::new(&graph);
    let ch_pot = CHPotential::from_ch(ch);
    let mut dijkstra_pot = Dijkstra::new_with_potential(&graph, ch_pot.clone());
    let mut instance_mcd_acc = OneRestrictionDijkstra::new_with_potential(&graph_mcd, ch_pot);

    for s in 0..5 {
        dijkstra.init_new_s(s);
        dijkstra_pot.init_new_s(s);
        instance_mcd_acc.init_new_s(s);
        for t in 0..5 {
            let dijkstra_dist = dijkstra.dist_query(t);
            assert_eq!(dijkstra_dist, instance_mcd_acc.dist_query(t));
            assert_eq!(dijkstra_dist, dijkstra_pot.dist_query(t));

            let dijkstra_path = dijkstra.current_node_path_to(t);
            assert_eq!(dijkstra_path, dijkstra_pot.current_node_path_to(t));
            assert_eq!(dijkstra_path, instance_mcd_acc.current_best_node_path_to(t));
        }
    }

    Ok(())
}

// #[test]
// fn test_reset() -> Result<(), Box<dyn Error>> {
//     let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/"));
//     let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
//     ch.check();

//     let mut ch_pot = CHPotential::from_ch(ch);
//     ch_pot.init_new_t(4);
//     ch_pot.reset();

//     Ok(())
// }

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
    let graph_mcd = FirstOutGraph::new(first_out, head, travel_time);
    let mut dijkstra = Dijkstra::new(&graph);
    let ch_pot = CHPotential::from_ch(ch);
    let mut dijkstra_pot = Dijkstra::new_with_potential(&graph, ch_pot.clone());
    let mut instance_mcd_acc = OneRestrictionDijkstra::new_with_potential(&graph_mcd, ch_pot);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542290214824);

    for i in 0..100 {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {}", i, s, t);

        dijkstra.init_new_s(s);
        dijkstra_pot.init_new_s(s);
        instance_mcd_acc.init_new_s(s);

        let dijkstra_dist = dijkstra.dist_query(t);
        assert_eq!(dijkstra_dist, instance_mcd_acc.dist_query(t));
        assert_eq!(dijkstra_dist, dijkstra_pot.dist_query(t));

        let dijkstra_path = dijkstra.current_node_path_to(t);

        assert_eq!(dijkstra_path, dijkstra_pot.current_node_path_to(t));
        assert_eq!(dijkstra_path, instance_mcd_acc.current_best_node_path_to(t));
    }

    Ok(())
}
