use rand::{Rng, SeedableRng};
use rust_truck_router::{
    algo::{
        ch::{ContractionHierarchy, ContractionHierarchyQuery},
        ch_potential::CHPotential,
        core_ch::{CoreContractionHierarchy, CoreContractionHierarchyQuery},
        csp::*,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir::CSP2BidirAstarCHPotQuery,
        csp_2_core_ch::CSP2CoreCHQuery,
        csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
        csp_bidir::CSPBidirAstarCHPotQuery,
        csp_core_ch::CSPCoreCHQuery,
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
        dijkstra::{Dijkstra, DijkstraData},
    },
    io::*,
    types::*,
};
use std::{error::Error, path::Path};

#[test]
#[ignore]
fn thousand_ka_queries_without_constraints() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);

    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
    let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    let mut csp_2_pot_state = TwoRestrictionDijkstraData::new(graph.num_nodes());

    let mut core_ch_csp_query = CSPCoreCHQuery::new(core_ch.borrow());
    let mut core_ch_csp_2_query = CSP2CoreCHQuery::new(core_ch.borrow());

    let mut core_ch_chpot_csp_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    let mut core_ch_chpot_csp_2_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());

    let mut bidir_csp_query = CSPBidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    let mut bidir_csp_2_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());

    let mut ch_query = ContractionHierarchyQuery::new(ch.borrow());
    let mut core_ch_query = CoreContractionHierarchyQuery::new(core_ch.borrow());

    let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
    let csp_2_pot = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
    let dijkstra = Dijkstra::new(graph.borrow());

    for i in 0..100000 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);

        println!("Query #{} from {} to {} without constraints", i, s, t);
        dijkstra_state.init_new_s(s);
        csp_pot_state.init_new_s(s);
        csp_2_pot_state.init_new_s(s);

        core_ch_csp_query.init_new_s(s);
        core_ch_csp_query.init_new_t(t);
        core_ch_csp_2_query.init_new_s(s);
        core_ch_csp_2_query.init_new_t(t);

        core_ch_chpot_csp_query.init_new_s(s);
        core_ch_chpot_csp_query.init_new_t(t);
        core_ch_chpot_csp_2_query.init_new_s(s);
        core_ch_chpot_csp_2_query.init_new_t(t);

        bidir_csp_query.init_new_s(s);
        bidir_csp_query.init_new_t(t);
        bidir_csp_2_query.init_new_s(s);
        bidir_csp_2_query.init_new_t(t);

        ch_query.init_new_s(s);
        ch_query.init_new_t(t);

        core_ch_query.init_new_s(s);
        core_ch_query.init_new_t(t);

        let dijkstra_dist = dijkstra.dist_query(&mut dijkstra_state, t);
        let csp_dist = csp_pot.dist_query(&mut csp_pot_state, t);
        let csp_2_dist = csp_2_pot.dist_query(&mut csp_2_pot_state, t);

        let core_ch_csp_dist = core_ch_csp_query.run_query();
        let core_ch_csp_2_dist = core_ch_csp_2_query.run_query();

        let core_ch_chpot_csp_dist = core_ch_chpot_csp_query.run_query();
        let core_ch_chpot_csp_2_dist = core_ch_chpot_csp_2_query.run_query();

        let bidir_csp_dist = bidir_csp_query.run_query();
        let bidir_csp_2_dist = bidir_csp_2_query.run_query();

        let ch_dist = ch_query.run_query();
        let core_ch_dist = core_ch_query.run_query();

        assert_eq!(dijkstra_dist, csp_dist);
        assert_eq!(dijkstra_dist, csp_2_dist);
        assert_eq!(dijkstra_dist, core_ch_csp_dist);
        assert_eq!(dijkstra_dist, core_ch_csp_2_dist);
        assert_eq!(dijkstra_dist, core_ch_chpot_csp_dist);
        assert_eq!(dijkstra_dist, core_ch_chpot_csp_2_dist);
        assert_eq!(dijkstra_dist, bidir_csp_dist);
        assert_eq!(dijkstra_dist, bidir_csp_2_dist);
        assert_eq!(dijkstra_dist, ch_dist);
        assert_eq!(dijkstra_dist, core_ch_dist);

        // let dijkstra_path = dijkstra_state.current_node_path_to(t);
        // let csp_path = csp_pot_state.current_best_node_path_to(t);
        // let csp_2_path = csp_2_pot_state.current_best_node_path_to(t);

        // assert_eq!(dijkstra_path, csp_path);
        // assert_eq!(dijkstra_path, csp_2_path);
    }

    Ok(())
}

#[test]
#[ignore]
fn thousand_ka_queries_csp() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let ch_pot = CHPotential::from_ch(ch.borrow());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let max_driving_time = 4_000_000;
    let pause_time = 270_000;

    let mut csp_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    csp_state.set_restriction(max_driving_time, pause_time);

    let mut csp_pot_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), ch_pot.clone());
    csp_pot_state.set_restriction(max_driving_time, pause_time);

    let mut csp_pot_prop_all_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), ch_pot);
    csp_pot_prop_all_state.set_restriction(max_driving_time, pause_time);

    let mut core_ch_csp_query = CSPCoreCHQuery::new(core_ch.borrow());
    core_ch_csp_query.set_restriction(max_driving_time, pause_time);

    let mut core_ch_chpot_csp_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_csp_query.set_restriction(max_driving_time, pause_time);

    let mut bidir_csp_query = CSPBidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_csp_query.set_restriction(max_driving_time, pause_time);

    let csp = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    for i in 0..100000 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        // let s = 339025;
        // let t = 125566;
        println!("CSP Query #{} from {} to {}", i, s, t);

        csp_state.init_new_s(s);
        csp_pot_state.init_new_s(s);
        csp_pot_prop_all_state.init_new_s(s);

        core_ch_csp_query.init_new_s(s);
        core_ch_csp_query.init_new_t(t);

        core_ch_chpot_csp_query.init_new_s(s);
        core_ch_chpot_csp_query.init_new_t(t);

        bidir_csp_query.init_new_s(s);
        bidir_csp_query.init_new_t(t);

        let csp_dist = csp.dist_query(&mut csp_state, t);
        let csp_pot_dist = csp.dist_query(&mut csp_pot_state, t);
        let csp_pot_prop_all_dist = csp.dist_query_propagate_all_labels(&mut csp_pot_prop_all_state, t);

        let core_ch_csp_dist = core_ch_csp_query.run_query();
        let core_ch_chpot_csp_dist = core_ch_chpot_csp_query.run_query();

        let bidir_csp_dist = bidir_csp_query.run_query();

        assert_eq!(csp_dist, csp_pot_dist);
        assert_eq!(csp_dist, csp_pot_prop_all_dist);
        assert_eq!(csp_dist, core_ch_csp_dist);
        assert_eq!(csp_dist, core_ch_chpot_csp_dist);
        assert_eq!(csp_dist, bidir_csp_dist);

        // let csp_path = csp_pot_state.current_best_node_path_to(t);
        // let csp_prop_all_path = csp_pot_prop_all_state.current_best_node_path_to(t);

        // assert_eq!(csp_path, csp_prop_all_path);
    }

    Ok(())
}

#[test]
#[ignore]
fn thousand_ka_queries_csp_2() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let ch_pot = CHPotential::from_ch(ch.borrow());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let max_driving_time_short = 4_000_000;
    let pause_time_short = 100_000;
    let max_driving_time_long = 6_000_000;
    let pause_time_long = 500_000;

    let mut csp_2_state = TwoRestrictionDijkstraData::new(graph.num_nodes());
    csp_2_state.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut csp_2_pot_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), ch_pot.clone());
    csp_2_pot_state.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut csp_2_pot_prop_all_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), ch_pot);
    csp_2_pot_prop_all_state.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut core_ch_csp_2_query = CSP2CoreCHQuery::new(core_ch.borrow());
    core_ch_csp_2_query.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut core_ch_chpot_csp_2_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_csp_2_query.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut bidir_csp_2_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_csp_2_query.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let csp_2 = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    for i in 0..100000 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        println!("CSP2 Query #{} from {} to {} ", i, s, t);

        csp_2_state.init_new_s(s);
        csp_2_pot_state.init_new_s(s);
        csp_2_pot_prop_all_state.init_new_s(s);

        core_ch_csp_2_query.init_new_s(s);
        core_ch_csp_2_query.init_new_t(t);

        core_ch_chpot_csp_2_query.init_new_s(s);
        core_ch_chpot_csp_2_query.init_new_t(t);

        bidir_csp_2_query.init_new_s(s);
        bidir_csp_2_query.init_new_t(t);

        let csp_2_dist = csp_2.dist_query(&mut csp_2_state, t);
        let csp_2_pot_dist = csp_2.dist_query(&mut csp_2_pot_state, t);
        let csp_2_pot_prop_all_dist = csp_2.dist_query_propagate_all_labels(&mut csp_2_pot_prop_all_state, t);

        let core_ch_csp_2_dist = core_ch_csp_2_query.run_query();
        let core_ch_chpot_csp_2_dist = core_ch_chpot_csp_2_query.run_query();

        let bidir_csp_dist = bidir_csp_2_query.run_query();

        assert_eq!(csp_2_dist, csp_2_pot_dist);
        assert_eq!(csp_2_dist, csp_2_pot_prop_all_dist);
        assert_eq!(csp_2_dist, core_ch_csp_2_dist);
        assert_eq!(csp_2_dist, core_ch_chpot_csp_2_dist);
        assert_eq!(csp_2_dist, bidir_csp_dist);

        // let csp_2_path = csp_2_pot_state.current_best_node_path_to(t);
        // let csp_2_prop_all_path = csp_2_pot_prop_all_state.current_best_node_path_to(t);

        // assert_eq!(csp_2_path, csp_2_prop_all_path);
    }

    Ok(())
}
