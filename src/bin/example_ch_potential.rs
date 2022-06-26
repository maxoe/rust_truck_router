use rust_truck_router::{
    algo::{
        ch::*,
        ch_potential::CHPotential,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir_chpot::CSP2BidirAstarCHPotQuery,
    },
    io::*,
    types::*,
};

use rand::Rng;
use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
    time::Instant,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    // let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(graph.borrow());

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    // let max_driving_time_short = 4_000_000;
    // let pause_time_short = 100_000;
    // let max_driving_time_long = 6_000_000;
    // let pause_time_long = 500_000;

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let mut csp2_astar_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    csp2_astar_state.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    // csp2_astar_state.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut csp2_bidir_astar_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    csp2_bidir_astar_query.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    // csp2_core_ch_chpot_query.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    csp2_astar_state.init_new_s(s);
    csp2_bidir_astar_query.init_new_s(s);
    csp2_bidir_astar_query.init_new_t(t);

    let csp2_astar = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
    let timer = Instant::now();
    let unidir_dist = csp2_astar.dist_query(&mut csp2_astar_state, t);
    println!("Elapsed: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
    print!("{}", csp2_astar.summary(&csp2_astar_state));

    let timer = Instant::now();
    let bidir_dist = csp2_bidir_astar_query.run_query();
    println!("Elapsed bidirectional: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
    print!("{}", csp2_bidir_astar_query.summary());

    assert_eq!(unidir_dist, bidir_dist);

    // instance_mcd_acc.reset();
    // instance_mcd_acc.dist_query_propagate_all_labels(t);
    // print!("{}", instance_mcd_acc.info());

    let latitude = Vec::<f32>::load_from(path.join("latitude"))?;
    let longitude = Vec::<f32>::load_from(path.join("longitude"))?;
    let osm_node_id = Vec::<u64>::load_from(path.join("osm_node_id"))?;
    assert_eq!(latitude.len(), longitude.len());
    assert_eq!(latitude.len(), graph.num_nodes());

    if let Some((p, _d)) = csp2_astar_state.current_best_path_to(t, true) {
        let file = File::create("path_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude")?;

        for &node_id in &p {
            writeln!(file, "{},{}", latitude[node_id as usize], longitude[node_id as usize])?;
        }

        let file = File::create("path_flagged_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude,osm_id,is_parking_used")?;
        let used_p = csp2_astar.reset_nodes_on_path(&(p, _d));
        for &node_id in used_p.0.iter() {
            writeln!(
                file,
                "{},{},{},{}",
                latitude[node_id as usize],
                longitude[node_id as usize],
                osm_node_id[node_id as usize],
                used_p.0.contains(&node_id)
            )?;
        }

        // let (reset_p_short, reset_p_long) = instance_mcd_acc.reset_nodes_on_path(&(p, d));
        // let file = File::create("path_flagged_acc.csv")?;
        // let mut file = LineWriter::new(file);
        // writeln!(file, "latitude,longitude,osm_id,is_parking_short_break,is_parking_long_break")?;

        // for node_id in flagged_p {
        //     writeln!(
        //         file,
        //         "{},{},{},{},{}",
        //         latitude[node_id as usize],
        //         longitude[node_id as usize],
        //         osm_node_id[node_id as usize],
        //         reset_p_short.contains(&node_id),
        //         reset_p_long.contains(&node_id)
        //     )?;
        // }
    }

    let file = File::create("labels_acc.csv")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "latitude,longitude,num_labels")?;

    for (i, n) in csp2_astar_state
        .get_per_node_number_of_labels()
        .into_iter()
        .enumerate()
        .filter(|(_, n)| *n != 0)
    {
        writeln!(file, "{},{},{}", latitude[i], longitude[i], n)?;
    }

    Ok(())
}
