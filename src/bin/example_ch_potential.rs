use stud_rust_base::{
    algo::{
        ch::*,
        ch_potential::CHPotentials,
        dijkstra::*,
        mcd::{OneRestrictionDijkstra, OwnedOneRestrictionGraph},
    },
    io::*,
    time::report_time,
    types::*,
};

use rand::Rng;
use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
};

// fn main() -> Result<(), Box<dyn Error>> {
//     let arg = &env::args().skip(1).next().expect("No directory arg given");
//     let path = Path::new(arg);

//     let first_out = Vec::<stud_rust_base::types::EdgeId>::load_from(path.join("first_out"))?;
//     let head = Vec::<stud_rust_base::types::NodeId>::load_from(path.join("head"))?;
//     let travel_time = Vec::<stud_rust_base::types::Weight>::load_from(path.join("travel_time"))?;
//     let graph = stud_rust_base::types::OwnedGraph::new(first_out, head, travel_time);

//     let mut instance = Dijkstra::new(graph.borrow());
//     let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
//     ch.check();
//     let mut ch_pot = CHPotentials::from_ch(ch);

//     let s = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);
//     let t = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);
//     ch_pot.init_target(t);
//     let mut pot_instance = Dijkstra::new_with_potential(graph.borrow(), ch_pot);
//     instance.init_new_s(s);
//     pot_instance.init_new_s(s);

//     println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

//     let dijkstra_dist = report_time("random dijkstra one-to-one distance query", || {
//         let dist = instance.dist_query(t);
//         println!("From {} to {}: {:?}", s, t, dist);
//         dist
//     });

//     let pot_dist = report_time("random ch potential one-to-one distance query", || {
//         let dist = pot_instance.dist_query(t);
//         println!("From {} to {}: {:?}", s, t, dist);
//         dist
//     });

//     assert_eq!(dijkstra_dist, pot_dist);

//     Ok(())
// }

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());

    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);

    let s = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);

    let mut instance = Dijkstra::new(graph.borrow());
    instance.init_new_s(s);
    report_time("random dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
        println!(
            "Nodes settled: {}, Labels propagated: {}, Queue pushes: {}",
            instance.num_settled, instance.num_labels_propagated, instance.num_queue_pushes
        );
    });

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let mut ch_pot = CHPotentials::from_ch(ch);
    ch_pot.init_target(t);
    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);
    let mut instance_mcd_acc = OneRestrictionDijkstra::new_with_potential(graph_mcd.borrow(), s, ch_pot);
    // let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);
    instance_mcd.set_reset_flags(is_parking_node.clone()).set_restriction(16_200_000, 3_600_000);
    instance_mcd_acc.set_reset_flags(is_parking_node).set_restriction(16_200_000, 3_600_000);

    report_time("random one restriction dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance_mcd.dist_query(t).iter().min());
        println!(
            "Nodes settled: {}, Labels propagated: {}, Labels reset: {}, Queue pushes: {}",
            instance_mcd.num_settled, instance_mcd.num_labels_propagated, instance_mcd.num_labels_reset, instance_mcd.num_queue_pushes
        );
    });

    report_time("random ch potential one restriction dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance_mcd_acc.dist_query(t).iter().min());
        println!(
            "Nodes settled: {}, Labels propagated: {}, Labels reset: {}, Queue pushes: {}",
            instance_mcd_acc.num_settled, instance_mcd_acc.num_labels_propagated, instance_mcd_acc.num_labels_reset, instance_mcd_acc.num_queue_pushes
        );
    });

    let latitude = Vec::<f32>::load_from(path.join("latitude"))?;
    let longitude = Vec::<f32>::load_from(path.join("longitude"))?;
    let osm_node_id = Vec::<u64>::load_from(path.join("osm_node_id"))?;
    assert_eq!(latitude.len(), longitude.len());
    assert_eq!(latitude.len(), graph_mcd.num_nodes());

    let shortest_mcd_path = report_time("Extracting path from one restriction dijkstra", || instance_mcd.current_best_path_to(t, true));
    let shortest_mcd_acc_path = report_time("Extracting path from one restriction ch_pot dijkstra", || {
        instance_mcd.current_best_path_to(t, true)
    });

    assert_eq!(shortest_mcd_path, shortest_mcd_acc_path);

    if let Some((p, d)) = shortest_mcd_path {
        println!("Path has length {}", p.len());

        let file = File::create("path_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude")?;

        for &node_id in &p {
            writeln!(file, "{},{}", latitude[node_id as usize], longitude[node_id as usize])?;
        }

        let flagged_p = instance_mcd.flagged_nodes_on_node_path(&p);
        println!("Number of flagged nodes is {}", flagged_p.len());

        let reset_p = instance_mcd.reset_nodes_on_path(&(p, d));
        println!("Number of reset nodes is {}", reset_p.len());

        let file = File::create("path_flagged_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude,osm_id,is_parking_used")?;

        for node_id in flagged_p {
            writeln!(
                file,
                "{},{},{},{}",
                latitude[node_id as usize],
                longitude[node_id as usize],
                osm_node_id[node_id as usize],
                reset_p.contains(&node_id)
            )?;
        }
    } else {
        println!("No path found from {} to {}", s, t)
    }

    Ok(())
}
