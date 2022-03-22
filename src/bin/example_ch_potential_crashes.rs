use stud_rust_base::{
    algo::{ch::*, ch_potential::CHPotential, mcd_2::TwoRestrictionDijkstra},
    io::*,
    osm_id_mapper::OSMIDMapper,
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
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);

    let s = 49041870; //rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);
    let t = 9384566; //rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);

    let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // println!("{}", is_routing_node.to_osm(53010882));
    //println!("s: {}", is_routing_node.to_osm(s));
    //println!("t: {}", is_routing_node.to_osm(t));
    // path with distance 20517304
    //let s = is_routing_node.to_local(382554273).unwrap();
    //let t = is_routing_node.to_local(253856110).unwrap(); // osm_id

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    // let mut instance_mcd_acc = OneRestrictionDijkstra::new_with_potential(graph_mcd.borrow(), CHPotential::from_ch(ch));
    // instance_mcd_acc
    //     .set_reset_flags(is_parking_node.to_bytes())
    //     .set_restriction(16_200_000, 1_950_000);
    let mut instance_mcd_acc = TwoRestrictionDijkstra::new_with_potential(graph_mcd.borrow(), CHPotential::from_ch(ch));
    instance_mcd_acc
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(32_400_000, 32_400_000, 16_200_000, 270_000);

    let timer = Instant::now();
    instance_mcd_acc.init_new_s(s);
    instance_mcd_acc.dist_query(t);
    println!("Elapsed: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
    //print!("{}", instance_mcd_acc.info());

    // instance_mcd_acc.reset();
    // instance_mcd_acc.dist_query_propagate_all_labels(t);
    // print!("{}", instance_mcd_acc.info());

    let latitude = Vec::<f32>::load_from(path.join("latitude"))?;
    let longitude = Vec::<f32>::load_from(path.join("longitude"))?;
    let osm_node_id = Vec::<u64>::load_from(path.join("osm_node_id"))?;
    assert_eq!(latitude.len(), longitude.len());
    assert_eq!(latitude.len(), graph_mcd.num_nodes());

    let timer = Instant::now();
    if let Some((p, _d)) = instance_mcd_acc.current_best_path_to(t, true) {
        println!("Elapsed: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
        let file = File::create("path_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude")?;

        for &node_id in &p {
            writeln!(file, "{},{}", latitude[node_id as usize], longitude[node_id as usize])?;
        }

        let file = File::create("path_flagged_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude,osm_id,is_parking_used")?;
        let used_p = instance_mcd_acc.reset_nodes_on_path(&(p, _d));
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

    for (i, n) in instance_mcd_acc
        .get_per_node_number_of_labels()
        .into_iter()
        .enumerate()
        .filter(|(_, n)| *n != 0)
    {
        writeln!(file, "{},{},{}", latitude[i], longitude[i], n)?;
    }

    Ok(())
}
