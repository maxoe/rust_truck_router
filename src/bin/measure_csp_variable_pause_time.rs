use stud_rust_base::{
    algo::{ch::ContractionHierarchy, ch_potential::CHPotential, mcd::*},
    io::*,
    osm_id_mapper::OSMIDMapper,
    types::*,
};

use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg).canonicalize()?;
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let mut search = OneRestrictionDijkstra::new_with_potential(graph_mcd.borrow(), CHPotential::from_ch(ch));
    search.set_reset_flags(is_parking_node.to_bytes());

    let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    //let s = 422258;
    let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    search.init_new_s(s);
    // let t = 4548361;
    let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    let n = 1;
    //leads to 4 breaks
    let max_driving_dist = 5_000_000;
    let pause_duration_start = 0;
    let pause_duration_stop = 36_000_000;
    let pause_duration_step = 30_000;

    #[derive(Debug, Clone, Copy)]
    struct MeasurementResult {
        pub pause_duration_ms: u32,
        pub num_queue_pushes: u32,
        pub num_settled: u32,
        pub num_labels_propagated: u32,
        pub num_labels_reset: u32,
        pub num_nodes_searched: usize,
        pub time: Duration,
        pub path_distance: Option<Weight>,
        pub path_number_nodes: Option<usize>,
        pub path_number_flagged_nodes: Option<usize>,
        pub path_number_pauses: Option<usize>,
    }

    impl MeasurementResult {
        const HEADER : &'static str= "pause_duration_ms,num_queue_pushes,num_settled,num_labels_propagated,num_labels_reset,num_nodes_searched,time_ms,path_distance,path_number_nodes,path_number_flagged_nodes,path_number_pauses";

        pub fn as_csv(&self) -> String {
            if self.path_distance.is_some() {
                format!(
                    "{},{},{},{},{},{},{},{},{},{},{}",
                    self.pause_duration_ms,
                    self.num_queue_pushes,
                    self.num_settled,
                    self.num_labels_propagated,
                    self.num_labels_reset,
                    self.num_nodes_searched,
                    self.time.as_secs_f64() * 1000.0,
                    self.path_distance.unwrap(),
                    self.path_number_nodes.unwrap(),
                    self.path_number_flagged_nodes.unwrap(),
                    self.path_number_pauses.unwrap()
                )
            } else {
                format!(
                    "{},{},{},{},{},{},{},{},{},{},{}",
                    self.pause_duration_ms,
                    self.num_queue_pushes,
                    self.num_settled,
                    self.num_labels_propagated,
                    self.num_labels_reset,
                    self.num_nodes_searched,
                    self.time.as_secs_f64() * 1000.0,
                    "NaN",
                    "NaN",
                    "NaN",
                    "NaN",
                )
            }
        }
    }

    let it = (pause_duration_start..pause_duration_stop + 1).step_by(pause_duration_step);
    let number_of_steps = it.clone().count() * n;

    let mut results = Vec::with_capacity(number_of_steps);
    for (i, current_pause_duration) in it.enumerate() {
        for j in 0..n {
            println!("Query {}/{}", i * n + j + 1, number_of_steps);
            search.set_restriction(max_driving_dist, current_pause_duration);
            search.reset();

            let start = Instant::now();
            let result = search.dist_query(t);
            let time = start.elapsed();

            // println!("{}", search.info());

            if let Some(dist) = result {
                let path = search.current_best_path_to(t, true).unwrap();
                let number_flagged_nodes = search.flagged_nodes_on_path(&path);
                let number_pauses = search.reset_nodes_on_path(&path);

                results.push(MeasurementResult {
                    pause_duration_ms: current_pause_duration,
                    num_queue_pushes: search.num_queue_pushes,
                    num_settled: search.num_settled,
                    num_labels_propagated: search.num_labels_propagated,
                    num_labels_reset: search.num_labels_reset,
                    num_nodes_searched: search.get_number_of_visited_nodes(),
                    time,
                    path_distance: Some(dist),
                    path_number_nodes: Some(path.0.len()),
                    path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    path_number_pauses: Some(number_pauses.len()),
                });
            } else {
                results.push(MeasurementResult {
                    pause_duration_ms: current_pause_duration,
                    num_queue_pushes: search.num_queue_pushes,
                    num_settled: search.num_settled,
                    num_labels_propagated: search.num_labels_propagated,
                    num_labels_reset: search.num_labels_reset,
                    num_nodes_searched: search.get_number_of_visited_nodes(),
                    time,
                    path_distance: None,
                    path_number_nodes: None,
                    path_number_flagged_nodes: None,
                    path_number_pauses: None,
                });
            }
        }
    }

    let file = File::create("measure_csp_variable_pause_time-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", MeasurementResult::HEADER)?;
    for r in results {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
