use rand::Rng;
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::Instant,
};
use stud_rust_base::{
    algo::{ch::*, ch_potential::CHPotential, dijkstra::Dijkstra, mcd_2::TwoRestrictionDijkstra},
    io::*,
    types::*,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let mut search = TwoRestrictionDijkstra::new_with_potential(graph.borrow(), CHPotential::from_ch(ch));
    // search.set_reset_flags(is_parking_node.to_bytes()).set_restriction(16_200_000, 2_700_000);
    search.set_reset_flags(is_parking_node.to_bytes());

    let log_num_nodes = (graph.num_nodes() as f32).log2() as usize;
    let mut dijkstra = Dijkstra::new(graph.borrow());

    let n = 1000;
    let mut result = Vec::with_capacity(n);

    for _i in 0..n {
        print!("Progress {}/{}\r", _i, n);
        stdout().flush()?;

        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        search.init_new_s(s);
        dijkstra.init_new_s(s);
        let rank_order = dijkstra.ranks_only_exponentials();

        let mut rank_times = Vec::with_capacity(log_num_nodes);
        for current_t in rank_order {
            let start = Instant::now();
            search.init_new_s(s);
            search.dist_query(current_t);
            rank_times.push(start.elapsed());
        }
        result.push(rank_times);
    }
    println!("Progress {}/{}", n, n);

    let file = File::create("measure_csp_2_1000_queries_rank_times-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);

    writeln!(
        file,
        "{}",
        (0..log_num_nodes).map(|d| u32::pow(2, d as u32).to_string()).collect::<Vec<_>>().join(",")
    )?;
    for one_result in result {
        writeln!(
            file,
            "{}",
            one_result
                .into_iter()
                .map(|d| (d.as_secs_f64() * 1000.0).to_string())
                .collect::<Vec<_>>()
                .join(",")
        )?;
    }

    Ok(())
}
