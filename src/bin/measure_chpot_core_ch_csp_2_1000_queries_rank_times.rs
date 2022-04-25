use rand::Rng;
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        core_ch::CoreContractionHierarchy,
        csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
        dijkstra::{Dijkstra, DijkstraData},
    },
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::Instant,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let graph = OwnedGraph::new(first_out, head, travel_time);

    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let mut search = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());

    search.check();
    search.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);

    let log_num_nodes = (graph.num_nodes() as f32).log2() as usize;
    let dijkstra = Dijkstra::new(graph.borrow());
    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());

    let n = 100;
    let mut result = Vec::with_capacity(n);

    for _i in 0..n {
        print!("Progress {}/{}\r", _i, n);
        stdout().flush()?;

        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        search.init_new_s(s);
        dijkstra_state.init_new_s(s);

        let rank_order = dijkstra.ranks_only_exponentials(&mut dijkstra_state);

        let mut rank_times = Vec::with_capacity(log_num_nodes);
        for current_t in rank_order.into_iter() {
            let start = Instant::now();
            search.init_new_s(s);
            search.init_new_t(current_t);
            let _dist = search.run_query();
            let time = start.elapsed();
            rank_times.push(time);
        }
        result.push(rank_times);
    }
    println!("Progress {}/{}", n, n);

    let file = File::create("measure_chpot_core_ch_csp_2_1000_queries_rank_times-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
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
