use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

use num::range_step_inclusive;
use rand::Rng;
use rust_truck_router::{
    algo::{ch::ContractionHierarchy, core_ch::CoreContractionHierarchy, csp_core_ch::CSPCoreCHQuery, csp_core_ch_chpot::CSPAstarCoreCHQuery},
    experiments::measurement::{MeasurementResult, EXPERIMENTS_BASE_N},
    types::{Graph, NodeId, OwnedGraph, EU_SHORT_DRIVING_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    // let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    // let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

    let break_time_start = 0;
    let break_time_limit = 72_000_000;
    let break_time_step = 1_000_000;
    let n = EXPERIMENTS_BASE_N;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        pub time: Duration,
        pub max_break_time: u32,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo,time_ms,max_break_time";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!("{},{},{}", self.algo, self.time.as_secs_f64() * 1000.0, self.max_break_time)
        }
    }

    let mut stat_logs = Vec::with_capacity(n);
    let total_n = range_step_inclusive(break_time_start, break_time_limit, break_time_step).count() * EXPERIMENTS_BASE_N;

    let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    let mut core_ch_query = CSPCoreCHQuery::new(core_ch.borrow());

    for (i, bt) in range_step_inclusive(break_time_start, break_time_limit, break_time_step).enumerate() {
        core_ch_query.set_restriction(EU_SHORT_DRIVING_TIME, bt);
        core_ch_chpot_query.set_restriction(EU_SHORT_DRIVING_TIME, bt);

        for j in 0..n {
            let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
            let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

            print!("\rProgress {}/{} from {} to {} - Core CH        ", i * n + j, total_n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            core_ch_query.init_new_s(s);
            core_ch_query.init_new_t(t);
            let _core_ch_dist = core_ch_query.run_query();
            let core_ch_time = start.elapsed();

            print!("\rProgress {}/{} from {} to {} - A* Core CH\t\t\t\t\t\t\t", i * n + j, total_n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(t);
            let _core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                time: core_ch_time,
                max_break_time: bt,
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                time: core_ch_chpot_time,
                max_break_time: bt,
            });
        }
    }

    println!("\rProgress {}/{}", n, n);

    let file = File::create("thesis_break_times-csp-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
